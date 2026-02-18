from gpiozero import DigitalInputDevice
import time

# --- NEW HELPER CLASS ---
class SmartSerial:
    def __init__(self, queue):
        self.queue = queue
        self.last_speed = -999
        self.last_steer = -999

        # --- ESC reverse arming tuning ---
        self.NEUTRAL_DELAY = 0.30      # try 0.30, then 0.40 if needed
        self.BRAKE_PULSE = 20          # small forward pulse; try 20-35 if needed
        self.BRAKE_PULSE_DELAY = 0.20  # duration of the brake pulse

    def send(self, speed, steer):
        """Only sends commands if they have changed.
        Adds a robust ESC reverse-arming sequence for forward->reverse transitions.
        Enforces 100ms delay between Speed and Steer commands."""
        
        # Clean inputs
        if speed is not None:
            speed = int(speed)
        if steer is not None:
            steer = int(steer)

        speed_sent = False # Flag to track if we touched the serial port for speed

        # --- FIX: Robust Forward -> Reverse arming sequence ---
        # Many car ESCs require: neutral -> brake pulse -> neutral -> reverse
        if speed is not None and self.last_speed != -999:
            if self.last_speed > 0 and speed < 0:
                # 1) Neutral
                self.queue.put('#speed:0;;\r\n')
                self.last_speed = 0
                time.sleep(self.NEUTRAL_DELAY)

                # 2) Brake/arming pulse (small forward)
                self.queue.put(f'#speed:{self.BRAKE_PULSE};;\r\n')
                self.last_speed = self.BRAKE_PULSE
                time.sleep(self.BRAKE_PULSE_DELAY)

                # 3) Neutral again
                self.queue.put('#speed:0;;\r\n')
                self.last_speed = 0
                time.sleep(self.NEUTRAL_DELAY)
                
                speed_sent = True

        # Send SPEED only if changed
        if speed is not None and speed != self.last_speed:
            self.queue.put(f'#speed:{speed};;\r\n')
            self.last_speed = speed
            speed_sent = True

        # Send STEER only if changed
        if steer is not None and steer != self.last_steer:
            # [FIX] If we just sent a speed command, wait 100ms before sending steer
            if speed_sent:
                time.sleep(0.1)
                
            self.queue.put(f'#steer:{steer};;\r\n')
            self.last_steer = steer

    def force_stop(self):
        self.queue.put('#speed:0;;\r\n')
        self.last_speed = 0


class ParkingSystem:
    def __init__(self):
        # Sensors
        self.sensor_r = DigitalInputDevice(4, pull_up=False)
        self.sensor_l = DigitalInputDevice(27, pull_up=False)
        self.sensor_f = DigitalInputDevice(22, pull_up=False)
        self.sensor_b = DigitalInputDevice(17, pull_up=False)

        # Serial Manager (Initialized in update)
        self.serial_manager = None

        # State Constants
        self.STATE_IDLE = "IDLE"
        self.STATE_SCANNING = "SCANNING"
        self.STATE_MOVING_NEXT = "MOVE_NEXT"
        self.STATE_EXECUTING_ROUTINE = "EXECUTING"

        # Internal Variables
        self.current_state = self.STATE_IDLE
        self.state_start_time = 0.0
        self.parking_sign_cooldown = 0.0

        # --- THE MAGIC LIST ---
        self.routine_steps = []
        self.current_step_index = 0
        self.step_start_time = 0.0

        # Settings
        self.SCAN_DURATION = 3.0
        self.RETRY_MOVE_DURATION = 1.0
        self.PARKING_SPEED = 50

    def trigger_scan(self):
        if self.current_state == self.STATE_IDLE and time.time() > self.parking_sign_cooldown:
            print(">>> PARKING SIGN: SCANNING <<<")
            self.current_state = self.STATE_SCANNING
            self.state_start_time = time.time()
            return True
        return False

    def update(self, serial_queue):
        # Initialize SmartSerial once we have the queue
        if self.serial_manager is None:
            self.serial_manager = SmartSerial(serial_queue)

        now = time.time()
        elapsed = now - self.state_start_time

        # 1. SCANNING (3s)
        if self.current_state == self.STATE_SCANNING:
            if elapsed < self.SCAN_DURATION:
                # Use SmartSerial to prevent flooding "speed:50"
                self.serial_manager.send(self.PARKING_SPEED, None)
                # Return True to block YOLO speed control, but None so main loop doesn't double-send
                return True, None, None
            else:
                return self._check_sensors_and_decide(now, is_retry=False)

        # 2. MOVING TO NEXT SPOT (1s)
        elif self.current_state == self.STATE_MOVING_NEXT:
            if elapsed < self.RETRY_MOVE_DURATION:
                self.serial_manager.send(self.PARKING_SPEED, None)
                return True, None, None
            else:
                return self._check_sensors_and_decide(now, is_retry=True)

        # 3. EXECUTING THE LIST OF STEPS
        elif self.current_state == self.STATE_EXECUTING_ROUTINE:
            return self._process_routine_step(now)

        return False, None, None

    def _process_routine_step(self, now):
        """Reads the current step and uses SmartSerial to send commands."""
        if self.current_step_index >= len(self.routine_steps):
            print(">>> ROUTINE COMPLETE <<<")
            self.reset()
            return False, None, None

        speed, steer, duration = self.routine_steps[self.current_step_index]
        step_elapsed = now - self.step_start_time

        if step_elapsed < duration:
            self.serial_manager.send(speed, steer)
            return True, None, None
        else:
            # Move to next step
            self.current_step_index += 1
            self.step_start_time = now
            return self._process_routine_step(now)

    def _check_sensors_and_decide(self, now, is_retry=False):
        left_free = (self.sensor_l.value == 1)
        right_free = (self.sensor_r.value == 1)
        print(f"Sensors -> L:{self.sensor_l.value} R:{self.sensor_r.value}")

        if left_free:
            print(">>> LEFT SPOT FOUND! <<<")
            self._start_left_routine()
            return self._process_routine_step(now)

        elif right_free:
            print(">>> RIGHT SPOT FOUND! <<<")
            self._start_right_routine()
            return self._process_routine_step(now)

        else:
            if not is_retry:
                print(">>> OCCUPIED. NEXT SPOT... <<<")
                self.current_state = self.STATE_MOVING_NEXT
                self.state_start_time = now
                self.serial_manager.send(self.PARKING_SPEED, None)
                return True, None, None
            else:
                print(">>> ABORT PARKING <<<")
                self.reset()
                return False, None, None

    # ==========================================
    #  ROUTINES
    # ==========================================
    def _start_right_routine(self):
        print("STARTING RIGHT ROUTINE")
        self.current_state = self.STATE_EXECUTING_ROUTINE
        self.current_step_index = 0
        self.step_start_time = time.time()

        self.routine_steps = [
            (50, 0, 1.5),
            (-50, 230, 3.5),
            (-50, -230, 3),
            (0, 0, 1)
        ]

    def _start_left_routine(self):
        print("STARTING LEFT ROUTINE")
        self.current_state = self.STATE_EXECUTING_ROUTINE
        self.current_step_index = 0
        self.step_start_time = time.time()

        self.routine_steps = [
            (50, 0, 1.5),
            (-50, 210, 3.5),
            (-50, -210, 3),
            (0, 0, 1)
        ]

    def reset(self):
        self.current_state = self.STATE_IDLE
        self.parking_sign_cooldown = time.time() + 10.0