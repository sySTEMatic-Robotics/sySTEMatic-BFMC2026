import time
import threading
import config
from multiprocessing import Process

# --- TRY IMPORTING EVDEV ---
try:
    import evdev
    from evdev import ecodes, InputDevice, list_devices
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    print("MANUAL ERROR: 'evdev' not installed. Run 'pip install evdev'")

# --- DEBUG SETTINGS ---
DEBUG_MODE = True 

class ManualControlState:
    def __init__(self):
        self.steer = 0.0
        self.accel = 0.0
        self.brake = 0.0
        self.connected = False

def manual_drive_process(serial_queue, shared_auto_mode, shared_state_array, shared_recording):
    if not EVDEV_AVAILABLE:
        return

    state = ManualControlState()
    cont = 500
    
    # Anti-spam variables
    last_sent_steer = -999
    last_sent_speed = -999
    last_sent_time = 0
    was_braking = False

    # --- Listener Thread ---
    def monitor_inputs():
        device = None
        absinfo_printed_for_this_connection = False
        
        while True:
            # 1. CONNECT TO CONTROLLER
            if device is None:
                state.connected = False
                absinfo_printed_for_this_connection = False
                try:
                    devices = [InputDevice(path) for path in list_devices()]
                    for dev in devices:
                        name = dev.name.lower()
                        if "xbox" in name or "gamepad" in name or "microsoft" in name:
                            device = dev
                            print(f"MANUAL: Connected to {dev.name}")
                            state.connected = True
                            break
                except:
                    pass
                
                if device is None:
                    time.sleep(1)
                    continue

            # 2. READ INPUTS
            try:
                for event in device.read_loop():
                    
                    # --- 1. STEERING (Left Stick X) ---
                    if event.type == ecodes.EV_ABS and event.code == 0:
                        state.steer = event.value / 32768.0 -1
                        # SAFETY OVERRIDE: If user steers, disable AI
                        if abs(state.steer) > 0.1: 
                            if shared_auto_mode.value:
                                shared_auto_mode.value = False
                                print("MANUAL: Override! Auto Mode -> OFF")

                    # --- 2. ACCELERATION (Right Trigger / RT) ---
                    elif event.type == ecodes.EV_ABS and event.code == 9: # Check your code (9 is often RT on xpadneo)
                        raw = event.value
                        state.accel = raw / 1023.0
                        # SAFETY OVERRIDE: If user accelerates, disable AI
                        if state.accel > 0.1:
                            if shared_auto_mode.value:
                                shared_auto_mode.value = False
                                print("MANUAL: Override! Auto Mode -> OFF")

                    # --- 3. AUTO MODE TOGGLE (A Button) ---
                    # Standard Code: 304 (BTN_SOUTH / A)
                    elif event.type == ecodes.EV_KEY and event.code == 304:
                        if event.value == 1: # Pressed
                            shared_auto_mode.value = not shared_auto_mode.value
                            status = "ON" if shared_auto_mode.value else "OFF"
                            print(f"MANUAL: Auto Mode Toggled -> {status}")

                    # --- 4. RECORDING (X Button) ---
                    # Standard Code: 307 (BTN_WEST / X)
                    elif event.type == ecodes.EV_KEY and event.code == 307:
                        if event.value == 1: 
                            shared_recording.value = not shared_recording.value
                            print(f"MANUAL: Recording Toggled -> {shared_recording.value}")

                    # --- 5. BRAKE (B Button) ---
                    elif event.type == ecodes.EV_KEY and event.code == 305:
                        state.brake = 1.0 if event.value == 1 else 0.0
                        # SAFETY OVERRIDE: Braking kills AI
                        if state.brake > 0.5 and shared_auto_mode.value:
                            shared_auto_mode.value = False
                            print("MANUAL: Braking Override! Auto Mode -> OFF")

            except Exception as e:
                print(f"MANUAL: Controller Disconnected ({e})")
                device = None
                state.connected = False
                time.sleep(1)

    t = threading.Thread(target=monitor_inputs, daemon=True)
    t.start()
    print("MANUAL PROCESS: Started.")

    # --- Sender Loop ---
    while True:
        # Update Visuals (Shared Array)
        shared_state_array[0] = state.steer
        shared_state_array[1] = state.accel
        shared_state_array[2] = state.brake

        # ONLY SEND COMMANDS IF AUTO MODE IS OFF
        # This prevents the manual script from fighting the AI script
        if state.connected and not shared_auto_mode.value:
            
            # A. Steering
            steer_val = 0.0 if abs(state.steer) < config.STEER_DEADZONE else state.steer
            steer_cmd = int(steer_val * config.MAX_STEER_ANGLE)

            # B. Speed / Brake Logic
            is_braking = state.brake > 0.5
            
            if is_braking:
                speed_cmd = 0
                if not was_braking or (time.time() - last_sent_time > 0.2):
                    try:
                        serial_queue.put(f'#steer:{steer_cmd};;\r\n')
                        serial_queue.put('#brake:0;;\r\n')
                        last_sent_time = time.time()
                    except: pass
            else:
                accel_clean = max(0.0, min(1.0, state.accel))
                speed_cmd = int(accel_clean * config.MAX_SPEED)

                steer_changed = abs(steer_cmd - last_sent_steer) > 1 
                speed_changed = abs(speed_cmd - last_sent_speed) > 2
                is_heartbeat = (time.time() - last_sent_time) > 0.2
                
                if steer_changed or speed_changed or is_heartbeat or was_braking:
                    try:
                        serial_queue.put(f'#steer:{steer_cmd};;\r\n')
                        serial_queue.put(f'#speed:{speed_cmd};;\r\n')
                        
                        last_sent_steer = steer_cmd
                        last_sent_speed = speed_cmd
                        last_sent_time = time.time()
                    except: pass

            was_braking = is_braking
        
        time.sleep(0.02)