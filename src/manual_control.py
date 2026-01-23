import time
import threading
import config
from multiprocessing import Process

try:
    from inputs import get_gamepad, devices
    INPUTS_AVAILABLE = True
except ImportError:
    INPUTS_AVAILABLE = False

class ManualControlState:
    def __init__(self):
        self.steer = 0.0
        self.accel = 0.0
        self.brake = 0.0
        self.connected = False

def manual_drive_process(serial_queue, shared_auto_mode, shared_state_array, shared_recording):
    if not INPUTS_AVAILABLE:
        return

    state = ManualControlState()
    
    last_sent_steer = -999
    last_sent_speed = -999
    last_sent_time = 0
    was_braking = False # Track previous state to avoid spamming brake command

    # --- Listener Thread ---
    def monitor_inputs():
        while True:
            try:
                if len(devices.gamepads) == 0:
                    state.connected = False
                    time.sleep(1)
                    continue
                
                state.connected = True
                events = get_gamepad() 
                for event in events:
                    if event.code == 'ABS_X':
                        state.steer = event.state / 32768.0
                    elif event.code == 'ABS_RZ' or event.code == 'ABS_GAS':
                        raw = event.state
                        state.accel = raw / 1023.0 if raw > 255 else raw / 255.0
                    elif event.code == 'BTN_EAST' or event.code == 'BTN_B':
                        state.brake = 1.0 if event.state == 1 else 0.0
                    elif event.code == 'BTN_WEST' or event.code == 'BTN_X':
                        if event.state == 1: 
                            shared_recording.value = not shared_recording.value
                            print(f"MANUAL: Recording set to {shared_recording.value}")
            except:
                state.connected = False
                time.sleep(1)

    t = threading.Thread(target=monitor_inputs, daemon=True)
    t.start()
    print("MANUAL PROCESS: Started.")

    # --- Sender Loop ---
    while True:
        shared_state_array[0] = state.steer
        shared_state_array[1] = state.accel
        shared_state_array[2] = state.brake

        if not shared_auto_mode.value and state.connected:
            
            # 1. Steering (Standard)
            steer_val = 0.0 if abs(state.steer) < config.STEER_DEADZONE else state.steer
            steer_cmd = int(steer_val * config.MAX_STEER_ANGLE)

            # 2. Speed / Brake Logic
            is_braking = state.brake > 0.5
            
            if is_braking:
                speed_cmd = 0 # Visual only
                # Force send brake command if we just started braking or heartbeat
                if not was_braking or (time.time() - last_sent_time > 0.2):
                    try:
                        serial_queue.put(f'#steer:{steer_cmd};;\r\n')
                        serial_queue.put('#brake:0;;\r\n') # <--- SPECIAL COMMAND
                        last_sent_time = time.time()
                    except: pass
            else:
                # Normal Driving
                accel_clean = max(0.0, min(1.0, state.accel))
                speed_cmd = int(accel_clean * config.MAX_SPEED)

                # Delta Check
                steer_changed = abs(steer_cmd - last_sent_steer) > 1 
                speed_changed = abs(speed_cmd - last_sent_speed) > 2
                is_heartbeat = (time.time() - last_sent_time) > 0.2
                
                # Send if changed OR if we were previously braking (to release brake)
                if steer_changed or speed_changed or is_heartbeat or was_braking:
                    try:
                        serial_queue.put(f'#steer:{steer_cmd};;\r\n')
                        serial_queue.put(f'#speed:{speed_cmd};;\r\n')
                        
                        last_sent_steer = steer_cmd
                        last_sent_speed = speed_cmd
                        last_sent_time = time.time()
                    except: pass

            # Update state tracker
            was_braking = is_braking
        
        time.sleep(0.02)