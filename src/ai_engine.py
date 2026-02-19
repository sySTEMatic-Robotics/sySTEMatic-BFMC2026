import cv2
import time
import numpy as np
import queue  # Import standard queue for Full exception
from ultralytics import YOLO
import config
from car_control import PIDController
from parking import ParkingSystem 

# Constants
SPEED_NORMAL = 100
SPEED_HIGHWAY = int(SPEED_NORMAL * 2) 
SPEED_SLOW = 50   
SPEED_STOP = 0

# Area Thresholds
CROSSWALK_THRESHOLD = 1000  
STOP_SIGN_THRESHOLD = 870   
HIGHWAY_SIGN_THRESHOLD = 900 
PARKING_SIGN_THRESHOLD = 1000 

# --- [FIX] NON-BLOCKING QUEUE HELPER ---
def try_put(q, data, name="Queue"):
    """
    Tries to put data into a queue. If full, drops the data 
    and keeps running instead of hanging the AI.
    """
    try:
        q.put(data, block=False) # <--- The Key Fix: Non-blocking
    except queue.Full:
        # Optional: Print warning if you want to know it's dropping
        pass 

def yolo_detection_process(frame_queue, result_queue, serial_queue, log_queue,
                           shared_auto_mode, shared_controller_state):
    
    pid_controller = PIDController(config.PID_KP, config.PID_KI, config.PID_KD)
    print("Loading YOLO model...")
    model = YOLO(config.YOLO_MODEL_PATH, task='segment')
    parking_sys = ParkingSystem()

    # State Variables
    prev_time = time.time()
    stop_mode_active = False
    stop_start_time = 0.0
    ignore_signs_until = 0.0 
    highway_mode_active = False

    current_nav_command = "RIGHT"
    intersection_turn_active = False
    intersection_turn_end_time = 0.0

    print("AI Engine Started. (Anti-Hang Mode Enabled)")

    while True:
        try:
            # [FIX] CLEAR OLD FRAMES: Only process the NEWEST frame
            # If the queue has 5 frames, skip the first 4.
            frame = None
            while not frame_queue.empty():
                try:
                    frame = frame_queue.get_nowait()
                except queue.Empty:
                    break
            
            if frame is None:
                # If we cleared everything or it was empty, wait for next one
                frame = frame_queue.get(timeout=1)
                
        except Exception:
            continue

        now = time.time()

        # 1. PARKING SYSTEM UPDATE
        is_parking, park_speed, park_steer = parking_sys.update(serial_queue)

        if is_parking:
            if shared_auto_mode.value:
                if park_speed is not None: 
                    try_put(serial_queue, f'#speed:{park_speed};;\r\n')
                if park_steer is not None: 
                    try_put(serial_queue, f'#steer:{park_steer};;\r\n')
                
                cv2.putText(frame, f"PARKING MODE: {parking_sys.current_state}", (10, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)
                
                try_put(result_queue, (frame, 160))
            continue 

        # 2. INFERENCE
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model.predict(source=frame_rgb, conf=config.CONFIDENCE_THRESHOLD, imgsz=320, verbose=False)
        annotated_frame = results[0].plot() 
        target_cX = 160 
        
        # Reset Flags
        current_loop_speed = SPEED_HIGHWAY if highway_mode_active else SPEED_NORMAL
        crosswalk_active_this_frame = False
        best_lane_detection = None
        highest_lane_conf = 0.0

        # 3. DETECTION SCAN
        for result in results[0]:
            label_index = int(result.boxes.cls[0])
            conf = float(result.boxes.conf[0])
            box = result.boxes.xywh[0]
            area = float(box[2]) * float(box[3])

            if label_index == config.LANE_CLASS_ID:
                if conf > highest_lane_conf:
                    highest_lane_conf = conf
                    best_lane_detection = result

            elif label_index == 8: # Stop Sign
                if area >= STOP_SIGN_THRESHOLD and not stop_mode_active and now > ignore_signs_until:
                    if shared_auto_mode.value:
                        stop_mode_active = True
                        stop_start_time = now

            elif label_index == 0: # Crosswalk
                if area >= CROSSWALK_THRESHOLD:
                    crosswalk_active_this_frame = True

            elif label_index == 1: # Highway Enter
                if area >= HIGHWAY_SIGN_THRESHOLD and not highway_mode_active:
                    highway_mode_active = True

            elif label_index == 2: # Highway Exit
                if area >= HIGHWAY_SIGN_THRESHOLD and highway_mode_active:
                    highway_mode_active = False

            elif label_index == 5: # Parking
                if area >= PARKING_SIGN_THRESHOLD:
                    triggered = parking_sys.trigger_scan()
                    if triggered:
                        current_loop_speed = 50 

        # 4. SPEED LOGIC
        if stop_mode_active:
            elapsed = now - stop_start_time
            if elapsed >= 3.0:
                stop_mode_active = False
                ignore_signs_until = now + 5.0 
                current_loop_speed = SPEED_HIGHWAY if highway_mode_active else SPEED_NORMAL
            else:
                current_loop_speed = SPEED_STOP
                cv2.putText(annotated_frame, f"STOPPING: {3.0-elapsed:.1f}s", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        elif crosswalk_active_this_frame:
            current_loop_speed = SPEED_SLOW
            cv2.putText(annotated_frame, "SLOWING: CROSSWALK", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        elif highway_mode_active:
            cv2.putText(annotated_frame, "MODE: HIGHWAY", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

      # 5. LANE CENTROID & INTERSECTION NAVIGATION
        cX = target_cX
        if best_lane_detection is not None and best_lane_detection.masks is not None:
            try:
                # 1. Grab mask and MUST USE .copy() so we don't corrupt the original tensor
                mask_array = best_lane_detection.masks.data[0].cpu().numpy().copy()
                h, w = mask_array.shape
                
                # 2. Intersection Trigger check
                row_idx = int(h * 0.7)
                road_width = np.count_nonzero(mask_array[row_idx, :])
                
                # Trigger ONLY if not already turning
                if not intersection_turn_active and road_width > (w * 0.85):
                    print(f"!!! ENTERING INTERSECTION: TURNING {current_nav_command} !!!")
                    intersection_turn_active = True
                    
                    # --- DYNAMIC TURN TIMERS ---
                    if current_nav_command == "RIGHT":
                        turn_duration = 2.5  # Short, sharp turn
                    elif current_nav_command == "LEFT":
                        turn_duration = 8  # Wide turn, needs more time
                    else: # STRAIGHT
                        turn_duration = 2.0  # Just enough to cross the wide part
                        
                    intersection_turn_end_time = now + turn_duration 

                # Check if the turn timer is up
                if intersection_turn_active and now > intersection_turn_end_time:
                    intersection_turn_active = False

                # 3. APPLY THE DIRECTIONAL ERASER (If actively turning)
                if intersection_turn_active and current_nav_command:
                    
                    # Force slow speed during the maneuver for safety
                    current_loop_speed = SPEED_SLOW 
                    
                    if current_nav_command == "RIGHT":
                        mask_array[:, :int(w * 0.5)] = 0  # Black out Left half
                        mask_array[int(h * 0.6):, :] = 0  # Black out Bottom
                        cv2.putText(annotated_frame, "TURNING: RIGHT (LOCKED)", (10, 150), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        
                    elif current_nav_command == "LEFT":
                        # --- PHASED LEFT TURN: Pull forward first, then turn ---
                        time_remaining = intersection_turn_end_time - now
                        
                        # Total time is 8.0s. If time remaining is > 5.5s, we are in the first 2.5 seconds.
                        if time_remaining > 3:
                            print("Left turn. Going straight")
                            # PHASE 1: Go STRAIGHT to pull into the intersection
                            mask_array[:, :int(w * 0.3)] = 0  
                            mask_array[:, int(w * 0.7):] = 0  
                            mask_array[int(h * 0.6):, :] = 0
                            cv2.putText(annotated_frame, "TURNING: LEFT (PULL FWD)", (10, 150), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        else:
                            print("Left turn. Going left")
                            # PHASE 2: Execute the LEFT turn
                            mask_array[:, int(w * 0.42):] = 0  # Black out right 60% of screen
                            mask_array[int(h * 0.6):, :] = 0  # Black out Bottom
                            
                            target_cX = 220  # Shift target to force harder left steering
                            
                            cv2.putText(annotated_frame, "TURNING: LEFT (LOCKED)", (10, 150), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                        
                    elif current_nav_command == "STRAIGHT":
                        mask_array[:, :int(w * 0.3)] = 0  
                        mask_array[:, int(w * 0.7):] = 0  
                        mask_array[int(h * 0.6):, :] = 0
                        cv2.putText(annotated_frame, "GOING: STRAIGHT (LOCKED)", (10, 150), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                # 4. Calculate Centroid
                mask_uint8 = (mask_array * 255).astype(np.uint8)
                M = cv2.moments(mask_uint8)
                
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # Draw a distinct pink dot so you can see the target moving into the turn
                    cv2.circle(annotated_frame, (cX, cY), 8, (255, 0, 255), -1)
                    
            except Exception as e:
                pass

        # 6. EXECUTE CONTROL
        if shared_auto_mode.value:
            dt = max(now - prev_time, 0.001)
            prev_time = now
            
            error = cX - target_cX
            steer_val = int(np.clip(pid_controller.update(error, dt), 
                                    -config.MAX_STEER_ANGLE, config.MAX_STEER_ANGLE))
            
            # [FIX] Use try_put so we don't hang if Serial is slow
            try_put(serial_queue, f'#steer:{steer_val};;\r\n')
            try_put(serial_queue, f'#speed:{current_loop_speed};;\r\n')

            cv2.putText(annotated_frame, f"SPD: {current_loop_speed} STR: {steer_val}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 7. OUTPUT
        # [FIX] Use try_put so we don't hang if Web Server (Port 5050) crashes
        try_put(result_queue, (annotated_frame, cX))