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

        # 5. LANE CENTROID
        cX = target_cX
        if best_lane_detection is not None and best_lane_detection.masks is not None:
            try:
                mask_pts = best_lane_detection.masks.xy[0]
                contour = mask_pts.astype(np.int32).reshape(-1, 1, 2)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(annotated_frame, (cX, cY), 5, (0, 255, 0), -1)
            except Exception:
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