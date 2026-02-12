import cv2
import time
import numpy as np
from ultralytics import YOLO
import config
from car_control import PIDController

# Constants
SPEED_NORMAL = 100
SPEED_HIGHWAY = int(SPEED_NORMAL * 1.3) # 1.3x Speed boost
SPEED_SLOW = 50   
SPEED_STOP = 0

# Area Thresholds
CROSSWALK_THRESHOLD = 1000  
STOP_SIGN_THRESHOLD = 870   
HIGHWAY_SIGN_THRESHOLD = 900 # Threshold for Entry/Exit signs

def yolo_detection_process(frame_queue, result_queue, serial_queue, log_queue,
                           shared_auto_mode, shared_controller_state):
    
    # Initialize PID and Model
    pid_controller = PIDController(config.PID_KP, config.PID_KI, config.PID_KD)
    print("Loading YOLO model...")
    model = YOLO(config.YOLO_MODEL_PATH, task='segment')

    # State Variables
    prev_time = time.time()
    
    # Flags
    stop_mode_active = False
    stop_start_time = 0.0
    ignore_signs_until = 0.0 
    highway_mode_active = False # New persistent state for Highway

    print("AI Engine Started.")

    while True:
        try:
            frame = frame_queue.get(timeout=1)
        except Exception:
            continue

        now = time.time()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 1. INFERENCE
        results = model.predict(
            source=frame_rgb,
            conf=config.CONFIDENCE_THRESHOLD,
            imgsz=320, 
            verbose=False
        )

        annotated_frame = results[0].plot(boxes=False) 
        target_cX = 160 
        
        # --- RESET PER-FRAME FLAGS ---
        # Base speed depends on whether we are in Highway Mode or not
        current_loop_speed = SPEED_HIGHWAY if highway_mode_active else SPEED_NORMAL
        
        crosswalk_active_this_frame = False
        best_lane_detection = None
        highest_lane_conf = 0.0

        # 2. DETECTION SCAN
        for result in results[0]:
            label_index = int(result.boxes.cls[0])
            conf = float(result.boxes.conf[0])
            box = result.boxes.xywh[0]
            area = float(box[2]) * float(box[3])

            # A. Lane Detection
            if label_index == config.LANE_CLASS_ID:
                if conf > highest_lane_conf:
                    highest_lane_conf = conf
                    best_lane_detection = result

            # B. Stop Sign Logic (Class 8)
            elif label_index == 8:
                if area >= STOP_SIGN_THRESHOLD and not stop_mode_active and now > ignore_signs_until:
                    if shared_auto_mode.value:
                        print(f"!!! STOP SIGN: Area {area:.0f} !!!")
                        stop_mode_active = True
                        stop_start_time = now

            # C. Crosswalk Logic (Class 0)
            elif label_index == 0:
                if area >= CROSSWALK_THRESHOLD:
                    crosswalk_active_this_frame = True

            # D. Highway Entry (Class 1)
            elif label_index == 1:
                if area >= HIGHWAY_SIGN_THRESHOLD and not highway_mode_active:
                    print(">>> HIGHWAY ENTRY DETECTED: SPEED BOOST <<<")
                    highway_mode_active = True

            # E. Highway Exit (Class 2)
            elif label_index == 2:
                if area >= HIGHWAY_SIGN_THRESHOLD and highway_mode_active:
                    print("<<< HIGHWAY EXIT DETECTED: RETURNING TO NORMAL <<<")
                    highway_mode_active = False

        # 3. SPEED STATE MACHINE (Hierarchy)
        # Priority 1: STOP (Absolute override)
        if stop_mode_active:
            elapsed = now - stop_start_time
            if elapsed >= 3.0:
                stop_mode_active = False
                ignore_signs_until = now + 5.0 
                # Revert to whatever mode we are in (Highway or Normal)
                current_loop_speed = SPEED_HIGHWAY if highway_mode_active else SPEED_NORMAL
            else:
                current_loop_speed = SPEED_STOP
                cv2.putText(annotated_frame, f"STOPPING: {3.0-elapsed:.1f}s", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Priority 2: CROSSWALK (Safety slow down)
        elif crosswalk_active_this_frame:
            current_loop_speed = SPEED_SLOW
            cv2.putText(annotated_frame, "SLOWING: CROSSWALK", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Priority 3: HIGHWAY / NORMAL (Default behavior)
        elif highway_mode_active:
            cv2.putText(annotated_frame, "MODE: HIGHWAY", (10, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # 4. LANE CENTROID & STEERING
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

        # 5. EXECUTE CONTROL
        if shared_auto_mode.value:
            dt = max(now - prev_time, 0.001)
            prev_time = now
            
            error = cX - target_cX
            steer_val = int(np.clip(pid_controller.update(error, dt), 
                                    -config.MAX_STEER_ANGLE, config.MAX_STEER_ANGLE))
            
            serial_queue.put(f'#steer:{steer_val};;\r\n')
            serial_queue.put(f'#speed:{current_loop_speed};;\r\n')

            cv2.putText(annotated_frame, f"SPD: {current_loop_speed} STR: {steer_val}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 6. OUTPUT
        if not result_queue.full():
            result_queue.put((annotated_frame, cX))