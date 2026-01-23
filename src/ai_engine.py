import cv2
import time
import numpy as np
from ultralytics import YOLO
import config
from car_control import PIDController

def yolo_detection_process(frame_queue, result_queue, serial_queue, log_queue, shared_auto_mode, shared_controller_state):
    """
    Main AI Process.
    shared_controller_state: Array('d', 3) -> [steer, accel, brake] passed from main
    """
    pid_controller = PIDController(config.PID_KP, config.PID_KI, config.PID_KD)
    model = YOLO(config.YOLO_MODEL_PATH, task='segment')
    
    prev_time = time.time()
    target_cX = 160
    
    # Wait for shared state initialization
    while shared_auto_mode is None:
        time.sleep(0.1)

    print("AI Engine Started.")

    while True:
        try:
            frame = frame_queue.get(timeout=1)
        except:
            continue

        # 1. AI VISUALS (Always Run)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model.predict(source=frame, show=False, save=False, 
                                conf=config.CONFIDENCE_THRESHOLD, 
                                imgsz=config.CAMERA_RES[0], verbose=False)
        annotated_frame = results[0].plot()

        # Lane Detection logic
        best_lane_detection = None
        highest_confidence = 0.0
        for result in results:
            for c in result:
                if int(c.boxes.cls[0]) == config.LANE_CLASS_ID:
                    if float(c.boxes.conf[0]) > highest_confidence:
                        highest_confidence = float(c.boxes.conf[0])
                        best_lane_detection = c

        cX = 160 
        if best_lane_detection is not None:
            b_mask = np.zeros(annotated_frame.shape[:2], np.uint8)
            contour = best_lane_detection.masks.xy.pop().astype(np.int32).reshape(-1, 1, 2)
            cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
            M = cv2.moments(b_mask)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(annotated_frame, (cX, cY), 5, (0, 255, 0), -1)

        # 2. CONTROL LOGIC
        if shared_auto_mode.value:
            # === AUTO MODE ===
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time
            
            error = cX - target_cX
            pid_output = int(pid_controller.update(error, dt))
            pid_output = np.clip(pid_output, -config.MAX_STEER_ANGLE, config.MAX_STEER_ANGLE)
            
            serial_queue.put(f'#steer:{pid_output};;\r\n')
            cv2.putText(annotated_frame, f"AUTO PID: {pid_output}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
        else:
            # === MANUAL MODE VISUALIZATION ===
            # We don't drive here! manual_control.py does the driving.
            # We just read the shared array to show the user what's happening.
            pid_controller.integral = 0 
            
            # Unpack shared array
            man_steer = shared_controller_state[0]
            man_accel = shared_controller_state[1]
            man_brake = shared_controller_state[2]
            
            state_text = "BRAKE" if man_brake > 0.5 else f"GAS: {man_accel:.2f}"
            cv2.putText(annotated_frame, f"MANUAL: {state_text}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            
            

        # 3. SEND TO WEB
        if not result_queue.full():
            result_queue.put((annotated_frame, cX))