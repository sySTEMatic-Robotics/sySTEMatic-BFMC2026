import multiprocessing
import time
import config
from car_control import init_serial, read_serial_process, write_serial_process
from ai_engine import yolo_detection_process
from web_server import start_server
from manual_control import manual_drive_process 

def main():
    print("Initializing System...")

    # 1. Setup Queues
    # frame_queue: Sends video frames from Web Server -> AI
    frame_queue = multiprocessing.Queue(maxsize=1)
    
    # result_queue: Sends annotated frames from AI -> Web Server
    result_queue = multiprocessing.Queue(maxsize=1)
    
    # serial_queue: Sends commands to the Arduino (from AI or Manual)
    serial_queue = multiprocessing.Queue()
    
    # log_queue: Sends text logs to the Web Interface
    log_queue = multiprocessing.Queue()

    # 2. Setup Shared State (Thread-safe variables shared across processes)
    # 'b' = boolean, 'd' = double/float
    shared_recording = multiprocessing.Value('b', False)       # Controls Video Recording
    shared_auto_mode = multiprocessing.Value('b', True)        # Controls Auto/Manual Mode
    shared_controller_state = multiprocessing.Array('d', [0.0, 0.0, 0.0]) # [Steer, Accel, Brake] for UI overlay

    # 3. Setup Serial Connection
    ser = init_serial()
    
    # 4. Initial Car Calibration
    if ser:
        print("Calibrating Steering...")
        # Send initial calibration commands (adjust as needed for your car)
        ser.write('#kl:30;;\r\n'.encode('utf-8'))
        time.sleep(0.1)
        ser.write("#battery:1;;\r\n".encode('utf-8'))
        time.sleep(0.5)
        ser.write('#steer:0;;\r\n'.encode('utf-8'))
        # Ensure motor is stopped
       # ser.write('#speed:0;;\r\n'.encode('utf-8'))

    # 5. Define Processes
    
    # A. Manual Control Process (High Priority, Low Latency)
    # Needs shared_recording so the 'X' button can trigger recording
    p_manual = multiprocessing.Process(
        target=manual_drive_process,
        args=(serial_queue, shared_auto_mode, shared_controller_state, shared_recording)
    )

    # B. AI Engine Process (High Compute)
    # Needs frame/result queues for video, serial for driving, and shared states for mode/overlay
    p_ai = multiprocessing.Process(
        target=yolo_detection_process, 
        args=(frame_queue, result_queue, serial_queue, log_queue, shared_auto_mode, shared_controller_state)
    )
    
    # C. Serial Handler Processes (IO Bound)
    p_read = multiprocessing.Process(
        target=read_serial_process, 
        args=(serial_queue, log_queue, ser)
    )
    
    p_write = multiprocessing.Process(
        target=write_serial_process, 
        args=(serial_queue, log_queue, ser)
    )

    # 6. Start All Background Processes
    print("Starting Processes...")
    p_manual.start()
    p_ai.start()
    p_read.start()
    p_write.start()

    # 7. Start Web Server (Runs in Main Process, Blocking)
    try:
        print(f"Starting Web Server on {config.WEB_HOST}:{config.WEB_PORT}")
        # Pass all queues + shared variables to the Flask app
        start_server(
            frame_queue, 
            result_queue, 
            log_queue, 
            serial_queue, 
            shared_recording, 
            shared_auto_mode
        )
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # 8. Cleanup on Exit
        print("Terminating processes...")
        p_manual.terminate()
        p_ai.terminate()
        p_read.terminate()
        p_write.terminate()
        
        p_manual.join()
        p_ai.join()
        p_read.join()
        p_write.join()
        
        # Stop car just in case
        if ser:
            ser.write('#speed:0;;\r\n'.encode('utf-8'))
            ser.close()
            
        print("System Halted.")

if __name__ == "__main__":
    main()