import cv2
from picamera2 import Picamera2
import time
import signal
import sys
import random
import uuid

# This allows us to handle the cleanup cleanly when you press Ctrl+C
def signal_handler(sig, frame):
    print("\nStopping recording... (Interrupted)")
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, signal_handler)

def record_camera():
    print("Initializing Camera (Headless Mode)...")
    picam2 = Picamera2()
    
    # Configure for BGR because OpenCV VideoWriter expects BGR
    config = picam2.create_video_configuration(
        main={"size": (1920, 1080), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()

    # Setup Video Writer
    output_filename = 'headless_video' + str(uuid.uuid4()) + ".mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 30.0
    resolution = (1920, 1080)
    
    out = cv2.VideoWriter(output_filename, fourcc, fps, resolution)

    print(f"Recording to: {output_filename}")
    print("Press 'Ctrl + C' on your keyboard to stop.")

    try:
        # We record for a max duration or until Ctrl+C
        # Using a loop counter or infinite loop
        frame_count = 0
        start_time = time.time()
        
        while True:
            # Capture frame
            frame = picam2.capture_array()

            # Write to file
            out.write(frame)
            
            # Print status every 30 frames (approx 1 sec) so you know it's working
            frame_count += 1
            if frame_count % 30 == 0:
                duration = time.time() - start_time
                # \r overwrites the line so your terminal doesn't get flooded
                print(f"Recorded {int(duration)} seconds...", end='\r')

    except KeyboardInterrupt:
        # This block runs when you press Ctrl+C
        print("\nStop signal received.")

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        # Cleanup
        print("Saving video file...")
        picam2.stop()
        out.release()
        print("Done. Video saved.")

if __name__ == "__main__":
    record_camera()