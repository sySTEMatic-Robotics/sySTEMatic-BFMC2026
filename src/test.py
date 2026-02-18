import serial
import time

# --- CONFIGURATION ---
# Try '/dev/ttyAMA0', '/dev/ttyS0', or '/dev/ttyUSB0' depending on your Pi setup
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

def send_command(ser, speed, steer):
    """
    Formats the command exactly like your main project:
    #speed:50;;\r\n
    #steer:0;;\r\n
    """
    # 1. format the strings
    cmd_speed = f"#speed:{int(speed)};;\r\n"
    cmd_steer = f"#steer:{int(steer)};;\r\n"
    
    # 2. Encode to bytes
    ser.write(cmd_speed.encode('utf-8'))
    ser.write(cmd_steer.encode('utf-8'))
    
    # 3. Flush to force send immediately (Critical for timing)
    ser.flush()
    
    print(f"SENT -> Speed: {speed}, Steer: {steer}")

def main():
    print(f"Opening Serial Port {SERIAL_PORT} at {BAUD_RATE}...")
    
    try:
        # Initialize Serial
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2) # Wait for Arduino to reset/stabilize
        print("Serial Ready. Starting Sequence...")
        ser.write('#kl:30;;\r\n'.encode('utf-8'))
        ser.write("#battery:1;;\r\n".encode('utf-8'))
        ser.write('#steer:0;;\r\n'.encode('utf-8'))

        ser.write('#speed:80;;\r\n'.encode('utf-8'))
        time.sleep(0.2)
        ser.write('#steer:0;;\r\n'.encode('utf-8'))
        time.sleep(1.5)
        ser.write('#speed:-80;;\r\n'.encode('utf-8'))
        time.sleep(0.2)
        ser.write('#steer:230;;\r\n'.encode('utf-8'))
        time.sleep(3.5)
        ser.write('#speed:-80;;\r\n'.encode('utf-8'))
        time.sleep(0.2)
        ser.write('#steer:-230;;\r\n'.encode('utf-8'))
        time.sleep(3)
        ser.write('#speed:0;;\r\n'.encode('utf-8'))

        ser.close()
        print("\nTest Complete.")

    except serial.SerialException as e:
        print(f"\nERROR: Could not open serial port.\n{e}")
        print("Tip: Check if /dev/ttyAMA0 is correct or if you need sudo.")

if __name__ == "__main__":
    main()