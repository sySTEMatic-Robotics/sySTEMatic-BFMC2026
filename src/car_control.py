import serial
import time
import queue
import config

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def init_serial():
    try:
        ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=0.1, write_timeout=0.1)
        return ser
    except Exception as e:
        print(f"Error opening Serial: {e}")
        return None

def log_msg(queue, message):
    # --- TERMINAL LOGGING DISABLED ---
    # print(message)  <-- This line is now commented out
    try:
        if not queue.full():
            queue.put(str(message))
    except:
        pass

def is_important_message(msg):
    msg = str(msg).lower()
    if "speed:0" in msg: return True
    if "steer" in msg or "speed" in msg: return False
    return True

def read_serial_process(serial_queue, log_queue, ser):
    while True:
        if ser and ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    serial_queue.put(data)
                    if is_important_message(data):
                         log_msg(log_queue, f"RX: {data}")
            except Exception:
                pass 
        time.sleep(0.01)

def write_serial_process(serial_queue, log_queue, ser):
    """
    Writes commands to the microcontroller.
    FIX: Removed aggressive queue draining to prevent command skipping.
    """
    while True:
        try:
            # Just get the next command and write it.
            value = serial_queue.get(timeout=0.1)
            
            if is_important_message(value):
                log_msg(log_queue, f"TX: {value.strip()}")
            
            if ser:
                try:
                    ser.write(str(value).encode('utf-8'))
                    ser.flush() 
                except Exception:
                    pass
                    
        except queue.Empty:
            continue
        except Exception:
            pass