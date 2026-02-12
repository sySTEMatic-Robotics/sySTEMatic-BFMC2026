# config.py

# Serial Settings
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Camera Settings
CAMERA_RES = (320, 320)
CAMERA_FRAMERATE = 30

# PID Settings
PID_KP = 5 # 3.5
PID_KI = 0.00
PID_KD = 0.0

# AI Settings
YOLO_MODEL_PATH = "../models/bmfc-2026-2_ncnn_model"
CONFIDENCE_THRESHOLD = 0.3
LANE_CLASS_ID = 9
STOP_SIGN_CLASS_ID = 8

# Web Server Settings
WEB_HOST = '0.0.0.0'
WEB_PORT = 5050

RECORDING_PATH = "../recordings/"

MAX_STEER_ANGLE = 250  # Maximum steering angle (+/-)
MAX_SPEED = 400     # Maximum speed value

# Xbox Controller Mapping (Standard Layout)
# Axis and Button IDs can vary slightly by OS, but these are standard for Linux/inputs
STEER_DEADZONE = 0.01   # Ignore small joystick movements