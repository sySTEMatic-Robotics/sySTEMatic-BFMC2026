# Setup Guide

## Hardware Requirements

### BFMC Vehicle Kit (Provided by Bosch)

| Component | Specification |
|-----------|---------------|
| Chassis | Reely TC-04, 1:10 scale on-road |
| Main Computer | Raspberry Pi 5 (8 GB RAM) |
| Microcontroller | Nucleo F401RE (STM32) or Arduino |
| Camera | Raspberry Pi Camera Module 3 (wide-angle) |
| Motor | Quickrun Fusion SE 1200KV (integrated ESC) |
| Steering Servo | RS-610WP |
| Battery | 2-cell LiPo, 4500–6200 mAh |
| Power Board | Custom Bosch power distribution board |

### Additional Components

| Component | Purpose |
|-----------|---------|
| Xbox-compatible controller | Manual override / testing |
| 4× GPIO sensors | Parking spot detection (left, right, front, back) |
| MicroSD card (64 GB+) | Raspberry Pi OS + software |

---

## Operating System Setup

### 1. Flash Raspberry Pi OS

```bash
# Using Raspberry Pi Imager (recommended)
# Select: Raspberry Pi OS (64-bit, Bookworm or later)
# Enable SSH, set hostname, configure Wi-Fi in Imager settings
```

### 2. Enable Hardware Interfaces

On the Raspberry Pi:

```bash
sudo raspi-config
```

Enable:
- **Camera** (Interface Options → Camera)
- **Serial Port** (Interface Options → Serial Port → disable login shell, enable hardware)
- **GPIO** (enabled by default)

### 3. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
    python3.11 \
    python3.11-venv \
    python3-pip \
    libcamera-dev \
    libatlas-base-dev \
    libopenblas-dev \
    cmake \
    git
```

---

## Software Installation

### 1. Clone Repository

```bash
git clone <repo-url>
cd sySTEMatic-BFMC2026
```

### 2. Virtual Environment

The project includes a pre-configured virtual environment at `src/myenv/`. To use it:

```bash
source src/myenv/bin/activate
```

To recreate from scratch:

```bash
python3.11 -m venv src/myenv
source src/myenv/bin/activate
pip install --upgrade pip
```

### 3. Python Dependencies

Install the required packages:

```bash
pip install ultralytics
pip install ncnn
pip install opencv-python
pip install picamera2
pip install flask
pip install pyserial
pip install evdev
pip install gpiozero
pip install numpy
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

> **Note**: `picamera2` is only available on Raspberry Pi OS. `evdev` and `gpiozero` are Linux-only. Development on a PC requires stubbing these.

### 4. Verify Camera

```bash
python -c "
from picamera2 import Picamera2
cam = Picamera2()
print(cam.sensor_modes)
"
```

---

## Hardware Wiring

### Camera

Connect the Raspberry Pi Camera Module 3 to the **MIPI CSI** connector on the Raspberry Pi 5 using the provided flex cable. Ensure the blue tab faces the Ethernet port.

### Serial Connection (Raspberry Pi → Microcontroller)

| Raspberry Pi 5 GPIO | Microcontroller |
|---------------------|-----------------|
| GPIO 14 (TXD) | RX pin |
| GPIO 15 (RXD) | TX pin |
| GND | GND |

The serial device will appear at `/dev/ttyAMA0` or `/dev/ttyACM0` (USB-based Arduino). Update `SERIAL_PORT` in `src/config.py` accordingly.

### Parking Sensors (GPIO)

| Sensor | GPIO Pin | Physical Pin |
|--------|----------|-------------|
| Right | GPIO 4 | Pin 7 |
| Left | GPIO 27 | Pin 13 |
| Front | GPIO 22 | Pin 15 |
| Back | GPIO 17 | Pin 11 |

Sensors should be wired as active-low digital inputs with pull-up resistors (the `gpiozero` library's `pull_up=False` setting uses the internal pull-downs).

### Xbox Controller

Connect via USB or Bluetooth. Verify with:

```bash
evtest
# Select the controller device (typically /dev/input/event0 or event1)
```

---

## Model Setup

### Active Model

Place the NCNN model files in `models/`:

```
models/
└── bmfc-2026-2_ncnn_model/
    ├── model.ncnn.bin       # Weights
    ├── model.ncnn.param     # Network structure
    ├── model_ncnn.py        # NCNN wrapper class
    └── metadata.yaml        # Model metadata (classes, input size)
```

### Exporting from PyTorch/YOLO

If training your own model:

```python
from ultralytics import YOLO

model = YOLO('best.pt')
model.export(format='ncnn', imgsz=320)
# Output saved to best_ncnn_model/
```

Copy the exported `*_ncnn_model/` directory to `models/` and update `YOLO_MODEL_PATH` in `src/config.py`.

---

## Calibration

### Steering Endpoints

The steering servo has configurable limits. Adjust `MAX_STEER_ANGLE` in `config.py` based on your servo's range. The default of 250 corresponds to the RS-610WP servo's range.

### PID Tuning

The PID controller for lane centering is P-only by default. If the car oscillates:

1. Reduce `PID_KP` (e.g., from 5 to 3)
2. If steady-state error is large, add a small `PID_KI` (e.g., 0.01)
3. If the car overcorrects, add `PID_KD` (e.g., 0.1)

All tuning is done in `config.py` — no rebuild required.

### Speed Calibration

Adjust `MAX_SPEED` (default 400) to match your ESC's PWM range. The base speeds are defined in `ai_engine.py`:

- `SPEED_NORMAL = 100` (25% of max)
- `SPEED_HIGHWAY = 200` (50% of max)
- `SPEED_SLOW = 50` (12.5% of max)

### Parking Sensor Thresholds

GPIO sensors should read `1` when a parking spot is detected (free) and `0` when occupied. If your sensors are inverted, change `pull_up=False` to `pull_up=True` in `parking.py:73-76`.

---

## Running

### Full System

```bash
cd src
python main.py
```

Expected output:
```
Loading YOLO model...
AI Engine Started. (Anti-Hang Mode Enabled)
 * Serving Flask app 'web_server'
 * Running on http://0.0.0.0:5050
```

Open `http://<raspberry-pi-ip>:5050` in a browser to see the cockpit.

### Standalone Recorder

```bash
python src/record.py
# Records 1080p/30fps to recordings/ with UUID filenames
```

### Parking Test

```bash
python src/test.py
# Sends a hardcoded parking routine via serial
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `No module named 'picamera2'` | Not running on Raspberry Pi or OS mismatch | Run on RPi with Bookworm+ |
| `SerialException: could not open port` | Wrong serial device | Check `ls /dev/tty*`, update `SERIAL_PORT` |
| Camera returns black frames | Lens cap or camera not detected | `libcamera-hello --list-cameras` |
| Controller not responding | Wrong event device | Run `python -m evdev.evtest` to find device |
| Parking sensors always read 0 | Wiring or pull-up config | Test with `gpiozero` `DigitalInputDevice` example script |
| YOLO model fails to load | Missing NCNN files or wrong path | Verify `models/bmfc-2026-2_ncnn_model/` contains `.bin` and `.param` files |
| Flask not reachable | Firewall or wrong interface | Check `WEB_HOST = '0.0.0.0'` in config |
