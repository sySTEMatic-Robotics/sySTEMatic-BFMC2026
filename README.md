# BFMC — Brain Project

[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.com/)
[![Competition](https://img.shields.io/badge/competition-BFMC%202026-purple.svg)](https://boschfuturemobility.com/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Autonomous driving software for the **Bosch Future Mobility Challenge (BFMC)** — an international student competition where teams program 1:10 scale RC vehicles to navigate a miniature smart city. Built by **Team sySTEMatic**.

---

## Overview

This repository contains the **Brain** — the autonomous driving software that runs on the Raspberry Pi 5 mounted inside the BFMC vehicle. It performs real-time computer vision, sensor-based decision-making, and vehicle control without relying on ROS (Robot Operating System). Instead, it uses a custom **multiprocessing pipeline** architecture for lower overhead and direct hardware access.

### Key Capabilities

- **Lane keeping** via YOLOv11 segmentation mask centroid tracking with PID control
- **Traffic sign recognition** — stop signs, crosswalks, highway entry/exit, parking, and 10 other sign types
- **Traffic light detection** — red, yellow, green
- **Intersection navigation** — timed directional masking for left, right, and straight maneuvers
- **Autonomous parking** — GPIO sensor-based spot detection and pre-programmed parking routines
- **Manual override** — Xbox controller input with auto/manual toggling
- **Web cockpit** — Flask-based streaming UI with live video, telemetry, and remote controls

### Competition Context

The BFMC is organized by the **Bosch Engineering Center Cluj** in partnership with **IEEE-ITSS**. Teams receive a standardized 1:10 scale RC car kit and develop autonomous driving algorithms over ~6 months. Top teams compete live in Cluj-Napoca, Romania in May.

- [Official BFMC Website](https://boschfuturemobility.com/)
- [Official Technical Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/)

---

## System Architecture

```
┌──────────┐    frame_queue    ┌─────────────┐    result_queue    ┌──────────────┐
│  Camera  │ ────────────────> │  Web Server │ ─────────────────> │  Flask MJPEG │
│ (Picam2) │                   │  (main)     │                    │  Stream      │
└──────────┘                   └──────┬──────┘                    └──────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    v                 v                   v
             ┌──────────────┐  ┌───────────────┐  ┌──────────────┐
             │ AI Engine    │  │ Manual Control│  │ Parking Sys  │
             │ (YOLO+PID)   │  │ (Xbox/evdev)  │  │ (GPIO)       │
             └──────┬───────┘  └───────┬───────┘  └──────┬───────┘
                    │                  │                   │
                    v                  v                   v
             ┌────────────────────────────────────────────────┐
             │               serial_queue                     │
             └──────────────────┬─────────────────────────────┘
                                │
                    ┌───────────┴───────────┐
                    v                       v
             ┌──────────────┐        ┌──────────────┐
             │ Serial Writer│        │ Serial Reader │
             │ (commands)   │        │ (telemetry)   │
             └──────┬───────┘        └──────┬───────┘
                    │                       │
                    v                       v
        ┌─────────────────────────────────────────┐
        │           Arduino / STM32 MCU            │
        │     (motor ESC + steering servo)         │
        └─────────────────────────────────────────┘
```

**Communication**: All modules communicate via Python `multiprocessing.Queue` objects with non-blocking writes to prevent backpressure. A custom text protocol (`#speed:N;;\r\n`, `#steer:N;;\r\n`, `#brake:0;;\r\n`) is used over serial (115200 baud) to the microcontroller.

---

## Repository Structure

```
sySTEMatic-BFMC2026/
├── README.md                   # Project overview (this file)
├── docs/                       # Detailed documentation
│   ├── ARCHITECTURE.md         # Full system design & data flow
│   ├── SETUP.md                # Hardware & software installation
│   └── AUTONOMOUS_DRIVING.md   # Feature descriptions & behavior specs
├── src/
│   ├── main.py                 # Entry point — process orchestration
│   ├── config.py               # All tunable constants
│   ├── ai_engine.py            # YOLO inference, lane centering, navigation
│   ├── car_control.py          # Serial communication + PID controller
│   ├── manual_control.py       # Xbox controller input via evdev
│   ├── parking.py              # GPIO-based parking state machine
│   ├── web_server.py           # Flask web UI with MJPEG streaming
│   ├── record.py               # Standalone 1080p camera recorder
│   └── test.py                 # Parking maneuver test script
├── models/
│   ├── bmfc-2026-2_ncnn_model/ # Active YOLOv11 segmentation model (NCNN)
│   ├── BFMC2026_1_ncnn_model/  # Previous model version
│   └── BFMCv13Test2_ncnn_model/# Intermediate test model
├── recordings/                 # Saved video recordings
└── project_status/             # Team progress reports (PDF)
```

---

## Quick Start

### Prerequisites

- Raspberry Pi 5 (8 GB) with Raspberry Pi OS
- BFMC vehicle kit (1:10 scale RC car with Arduino/STM32, Picamera2, GPIO sensors)
- Xbox-compatible controller (optional, for manual override)
- Python 3.11 with the virtual environment at `src/myenv/`

### Installation

```bash
# Clone the repository
git clone <repo-url>
cd sySTEMatic-BFMC2026

# Activate the virtual environment
source src/myenv/bin/activate

# Verify camera
python -c "from picamera2 import Picamera2; print(Picamera2())"
```

### Running

```bash
# Full autonomous driving system
python src/main.py

# Standalone video recorder (1080p)
python src/record.py

# Parking maneuver test (serial only)
python src/test.py
```

The web cockpit is available at `http://<raspberry-pi-ip>:5050`.

### Controls

| Input | Action |
|---|---|
| A button | Toggle Auto / Manual mode |
| X button | Toggle video recording |
| B button | Emergency brake (switches to manual) |
| Left stick X | Steering (manual mode) |
| Right trigger | Throttle (manual mode) |

---

## Model Information

The perception system uses a **YOLOv11-nano segmentation model** trained on BFMC track elements. The model is exported to **NCNN format** for efficient inference on the Raspberry Pi CPU.

**Classes (14 total):**

| ID | Class | Response |
|----|-------|----------|
| 0 | Crosswalk sign | Reduce speed to 50 |
| 1 | Highway entrance | Enable highway mode (2x speed) |
| 2 | Highway exit | Disable highway mode |
| 3 | No-entry sign | — |
| 4 | One-way sign | — |
| 5 | Parking sign | Trigger parking scan |
| 6 | Priority sign | — |
| 7 | Roundabout sign | — |
| 8 | Stop sign | Full stop for 3s, 5s cooldown |
| 9 | Drum (lane marker) | Lane centroid tracking |
| 10 | Drum interrupted | Broken lane line |
| 11–13 | Traffic lights | Yellow / Red / Green |

---

## Configuration

All tunable parameters are in `src/config.py`:

```python
# Camera
CAMERA_RES = (320, 320)     # Inference resolution
CAMERA_FRAMERATE = 30

# PID (lane centering)
PID_KP = 5                   # Proportional gain

# YOLO
CONFIDENCE_THRESHOLD = 0.6
LANE_CLASS_ID = 9
STOP_SIGN_CLASS_ID = 8

# Serial
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Vehicle limits
MAX_STEER_ANGLE = 250
MAX_SPEED = 400
```

---

## License

MIT License. See [LICENSE](LICENSE) for details.

## Contact

**Team sySTEMatic** — BFMC 2025–2026

---

*Built for the Bosch Future Mobility Challenge* — [boschfuturemobility.com](https://boschfuturemobility.com/)
