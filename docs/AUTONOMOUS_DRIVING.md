# Autonomous Driving Features

## Overview

The BFMC Brain implements the following autonomous driving capabilities required by the Bosch Future Mobility Challenge. Each feature is described with its detection mechanism, behavioral response, and configurable parameters.

---

## Lane Keeping

**Detection**: YOLOv11 segmentation mask of class 9 (drum/lane marker). The mask's **centroid** is computed using OpenCV moments, and the horizontal offset from the frame center (X=160) becomes the PID error signal.

**Control**: A **P-only PID controller** (`Kp=5`) converts the centroid error to a steering angle. The output is clamped to `±MAX_STEER_ANGLE` (250).

```
error = centroid_x - 160
steer = clamp(Kp × error, -250, 250)
```

**Configuration:**
| Parameter | Default | Location |
|-----------|---------|----------|
| `PID_KP` | 5 | `config.py` |
| `PID_KI` | 0 | `config.py` |
| `PID_KD` | 0 | `config.py` |
| `LANE_CLASS_ID` | 9 | `config.py` |
| `CONFIDENCE_THRESHOLD` | 0.6 | `config.py` |
| `MAX_STEER_ANGLE` | 250 | `config.py` |

**Edge Cases:**
- **No lane detected**: Car drives straight (centroid defaults to 160, error=0)
- **Multiple lane detections**: The highest-confidence detection is used
- **Broken lane lines** (class 10, drum interrupted): Class handled by model but control falls back to highest-confidence lane mask

---

## Traffic Sign Recognition

### Stop Sign (Class 8)

**Detection**: YOLO bounding box area ≥ 870 px² (threshold tuned for ~1.5m distance at 320×320 resolution).

**Behavior:**
1. Full stop (speed=0) for **3.0 seconds**
2. Resume normal speed (100 or 200 if highway)
3. **5-second cooldown** before responding to another stop sign (prevents double-triggering)

**Configuration:**
| Parameter | Default | Location |
|-----------|---------|----------|
| `STOP_SIGN_CLASS_ID` | 8 | `config.py` |
| `STOP_SIGN_THRESHOLD` | 870 | `ai_engine.py` |
| Stop duration | 3.0s | `ai_engine.py:142` |
| Cooldown | 5.0s | `ai_engine.py:144` |

### Crosswalk (Class 0)

**Detection**: YOLO bounding box area ≥ 1000 px².

**Behavior**: Reduce speed to `SPEED_SLOW` (50) while the sign is visible. Returns to normal speed once out of view.

**Configuration:**
| Parameter | Default | Location |
|-----------|---------|----------|
| `CROSSWALK_THRESHOLD` | 1000 | `ai_engine.py` |
| `SPEED_SLOW` | 50 | `ai_engine.py` |

> **Note**: The BFMC track also has pedestrian dolls at crosswalks. Pedestrian detection was not yet implemented in the YOLO model at the time of this writing. The crosswalk sign response is the primary mechanism for crosswalk compliance.

### Highway Entry (Class 1) / Exit (Class 2)

**Detection**: YOLO bounding box area ≥ 900 px².

**Behavior**:
- **Entry**: Activates highway mode, doubling speed from 100 to 200
- **Exit**: Deactivates highway mode, returning to normal speed (100)

**Edge case**: The system uses guard flags so that highway mode toggles only on transition (entry → highway ON, exit → highway OFF), not every frame the sign is visible.

### Parking Sign (Class 5)

**Detection**: YOLO bounding box area ≥ 1000 px².

**Behavior**: Triggers the parking system's `trigger_scan()` method. See [Parking](#parking) below for full details.

### Other Signs

The YOLO model also detects these signs, but active responses are not yet implemented:
- No-entry (class 3)
- One-way (class 4)
- Priority (class 6)
- Roundabout (class 7)

---

## Traffic Light Detection

**Detection**: YOLO classes 11 (yellow), 12 (red), 13 (green).

**Status**: The YOLO model is trained on these classes, but **active traffic light response logic is not yet implemented** in the control pipeline. The BFMC track has 5 traffic lights (1 at start, 4 at an intersection) at 24 cm height. Smart lights also broadcast state via Wi-Fi UDP (V2X).

---

## Intersection Navigation

**Detection**: When the lane segmentation mask width exceeds **85% of frame width** at **70% frame height**, an intersection is detected. This corresponds to the lane lines widening into an intersection.

**Mechanism**: The system uses **directional mask erasure** to force the PID controller to steer in the desired direction:

### Right Turn
- **Duration**: 2.5 seconds
- **Masking**: Left 50% of frame erased, bottom 40% erased
- **Effect**: Only the right-side lane is visible, centroid shifts right, car steers right

### Left Turn (Two-Phase)
- **Total duration**: 8 seconds
- **Phase 1 (first 5s, "pull forward")**: Sides masked (30% left, 30% right, bottom 40%). Car drives straight into the intersection
- **Phase 2 (last 3s, "hard left")**: Right 58% of frame masked, bottom 40% masked. Target centroid shifted to X=220 to force leftward steering

### Straight
- **Duration**: 2 seconds
- **Masking**: Sides masked (30% left, 30% right, bottom 40%). Car drives straight through

**Configuration:**

| Parameter | Default | Location |
|-----------|---------|----------|
| Intersection width trigger | 85% of frame | `ai_engine.py:173` |
| Trigger row | 70% of frame height | `ai_engine.py:169` |
| RIGHT duration | 2.5s | `ai_engine.py:179` |
| LEFT duration | 8.0s | `ai_engine.py:181` |
| STRAIGHT duration | 2.0s | `ai_engine.py:183` |
| Turn speed | `SPEED_SLOW` (50) | `ai_engine.py:195` |

**Navigation Direction**: The `current_nav_command` variable is currently hardcoded to `"RIGHT"` (line 49 of `ai_engine.py`). In a full implementation, this would be set by a path planner or GPS waypoint system.

**Edge Case**: Intersection detection is disabled while already in a turn (`intersection_turn_active` flag prevents re-triggering).

---

## Autonomous Parking

**Detection**: Triggered by the parking sign (class 5). See [Parking Sign](#parking-sign-class-5).

**Hardware**: 4 × GPIO digital sensors (left, right, front, back) detect the presence or absence of a vehicle in adjacent parking spots.

**State Machine:**

```
IDLE
 │
 │ Parking sign detected + cooldown expired
 ▼
SCANNING (3.0s, speed=50)
 │
 ├── Left sensor free → LEFT ROUTINE
 ├── Right sensor free → RIGHT ROUTINE
 └── Both occupied → MOVE_NEXT
                          │
                          │ Drive forward 1.0s
                          ▼
                     Check sensors again
                          │
                     ├── Spot found → Execute routine
                     └── Still occupied → ABORT (10s cooldown)
```

**Parking Routines:**

| Step | Right Spot | Left Spot |
|------|-----------|-----------|
| 1 | Forward 50, 0° steer, 1.5s | Forward 50, 0° steer, 1.5s |
| 2 | Reverse −50, +230° steer, 3.5s | Reverse −50, +210° steer, 3.5s |
| 3 | Reverse −50, −230° steer, 3.0s | Reverse −50, −210° steer, 3.0s |
| 4 | Stop, 1.0s | Stop, 1.0s |

**ESC Reverse-Arming**: The Quickrun Fusion SE motor requires a specific arming sequence for forward-to-reverse transitions. The `SmartSerial` helper handles this automatically:
1. Send neutral (speed=0), wait 300ms
2. Send brake pulse (speed=20), wait 200ms
3. Send neutral again, wait 300ms
4. Send reverse command

**Configuration:**
| Parameter | Default | Location |
|-----------|---------|----------|
| `SCAN_DURATION` | 3.0s | `parking.py:98` |
| `RETRY_MOVE_DURATION` | 1.0s | `parking.py:99` |
| `PARKING_SPEED` | 50 | `parking.py:100` |
| Cooldown after parking | 10.0s | `parking.py:219` |

---

## Manual Override

At any time, the Xbox controller can take control:

- **A button**: Toggles between Auto and Manual mode
- **B button**: Emergency brake — immediately sets speed to 0 and switches to Manual mode
- **Auto-disconnect**: If the controller is unplugged during manual mode, the car stops (no heartbeat → speed resets to 0)

Manual commands are sent to the same `serial_queue` as AI commands. The serial writer process doesn't care about the source — it just writes whatever's in the queue.

---

## Web Cockpit

The Flask-based web interface (`http://<ip>:5050`) provides:

- **Live video feed** (MJPEG stream with YOLO annotations)
- **Real-time telemetry** (speed, steer angle, lane centroid, current state)
- **SSE log stream** (serial messages, state transitions, detections)
- **Control buttons**: AUTO/MANUAL toggle, REC toggle, EMERGENCY SHUTDOWN
- **Responsive design**: Works on desktop and mobile browsers

---

## Model Details

| Property | Value |
|----------|-------|
| Architecture | YOLOv11-nano (segmentation) |
| Input size | 320 × 320 |
| Format | NCNN (Tencent neural network inference) |
| Classes | 14 (see class table in README) |
| Inference | CPU-only on Raspberry Pi 5 |
| Approx FPS | ~10 FPS at 320×320 |
| Confidence threshold | 0.6 |
