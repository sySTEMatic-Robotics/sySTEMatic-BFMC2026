# Architecture

## Design Philosophy

The BFMC Brain is a **multiprocessing-based autonomous driving pipeline** built on Python's `multiprocessing` primitives. It deliberately avoids ROS to minimize overhead, reduce dependency complexity, and maintain direct control over hardware interfaces on the Raspberry Pi 5.

## Process Architecture

The system runs as **4 child processes** spawned by the main process, which itself hosts the Flask web server:

```
main.py (Process 0 — Web Server + Orchestrator)
├── ai_engine (Process 1 — YOLO Inference + Control)
├── manual_control (Process 2 — Xbox Controller Input)
├── serial_reader (Process 3 — Arduino Telemetry)
└── serial_writer (Process 4 — Arduino Commands)
```

### Why Multiprocessing Instead of Threads

Python's **Global Interpreter Lock (GIL)** prevents true parallel execution of CPU-bound Python code. Since YOLO inference is CPU-intensive, running it in a thread would block the web server and manual controller. Processes provide true parallelism and fault isolation — if the AI engine crashes, the web server stays running.

## Inter-Process Communication (IPC)

### Queues

All data exchange uses non-blocking `multiprocessing.Queue` objects with a **drop-oldest** strategy to prevent backpressure:

| Queue | Producer | Consumer | Content |
|-------|----------|----------|---------|
| `frame_queue` | Web Server | AI Engine | Raw camera frames (320×320 BGR) |
| `result_queue` | AI Engine | Web Server | Annotated frames + lane centroid |
| `serial_queue` | AI Engine, Manual Control, Parking | Serial Writer | Speed/steer commands |
| `log_queue` | All processes | Web Server (SSE) | Log message strings |

### Shared Memory

`multiprocessing.Value` and `multiprocessing.Array` objects are used for atomic state flags shared across processes:

| Variable | Type | Purpose |
|----------|------|---------|
| `shared_auto_mode` | `Value('b', 1)` | Auto (1) vs Manual (0) mode toggle |
| `shared_controller_state` | `Array('i', [0, 0])` | [steer, speed] from Xbox controller |
| `shared_record_toggle` | `Value('b', 0)` | Recording on/off flag |

### Non-Blocking Queue Helper

```python
def try_put(q, data, name="Queue"):
    try:
        q.put(data, block=False)
    except queue.Full:
        pass  # Drop frame/command, keep running
```

This is critical: if the web server stops consuming frames (e.g., browser disconnects), the AI engine would block and the car would keep driving blind. Dropping frames keeps the control loop alive.

---

## Module Details

### 1. Web Server (`web_server.py`)

**Role**: Main process. Serves the web cockpit UI, streams video, manages recordings, and acts as the camera frame source for the entire pipeline.

**Endpoints:**

| Route | Method | Description |
|-------|--------|-------------|
| `/` | GET | Web cockpit HTML page |
| `/video_feed` | GET | MJPEG video stream |
| `/log_stream` | GET | SSE (Server-Sent Events) log feed |
| `/toggle_auto` | POST | Switch between Auto/Manual mode |
| `/toggle_record` | POST | Start/stop video recording |
| `/emergency` | POST | Send brake commands + shutdown |

**Recording**: Frames are written to AVI files in `../recordings/` using OpenCV's `VideoWriter`. The recording toggle is checked once per frame via shared memory, allowing the Xbox controller (X button) to start/stop recordings.

**Camera**: Uses `picamera2` library in a thread-safe singleton pattern. Flask's multi-threaded request handling requires this to prevent concurrent camera access.

### 2. AI Engine (`ai_engine.py`)

**Role**: The perception and control "brain." Consumes camera frames, runs YOLO inference, and produces steering/speed commands.

**Pipeline per frame:**

```
Frame → YOLOv11 Segmentation → Detection Scan → Speed Logic → Lane Centroid → PID → Serial Commands
         ↓                         ↓                ↓              ↓
    [Annotated frame]     [Stop/Crosswalk/      [Speed       [Steering
                           Highway/Parking]      decision]     angle]
```

**Speed Table:**

| Condition | Speed |
|-----------|-------|
| Normal driving | 100 |
| Highway mode | 200 |
| Crosswalk detected | 50 |
| Parking scan | 50 |
| Intersection turn | 50 |
| Stop sign active | 0 |
| Parking mode active | Controlled by parking FSM |

**Intersection Handling**: When the lane mask width exceeds 85% of the frame width at 70% frame height, an intersection is detected. The system uses **directional masking** — erasing portions of the lane mask to force the PID controller to steer in the desired direction:

- **RIGHT turn (2.5s)**: Masks left half of frame → centroid shifts right → car steers right
- **LEFT turn (8s)**: Two-phase — pull forward straight (first 5s, masking sides only), then hard left (3s, masking right 58% of frame + shifting target centroid to 220)
- **STRAIGHT (2s)**: Masks sides, keeps center corridor

**PID Controller**: P-only controller (`Kp=5`, `Ki=0`, `Kd=0`). Error is `lane_centroid_x - frame_center(160)`. Output clamped to `±250` (MAX_STEER_ANGLE).

### 3. Manual Control (`manual_control.py`)

**Role**: Reads Xbox controller HID events via `python-evdev` and overrides AI commands when the user takes control.

**Input Mapping:**

| Physical Control | Linux Event | Function |
|-----------------|-------------|----------|
| Left stick X | EV_ABS, code 0 | Steering (−32768 to +32767 → mapped to ±250) |
| Right trigger | EV_ABS, code 9 | Throttle (0 to 255 → mapped to 0–400) |
| A button | EV_KEY, code 304 | Toggle Auto/Manual |
| X button | EV_KEY, code 307 | Toggle recording |
| B button | EV_KEY, code 305 | Emergency brake + switch to manual |

**Anti-spam logic**: Commands are only sent when values change by more than a deadzone or when a 100ms heartbeat timer fires. This prevents serial flooding.

**Disconnect detection**: When `/dev/input/event*` disappears (controller unplugged), the process detects the error and auto mode remains off until the controller is reconnected and A is pressed.

### 4. Car Control (`car_control.py`)

**Role**: Serial protocol implementation and PID controller class.

**Serial Protocol:**

```
#speed:<int>;;\r\n   — Set motor speed (positive=forward, negative=reverse)
#steer:<int>;;\r\n   — Set steering angle (0=center, ±250=max)
#brake:0;;\r\n       — Emergency brake
```

**Serial Processes**:
- `read_serial_process()`: Continuously reads from serial. Filters "important" messages (containing specific keywords) to `log_queue`.
- `write_serial_process()`: Consumes `serial_queue` and writes commands to the Arduino. This serializes writes from multiple producers (AI engine, manual control, parking).

### 5. Parking System (`parking.py`)

**Role**: GPIO sensor-based autonomous parking state machine.

**Hardware**: 4 digital input sensors on Raspberry Pi GPIO:
- GPIO 4 — Right sensor
- GPIO 27 — Left sensor
- GPIO 22 — Front sensor
- GPIO 17 — Back sensor

**State Machine:**

```
IDLE ──[Parking sign detected]──> SCANNING (3s at speed 50)
                                       │
                        ┌──────────────┼──────────────┐
                        v              v              v
                   Left free?    Right free?     Both occupied
                        │              │              │
                        v              v              v
                  LEFT ROUTINE  RIGHT ROUTINE   MOVE_NEXT (1s)
                                                      │
                                                ┌─────┴─────┐
                                           Left/right free?  Both occupied
                                                │              │
                                           [park]         ABORT → IDLE
                                                                 (10s cooldown)
```

**Parking Routines (sequence of steps: speed, steer, duration):**

| Routine | Phase 1 | Phase 2 | Phase 3 | Phase 4 |
|---------|---------|---------|---------|---------|
| Right | Forward 50, steer 0, 1.5s | Reverse −50, steer +230, 3.5s | Reverse −50, steer −230, 3s | Stop, 1s |
| Left | Forward 50, steer 0, 1.5s | Reverse −50, steer +210, 3.5s | Reverse −50, steer −210, 3s | Stop, 1s |

**SmartSerial Helper**: Prevents redundant serial writes (only sends when values change). Implements an **ESC reverse-arming sequence** (neutral → brake pulse → neutral → reverse) required by the Quickrun Fusion SE motor for forward-to-reverse transitions.

### 6. Config (`config.py`)

Centralized configuration. All tunable constants are in this single file — no magic numbers elsewhere.

---

## Frame Drop Strategy

Every queue consumer uses a **"drain and keep newest"** pattern:

```python
frame = None
while not frame_queue.empty():
    try:
        frame = frame_queue.get_nowait()
    except queue.Empty:
        break
if frame is None:
    frame = frame_queue.get(timeout=1)
```

This ensures the AI engine always works on the freshest frame. If the YOLO inference runs at 10 FPS but the camera produces 30 FPS, old frames are discarded automatically.

---

## Fault Tolerance

- **AI crash**: Web server continues streaming. The serial writer still processes commands from manual control. Restart the AI process via the main process.
- **Serial disconnect**: `init_serial()` returns `None`. The serial reader/writer processes log errors and retry.
- **Camera failure**: Flask endpoint returns 500. The rest of the system remains responsive.
- **Parking sensor failure**: GPIO read returns 0 (occupied), so parking routines abort safely after retry.

---

## Startup Sequence

1. `main.py` creates all queues and shared memory objects
2. Serial connection initialized, calibration commands sent (steering endpoints, battery enable, center)
3. 4 child processes spawned
4. Flask web server starts in main process (blocking)
5. On `SIGINT` (Ctrl+C), all child processes are terminated and cleaned up

---

## Performance Considerations

- **320×320 resolution** for YOLO inference is a tradeoff: enough detail for lane/sign detection, small enough for ~10 FPS on Raspberry Pi 5 CPU
- **NCNN model format** provides faster inference than PyTorch on ARM
- **Non-blocking queues** prevent the pipeline from stalling under load
- **P-only PID** was chosen over PI/PD after testing — the I term caused oscillations on the variable track surface
