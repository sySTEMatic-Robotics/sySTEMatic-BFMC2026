from flask import Flask, Response, render_template_string, request, jsonify
from picamera2 import Picamera2
import cv2
import time
import os
import datetime
import logging
import signal
import threading
import config

# --- SILENCE FLASK LOGS ---
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# --- GLOBAL STATE ---
global_frame_q = None
global_result_q = None
global_log_q = None
global_serial_q = None
shared_recording = None
shared_auto_mode = None

# --- CAMERA SINGLETON ---
camera_instance = None
camera_lock = threading.Lock()

app = Flask(__name__)

# --- UI TEMPLATE ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AV Cockpit</title>
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" rel="stylesheet">
    <style>
        :root { --primary: #00f2ff; --danger: #ff0055; --bg: #0a0b10; --panel: #14161f; }
        body { background-color: var(--bg); color: #e0e0e0; font-family: 'Segoe UI', sans-serif; margin: 0; height: 100vh; display: flex; flex-direction: column; overflow: hidden; }
        
        .main-grid {
            display: grid;
            grid-template-columns: 3fr 1fr;
            grid-template-rows: 1fr auto;
            gap: 15px;
            padding: 15px;
            height: 100%;
            box-sizing: border-box;
        }

        /* Video Feed */
        .video-container {
            grid-column: 1 / 2;
            grid-row: 1 / 2;
            background: #000;
            border-radius: 8px;
            border: 1px solid #333;
            display: flex;
            align-items: center;
            justify-content: center;
            position: relative;
            overflow: hidden;
        }
        .video-feed { width: 100%; height: 100%; object-fit: contain; }
        
        .rec-indicator {
            position: absolute; top: 20px; right: 20px;
            background: var(--danger); color: white;
            padding: 5px 15px; border-radius: 20px;
            font-weight: bold; display: none;
            animation: blink 1s infinite;
        }

        /* Logs Panel */
        .log-container {
            grid-column: 2 / 3;
            grid-row: 1 / 3;
            background: var(--panel);
            border-radius: 8px;
            display: flex;
            flex-direction: column;
            border: 1px solid #333;
            min-width: 250px;
        }
        .log-header { padding: 10px; border-bottom: 1px solid #333; font-weight: bold; color: var(--primary); font-size: 14px; }
        
        #log-content { 
            flex: 1; overflow-y: auto; padding: 10px; 
            font-family: 'Courier New', monospace; font-size: 12px; color: #bbb; 
        }
        .log-entry { margin-bottom: 2px; border-bottom: 1px solid #222; word-wrap: break-word;}

        /* Controls */
        .controls-container {
            grid-column: 1 / 2;
            grid-row: 2 / 3;
            background: var(--panel);
            padding: 15px;
            border-radius: 8px;
            display: flex;
            gap: 20px;
            align-items: center;
            justify-content: space-between;
            border: 1px solid #333;
            height: 80px;
        }

        .btn-group { display: flex; gap: 15px; }
        .btn {
            padding: 0 25px; height: 50px; border: none; border-radius: 6px;
            cursor: pointer; font-weight: bold; font-size: 14px;
            display: flex; align-items: center; gap: 10px; color: white;
            transition: background 0.2s;
        }
        .btn-stop { background: var(--danger); box-shadow: 0 4px 15px rgba(255,0,85,0.3); }
        .btn-stop:active { transform: scale(0.95); }
        
        .btn-toggle { background: #333; border: 1px solid #555; }
        .btn-toggle.active { background: var(--primary); color: #000; border: none; box-shadow: 0 0 10px rgba(0, 242, 255, 0.3);}

        .status-badge {
            background: #222; padding: 10px 20px; border-radius: 6px;
            font-family: monospace; border: 1px solid #444; font-size: 16px;
        }
        
        @keyframes blink { 50% { opacity: 0.5; } }
    </style>
</head>
<body>
    <div class="main-grid">
        <div class="video-container">
            <div id="rec-status" class="rec-indicator">● REC</div>
            <img src="/video_feed" class="video-feed">
        </div>

        <div class="log-container">
            <div class="log-header">SYSTEM LOGS (Auto-clear 20s)</div>
            <div id="log-content"></div>
        </div>

        <div class="controls-container">
            <div class="btn-group">
                <button class="btn btn-stop" onclick="emergencyStop()">
                    <i class="fas fa-power-off"></i> SHUTDOWN
                </button>
            </div>
            <div class="status-badge" id="mode-display">MODE: AUTO</div>
            <div class="btn-group">
                <button id="btn-mode" class="btn btn-toggle active" onclick="toggleMode()">
                    <i class="fas fa-robot"></i> AUTO
                </button>
                <button id="btn-rec" class="btn btn-toggle" onclick="toggleRec()">
                    <i class="fas fa-video"></i> REC
                </button>
            </div>
        </div>
    </div>

    <script>
        const logBox = document.getElementById("log-content");
        
        const evtSource = new EventSource("/stream_logs");
        evtSource.onmessage = function(e) {
            const div = document.createElement("div");
            div.className = "log-entry";
            div.textContent = "> " + e.data;
            logBox.appendChild(div);
            
            if (logBox.children.length > 50) {
                logBox.removeChild(logBox.firstChild);
            }
            logBox.scrollTop = logBox.scrollHeight;
        };

        setInterval(function() {
            logBox.innerHTML = ''; 
            const div = document.createElement("div");
            div.className = "log-entry";
            div.style.color = "#888"; 
            div.textContent = "> [System] Logs cleared (20s timer)";
            logBox.appendChild(div);
        }, 20000);

        async function emergencyStop() { 
            await fetch('/action/stop'); 
            document.getElementById('mode-display').innerText = "SHUTTING DOWN...";
            document.getElementById('mode-display').style.color = "#ff0055";
            document.body.style.opacity = "0.5"; 
        }
        
        async function toggleMode() {
            const btn = document.getElementById('btn-mode');
            const display = document.getElementById('mode-display');
            const res = await fetch('/action/toggle_mode');
            const data = await res.json();
            
            if(data.auto_mode) {
                btn.classList.add('active');
                btn.innerHTML = '<i class="fas fa-robot"></i> AUTO';
                display.innerText = "MODE: AUTO";
                display.style.color = "#00f2ff";
            } else {
                btn.classList.remove('active');
                btn.innerHTML = '<i class="fas fa-gamepad"></i> MANUAL';
                display.innerText = "MODE: MANUAL";
                display.style.color = "#ffaa00";
            }
        }

        async function toggleRec() {
            const btn = document.getElementById('btn-rec');
            const indicator = document.getElementById('rec-status');
            const res = await fetch('/action/toggle_rec');
            const data = await res.json();
            
            if(data.recording) {
                btn.classList.add('active');
                btn.style.background = "#ff0055";
                indicator.style.display = "block";
            } else {
                btn.classList.remove('active');
                btn.style.background = "#333";
                indicator.style.display = "none";
            }
        }
    </script>
</body>
</html>
"""

# --- CAMERA SINGLETON ---
def get_camera():
    global camera_instance
    # Use a lock to ensure only one thread initializes the camera at a time
    with camera_lock:
        if camera_instance is None:
            try:
                camera_instance = Picamera2()
                config_list = camera_instance.create_video_configuration(main={"size": config.CAMERA_RES})
                camera_instance.configure(config_list)
                camera_instance.start()
                print("[Camera] Initialized Successfully.")
            except Exception as e:
                print(f"[Camera] Error: {e}")
                # If init fails, ensure we don't leave a broken object
                camera_instance = None
    return camera_instance

# --- VIDEO GENERATOR ---
def generate_frames():
    picam = get_camera()
    video_writer = None
    
    if not os.path.exists(config.RECORDING_PATH):
        try: os.makedirs(config.RECORDING_PATH)
        except: pass

    # If camera failed to init, return safely
    if picam is None:
        return

    while True:
        # Capture
        try:
            frame = picam.capture_array()
        except RuntimeError:
            # If camera disconnects, try to recover or just break
            break

        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Recording
        if shared_recording.value:
            if video_writer is None:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{config.RECORDING_PATH}/rec_{timestamp}.avi"
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                video_writer = cv2.VideoWriter(filename, fourcc, 20.0, config.CAMERA_RES)
                if global_log_q: global_log_q.put(f"Rec Start: {filename}")
            video_writer.write(frame)
        else:
            if video_writer is not None:
                video_writer.release()
                video_writer = None
                if global_log_q: global_log_q.put("Rec Saved.")

        # Send to AI
        try:
            if global_frame_q.full(): global_frame_q.get_nowait()
            global_frame_q.put_nowait(frame)
        except: pass

        # Get Result
        try:
            annotated_frame, _ = global_result_q.get(timeout=0.2)
            ret, buffer = cv2.imencode('.jpg', annotated_frame)
        except:
            ret, buffer = cv2.imencode('.jpg', frame)

        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# --- API ENDPOINTS ---

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream_logs')
def stream_logs():
    def event_stream():
        while True:
            while not global_log_q.empty():
                yield f"data: {global_log_q.get()}\n\n"
            time.sleep(0.1)
    return Response(event_stream(), mimetype='text/event-stream')

@app.route('/action/stop')
def action_stop():
    global_serial_q.put('#brake:0;;\r\n')
    global_serial_q.put('#speed:0;;\r\n')
    global_log_q.put("⚠️ SYSTEM SHUTDOWN TRIGGERED")

    def kill_server():
        time.sleep(1) 
        os.kill(os.getpid(), signal.SIGINT) 
        
    threading.Thread(target=kill_server).start()
    return jsonify(status="shutting_down")

@app.route('/action/toggle_mode')
def action_toggle_mode():
    shared_auto_mode.value = not shared_auto_mode.value
    status = "AUTO" if shared_auto_mode.value else "MANUAL"
    if not shared_auto_mode.value:
        global_serial_q.put('#speed:0;;\r\n')
    global_log_q.put(f"Mode: {status}")
    return jsonify(auto_mode=shared_auto_mode.value)

@app.route('/action/toggle_rec')
def action_toggle_rec():
    shared_recording.value = not shared_recording.value
    return jsonify(recording=shared_recording.value)

# --- SERVER STARTUP ---
def start_server(frame_q, result_q, log_q, serial_q, s_rec, s_auto):
    global global_frame_q, global_result_q, global_log_q, global_serial_q
    global shared_recording, shared_auto_mode
    
    global_frame_q = frame_q
    global_result_q = result_q
    global_log_q = log_q
    global_serial_q = serial_q
    shared_recording = s_rec
    shared_auto_mode = s_auto
    
    app.run(host=config.WEB_HOST, port=config.WEB_PORT, threaded=True)