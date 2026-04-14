"""
Military Weapon Detection + Vehicle / Robotic Arm Control
Run: python app.py
Open: http://localhost:5000
"""

from flask import Flask, Response, render_template_string, jsonify, request, send_from_directory
from ultralytics import YOLO
import cv2
import numpy as np
import io
import csv
import os
import time
import threading
from datetime import datetime
from collections import deque

# ---- Optional serial (Arduino) ----
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

app = Flask(__name__)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "weapon_detection_yolov12.pt")
SNAPSHOT_DIR = os.path.join(BASE_DIR, "snapshots")
os.makedirs(SNAPSHOT_DIR, exist_ok=True)

# ---- Config ----
CONTROL_PASSWORD = "1234"
SERIAL_PORT = os.environ.get("ARDUINO_PORT", "")  # e.g. "COM5"; auto-detect if empty
SERIAL_BAUD = 9600

model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names
WEAPON_CLASSES = {"gun", "pistol", "rifle", "knife", "Knife", "weapon", "guns"}
MILITARY_WEAPON_CLASSES = {"rifle", "gun", "guns"}  # person + these → military personnel

# Lightweight pre-filter: skip the heavy weapon model on frames that contain no persons.
# yolo12n.pt is the COCO nano model (~4 MB). Ultralytics auto-downloads it on first run.
_person_model_path = os.path.join(BASE_DIR, "yolo12n.pt")
person_model = YOLO(_person_model_path if os.path.exists(_person_model_path) else "yolo12n.pt")

# ── RPi performance knobs ────────────────────────────────────────────────────
_INFER_SIZE   = 320   # YOLO input resolution; 320 ≈ 4× faster than 640
_INFER_EVERY  = 2     # run full inference every N frames; reuse boxes otherwise
_JPEG_QUALITY = 65    # JPEG encode quality (65 is fine for surveillance)

# ── Shared MJPEG frame buffer ────────────────────────────────────────────────
# One inference loop runs in a background thread; all HTTP clients read from
# this single pre-encoded JPEG — no duplicate work per viewer.
_frame_lock  = threading.Lock()
_latest_jpeg = b''

# ── Model warm-up (eliminates first-frame latency on RPi) ───────────────────
print("  Warming up models...", flush=True)
_warmup = np.zeros((480, 640, 3), dtype=np.uint8)
person_model.predict(_warmup, imgsz=_INFER_SIZE, verbose=False)
model.predict(_warmup, imgsz=_INFER_SIZE, verbose=False)
del _warmup
print("  Models ready.", flush=True)

# ---- Shared state ----
state_lock = threading.Lock()
state = {
    "detections": [],
    "fps": 0,
    "alert": False,
    "conf_threshold": 0.35,
    "total_weapons_seen": 0,
    "events": deque(maxlen=200),  # full log; UI shows last 20
    "last_snapshot_time": 0,
    "view_mode": "normal",        # normal | night | thermal | gray
    "vehicle_speed": 220,         # 0-255 PWM for drive motors
    "persons": 0,
    "military": 0,
    "gps": {"lat": None, "lon": None, "accuracy": None, "ts": None},
    "alert_markers": [],  # list of {lat, lon, msg, time}
}

VIEW_MODES = {"normal", "night", "thermal", "gray"}

# ---- Serial connection to Arduino ----
arduino = None
arduino_lock = threading.Lock()


def init_serial():
    """Try to open the Arduino serial port. Non-fatal if it fails."""
    global arduino
    if not SERIAL_AVAILABLE:
        print("  [serial] pyserial not installed — control commands will be no-ops")
        return
    port = SERIAL_PORT
    if not port:
        ports = list(serial.tools.list_ports.comports())
        print(f"  [serial] Scanning {len(ports)} port(s)…")
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").upper()
            print(f"           {p.device} — {p.description} [{p.hwid}]")
            # Match by description keywords OR by Arduino/CH340 USB vendor IDs
            if ("arduino" in desc or "ch340" in desc or
                    "usb-serial" in desc or "wchusbserial" in desc or
                    "VID:PID=2341" in hwid or "VID:PID=1A86" in hwid):
                port = p.device
                break
    if not port:
        print("  [serial] No Arduino port found — set ARDUINO_PORT env var to override.")
        return
    try:
        arduino = serial.Serial(port, SERIAL_BAUD, timeout=0.1)
        time.sleep(2)  # let the board reset after DTR pulse
        print(f"  [serial] Connected → {port} @ {SERIAL_BAUD} baud")
    except Exception as e:
        print(f"  [serial] Failed to open {port}: {e}")
        arduino = None


def send_serial(cmd: str):
    """Send a single line command to the Arduino. Safe if not connected."""
    if arduino is None:
        return False
    try:
        with arduino_lock:
            arduino.write((cmd.strip() + "\n").encode("utf-8"))
        return True
    except Exception as e:
        print(f"  [serial] write error: {e}")
        return False


HTML_PAGE = r"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>TACTICAL // WEAPON DETECTION SYSTEM</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }

        :root {
            --bg-0: #060a05;
            --bg-1: #0b110a;
            --bg-2: #11180e;
            --bg-3: #182112;
            --border: #2a3a1a;
            --border-bright: #4a6b22;
            --olive: #6b8e3d;
            --olive-bright: #9bcc4a;
            --amber: #d4a017;
            --amber-bright: #f4c430;
            --text: #c8d6b9;
            --text-dim: #7d8f6c;
            --text-faint: #4a5d3a;
            --red: #dc2626;
            --red-bright: #ff3b30;
        }

        /* ── NO SCROLL — entire UI lives in 100vh ─────── */
        html, body {
            height: 100%;
            overflow: hidden;
            font-family: 'Consolas', 'Courier New', monospace;
            background: var(--bg-0);
            color: var(--text);
            font-size: 12px;
            -webkit-font-smoothing: antialiased;
        }

        body::before {
            content: '';
            position: fixed;
            inset: 0;
            background: repeating-linear-gradient(
                0deg,
                rgba(155, 204, 74, 0.018) 0px,
                rgba(155, 204, 74, 0.018) 1px,
                transparent 1px,
                transparent 3px
            );
            pointer-events: none;
            z-index: 9999;
        }

        /* ── Page shell ──────────────────────────────── */
        .page {
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        /* ── Classification banners ──────────────────── */
        .banner {
            flex-shrink: 0;
            height: 20px;
            line-height: 20px;
            background: #000;
            color: var(--amber-bright);
            text-align: center;
            font-size: 10px;
            font-weight: 700;
            letter-spacing: 4px;
            border-bottom: 1px solid var(--amber);
        }
        .banner.bottom { border-top: 1px solid var(--amber); border-bottom: none; }

        /* ── Header ──────────────────────────────────── */
        .header {
            flex-shrink: 0;
            height: 46px;
            background: linear-gradient(180deg, var(--bg-2), var(--bg-1));
            border-bottom: 2px solid var(--border-bright);
            padding: 0 16px;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }
        .header-left { display: flex; align-items: center; gap: 10px; }
        .insignia {
            width: 30px; height: 30px; flex-shrink: 0;
            border: 2px solid var(--olive-bright);
            display: flex; align-items: center; justify-content: center;
            color: var(--olive-bright); font-size: 14px; font-weight: 900;
            transform: rotate(45deg);
        }
        .insignia span { transform: rotate(-45deg); }
        .header h1 { font-size: 14px; font-weight: 800; color: var(--text); letter-spacing: 2.5px; text-transform: uppercase; }
        .header h1 .accent { color: var(--olive-bright); }
        .header .subtitle { font-size: 9px; color: var(--text-dim); letter-spacing: 2px; margin-top: 1px; }
        .header-right { display: flex; align-items: center; gap: 8px; }
        .meta-chip { font-size: 10px; color: var(--text-dim); padding: 4px 10px; background: var(--bg-2); border: 1px solid var(--border); letter-spacing: 1.5px; }
        .meta-chip strong { color: var(--olive-bright); }
        .status-badge { display: flex; align-items: center; gap: 6px; background: var(--bg-2); padding: 5px 10px; border: 1px solid var(--olive); font-size: 10px; color: var(--olive-bright); letter-spacing: 2px; font-weight: 700; }
        .status-dot { width: 7px; height: 7px; background: var(--olive-bright); border-radius: 50%; box-shadow: 0 0 6px var(--olive-bright); animation: pulse 1.4s infinite; }
        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.3} }
        .serial-chip { display:flex; align-items:center; gap:5px; padding:4px 8px; background:var(--bg-2); border:1px solid var(--border); font-size:9px; font-weight:700; letter-spacing:1.5px; cursor:pointer; transition:border-color 0.2s; }
        .serial-chip:hover { border-color:var(--olive); }
        .serial-chip .sdot { width:6px; height:6px; border-radius:50%; flex-shrink:0; }
        .serial-chip.online  { border-color:var(--olive); color:var(--olive-bright); }
        .serial-chip.online  .sdot { background:var(--olive-bright); box-shadow:0 0 5px var(--olive-bright); }
        .serial-chip.offline { border-color:var(--red); color:var(--red-bright); }
        .serial-chip.offline .sdot { background:var(--red-bright); }

        /* ── Main 2-column grid ──────────────────────── */
        .main {
            flex: 1;
            min-height: 0;
            display: grid;
            grid-template-columns: 1fr 308px;
            gap: 8px;
            padding: 8px;
            overflow: hidden;
        }

        /* ── LEFT COLUMN ─────────────────────────────── */
        .left-col { display: flex; flex-direction: column; min-height: 0; gap: 6px; }

        /* ── Feed container ──────────────────────────── */
        .feed-container {
            flex: 1; min-height: 0;
            display: flex; flex-direction: column;
            background: var(--bg-1); border: 1px solid var(--border);
            position: relative;
        }
        .feed-container::before { content:''; position:absolute; top:-1px; left:-1px; width:14px; height:14px; border-top:2px solid var(--olive-bright); border-left:2px solid var(--olive-bright); z-index:1; pointer-events:none; }
        .feed-container::after  { content:''; position:absolute; bottom:-1px; right:-1px; width:14px; height:14px; border-bottom:2px solid var(--olive-bright); border-right:2px solid var(--olive-bright); z-index:1; pointer-events:none; }

        .feed-header { flex-shrink:0; height:34px; padding:0 14px; background:var(--bg-2); border-bottom:1px solid var(--border); display:flex; justify-content:space-between; align-items:center; }
        .feed-header h2 { font-size:10px; font-weight:800; color:var(--olive-bright); letter-spacing:2.5px; text-transform:uppercase; }
        .feed-header h2::before { content:'\25B8 '; color:var(--olive); }
        .cam-label { font-size:9px; color:var(--amber); letter-spacing:1.5px; }

        .feed-frame { flex:1; min-height:0; position:relative; background:#000; overflow:hidden; }
        .feed-video { width:100%; height:100%; object-fit:contain; display:block; }

        .feed-frame::before { content:''; position:absolute; top:8px; left:8px; width:22px; height:22px; border-top:2px solid var(--olive-bright); border-left:2px solid var(--olive-bright); pointer-events:none; opacity:.85; z-index:2; }
        .feed-frame::after  { content:''; position:absolute; bottom:8px; right:8px; width:22px; height:22px; border-bottom:2px solid var(--olive-bright); border-right:2px solid var(--olive-bright); pointer-events:none; opacity:.85; z-index:2; }
        .crosshair-tr { position:absolute; top:8px; right:8px; width:22px; height:22px; border-top:2px solid var(--olive-bright); border-right:2px solid var(--olive-bright); pointer-events:none; opacity:.85; z-index:2; }
        .crosshair-bl { position:absolute; bottom:8px; left:8px; width:22px; height:22px; border-bottom:2px solid var(--olive-bright); border-left:2px solid var(--olive-bright); pointer-events:none; opacity:.85; z-index:2; }
        .feed-overlay { position:absolute; left:12px; top:36px; font-size:9px; color:var(--olive-bright); text-shadow:0 0 4px #000; letter-spacing:1.5px; line-height:1.7; pointer-events:none; z-index:2; }

        .feed-actions { flex-shrink:0; height:44px; padding:0 10px; background:var(--bg-2); border-top:1px solid var(--border); display:flex; align-items:center; gap:8px; }

        .conf-wrap { display:flex; align-items:center; gap:7px; flex:1; min-width:120px; }
        .conf-wrap label { font-size:9px; color:var(--text-dim); letter-spacing:1px; text-transform:uppercase; white-space:nowrap; }
        .conf-wrap input[type=range] { flex:1; accent-color:var(--olive-bright); }
        .conf-wrap .conf-value { font-size:10px; color:var(--olive-bright); min-width:34px; text-align:right; font-weight:700; }

        .view-modes { display:flex; gap:2px; background:var(--bg-2); border:1px solid var(--border); padding:2px; }
        .vm-btn { background:transparent; color:var(--text-dim); border:none; padding:3px 6px; font-family:inherit; font-size:9px; font-weight:700; letter-spacing:1px; cursor:pointer; transition:all .12s; text-transform:uppercase; }
        .vm-btn:hover { color:var(--olive-bright); }
        .vm-btn.active { background:var(--olive); color:#000; }

        /* ── Buttons ─────────────────────────────────── */
        .btn { background:var(--bg-3); color:var(--olive-bright); border:1px solid var(--border-bright); padding:6px 11px; font-family:inherit; font-size:10px; font-weight:800; letter-spacing:1.5px; text-transform:uppercase; cursor:pointer; transition:all .12s; white-space:nowrap; }
        .btn:hover { background:#1f2c14; border-color:var(--olive-bright); box-shadow:0 0 6px rgba(155,204,74,.3); }
        .btn:active { transform:translateY(1px); }
        .btn.sm { padding:4px 8px; font-size:9px; letter-spacing:1px; }
        .btn.unlock { background:linear-gradient(180deg,#2a1a05,#1a1003); border-color:var(--amber); color:var(--amber-bright); }
        .btn.unlock:hover { border-color:var(--amber-bright); box-shadow:0 0 8px rgba(244,196,48,.4); }
        .btn.danger { background:#2a0a0a; border-color:var(--red); color:var(--red-bright); }
        .btn.danger:hover { box-shadow:0 0 8px rgba(220,38,38,.5); }

        /* ── Controls panel (drive + arm, 2-col) ─────── */
        .controls-panel {
            flex-shrink: 0;
            display: none;
            height: 192px;
            background: var(--bg-1);
            border: 1px solid var(--border);
            grid-template-columns: 1fr 1fr;
            position: relative;
        }
        .controls-panel.active { display: grid; }
        .controls-panel::before { content:''; position:absolute; top:-1px; left:-1px; width:10px; height:10px; border-top:2px solid var(--olive); border-left:2px solid var(--olive); }
        .controls-panel::after  { content:''; position:absolute; bottom:-1px; right:-1px; width:10px; height:10px; border-bottom:2px solid var(--olive); border-right:2px solid var(--olive); }

        .ctrl-section { padding:8px 12px; display:flex; flex-direction:column; gap:5px; overflow:hidden; }
        .ctrl-section + .ctrl-section { border-left:1px solid var(--border); }
        .ctrl-title { font-size:9px; font-weight:800; color:var(--olive-bright); letter-spacing:2px; text-transform:uppercase; flex-shrink:0; padding-bottom:4px; border-bottom:1px solid var(--border); }
        .ctrl-title::before { content:'\25B8 '; color:var(--olive); }

        /* ── Joystick (compact) ──────────────────────── */
        .joystick-wrap { display:flex; align-items:center; gap:10px; flex:1; min-height:0; }
        .joystick-base {
            width:112px; height:112px; flex-shrink:0;
            border-radius:50%;
            background:radial-gradient(circle at center, var(--bg-3) 60%, var(--bg-2) 100%);
            border:2px solid var(--border-bright);
            position:relative; touch-action:none; user-select:none;
            box-shadow:0 0 12px rgba(155,204,74,.12), inset 0 0 20px rgba(0,0,0,.4);
        }
        .joystick-base::before { content:''; position:absolute; inset:5px; border-radius:50%; border:1px dashed var(--border); }
        .joystick-base::after {
            content:''; position:absolute; inset:0; border-radius:50%;
            background:
                linear-gradient(var(--border-bright),var(--border-bright)) center 6px/2px 12px no-repeat,
                linear-gradient(var(--border-bright),var(--border-bright)) center bottom 6px/2px 12px no-repeat,
                linear-gradient(var(--border-bright),var(--border-bright)) 6px center/12px 2px no-repeat,
                linear-gradient(var(--border-bright),var(--border-bright)) right 6px center/12px 2px no-repeat;
        }
        .joystick-knob {
            width:42px; height:42px; border-radius:50%;
            background:radial-gradient(circle at 35% 35%, var(--olive-bright), var(--olive) 60%, #3a5010);
            border:2px solid var(--olive-bright);
            box-shadow:0 0 10px rgba(155,204,74,.5), inset 0 0 6px rgba(255,255,255,.1);
            position:absolute; top:50%; left:50%;
            transform:translate(-50%,-50%);
            cursor:grab; transition:box-shadow .1s;
        }
        .joystick-knob:active { cursor:grabbing; }
        .joystick-knob.active { box-shadow:0 0 18px rgba(155,204,74,.9); }
        .joystick-right { display:flex; flex-direction:column; gap:6px; flex:1; }
        .joystick-cmd-display { font-size:10px; color:var(--text-dim); letter-spacing:1.5px; font-weight:700; }
        .joystick-cmd-display.moving { color:var(--olive-bright); }
        .pad-hint { font-size:9px; color:var(--text-faint); letter-spacing:1px; }
        .pad-hint kbd { background:var(--bg-2); border:1px solid var(--border); padding:1px 4px; color:var(--olive-bright); font-size:8px; }

        /* ── Servo rows ──────────────────────────────── */
        .servo-row { display:flex; align-items:center; gap:8px; }
        .servo-row label { font-size:9px; color:var(--text-dim); min-width:50px; text-transform:uppercase; letter-spacing:1.5px; font-weight:700; }
        .servo-row .val { font-size:9px; color:var(--olive-bright); min-width:34px; text-align:right; font-weight:700; }
        .arm-actions { display:flex; gap:4px; margin-top:2px; }
        .arm-actions .btn { flex:1; padding:4px 4px; font-size:9px; letter-spacing:1px; }
        /* ── Arm sliders ─────────────────────────────── */
        .servo-row input[type=range] { flex:1; accent-color:var(--olive-bright); background:transparent; height:14px; cursor:pointer; }
        .servo-row .deg-val { font-size:9px; color:var(--olive-bright); min-width:34px; text-align:right; font-weight:700; }

        /* ── RIGHT SIDEBAR ───────────────────────────── */
        .sidebar { display:flex; flex-direction:column; min-height:0; gap:6px; }

        /* ── Alert banner ────────────────────────────── */
        .alert-banner {
            display:none; flex-shrink:0;
            background:linear-gradient(180deg,#3a0a0a,#1f0505);
            border:1px solid var(--red); padding:8px 12px;
            position:relative; animation:alertFlash 1s infinite;
        }
        .alert-banner::before { content:''; position:absolute; top:-1px; left:-1px; width:10px; height:10px; border-top:2px solid var(--red-bright); border-left:2px solid var(--red-bright); }
        .alert-banner::after  { content:''; position:absolute; bottom:-1px; right:-1px; width:10px; height:10px; border-bottom:2px solid var(--red-bright); border-right:2px solid var(--red-bright); }
        .alert-banner.active { display:block; }
        @keyframes alertFlash { 0%,100%{box-shadow:0 0 10px rgba(220,38,38,.5)} 50%{box-shadow:0 0 22px rgba(220,38,38,.85)} }
        .alert-banner h3 { color:var(--red-bright); font-size:11px; letter-spacing:2px; font-weight:800; text-transform:uppercase; }
        .alert-banner p  { color:#fecaca; font-size:9px; letter-spacing:1px; text-transform:uppercase; margin-top:2px; }

        /* ── Stats 3×2 grid ──────────────────────────── */
        .stats-grid { flex-shrink:0; display:grid; grid-template-columns:repeat(3,1fr); gap:5px; }
        .stat-card { background:var(--bg-2); border:1px solid var(--border); padding:4px 4px; text-align:center; position:relative; }
        .stat-card::before { content:''; position:absolute; top:0; left:0; width:6px; height:6px; border-top:1px solid var(--olive); border-left:1px solid var(--olive); }
        .stat-value { font-size:20px; font-weight:900; color:var(--text); line-height:1; font-family:'Consolas',monospace; }
        .stat-value.danger  { color:var(--red-bright); text-shadow:0 0 5px rgba(220,38,38,.5); }
        .stat-value.success { color:var(--olive-bright); text-shadow:0 0 5px rgba(155,204,74,.4); }
        .stat-value.purple  { color:var(--amber-bright); text-shadow:0 0 5px rgba(244,196,48,.4); }
        .stat-value.military-stat { color:#ffb400; text-shadow:0 0 5px rgba(255,180,0,.5); }
        .stat-value.person-stat   { color:#6eb0f0; text-shadow:0 0 5px rgba(80,140,220,.4); }
        .stat-label { font-size:8px; color:var(--text-dim); margin-top:3px; text-transform:uppercase; letter-spacing:1px; }

        /* ── Panel base ──────────────────────────────── */
        .panel { background:var(--bg-1); border:1px solid var(--border); display:flex; flex-direction:column; min-height:0; position:relative; }
        .panel::before { content:''; position:absolute; top:-1px; left:-1px; width:10px; height:10px; border-top:2px solid var(--olive-bright); border-left:2px solid var(--olive-bright); }
        .panel::after  { content:''; position:absolute; bottom:-1px; right:-1px; width:10px; height:10px; border-bottom:2px solid var(--olive-bright); border-right:2px solid var(--olive-bright); }

        .panel-header { flex-shrink:0; height:30px; padding:0 12px; background:var(--bg-2); border-bottom:1px solid var(--border); display:flex; justify-content:space-between; align-items:center; font-size:10px; font-weight:800; color:var(--olive-bright); text-transform:uppercase; letter-spacing:2px; }
        .panel-header::before { content:'\25B8 '; color:var(--olive); margin-right:3px; }

        /* ── Map panel ───────────────────────────────── */
        .map-panel { flex:1; min-height:220px; }
        #mapContainer { flex:1; min-height:0; width:100%; position:relative; background:#0a0f08; overflow:hidden; }
        #tacticalMap { position:absolute; inset:0; filter:hue-rotate(100deg) saturate(.6) brightness(.75); }
        .map-gps-status { position:absolute; bottom:4px; left:6px; font-size:9px; color:var(--olive-bright); background:rgba(6,10,5,.85); padding:2px 6px; border:1px solid var(--border); pointer-events:none; z-index:1000; letter-spacing:1px; }
        .map-gps-status.acquiring { color:var(--amber); }
        #gpsCoords { font-size:9px; color:var(--text-dim); letter-spacing:.5px; font-weight:400; }
        .leaflet-popup-content-wrapper { background:var(--bg-2)!important; color:var(--text)!important; border:1px solid var(--border-bright)!important; border-radius:0!important; font-family:'Consolas',monospace!important; font-size:10px!important; }
        .leaflet-popup-tip { background:var(--bg-2)!important; }

        /* ── Detection panel ─────────────────────────── */
        .det-panel { flex-shrink:0; height:126px; }
        .detection-list { list-style:none; overflow-y:auto; flex:1; padding:4px 10px; }
        .detection-list::-webkit-scrollbar { width:4px; }
        .detection-list::-webkit-scrollbar-thumb { background:var(--olive); }
        .detection-list::-webkit-scrollbar-track { background:var(--bg-2); }
        .detection-item { display:flex; justify-content:space-between; align-items:center; padding:5px 0; border-bottom:1px dashed var(--border); }
        .detection-item:last-child { border-bottom:none; }
        .det-name { display:flex; align-items:center; gap:6px; }
        .det-icon { width:22px; height:22px; display:flex; align-items:center; justify-content:center; font-size:11px; font-weight:900; border:1px solid; flex-shrink:0; }
        .det-icon.weapon   { background:rgba(220,38,38,.12); color:var(--red-bright); border-color:var(--red); }
        .det-icon.other    { background:rgba(212,160,23,.12); color:var(--amber-bright); border-color:var(--amber); }
        .det-icon.military { background:rgba(255,180,0,.15); color:#ffb400; border-color:#c88000; }
        .det-icon.person   { background:rgba(80,140,220,.12); color:#6eb0f0; border-color:#3a6090; }
        .det-label { font-size:10px; font-weight:700; text-transform:uppercase; color:var(--text); letter-spacing:.5px; }
        .det-conf  { font-size:9px; color:var(--olive-bright); padding:2px 6px; background:var(--bg-2); border:1px solid var(--border); font-weight:700; }
        .no-detections { color:var(--text-faint); font-size:10px; text-align:center; padding:12px; letter-spacing:1.5px; text-transform:uppercase; }

        /* ── Events panel ────────────────────────────── */
        .events-panel { flex-shrink:0; height:98px; }
        .event-list { list-style:none; overflow-y:auto; flex:1; padding:4px 10px; }
        .event-list::-webkit-scrollbar { width:4px; }
        .event-list::-webkit-scrollbar-thumb { background:var(--olive); }
        .event-list::-webkit-scrollbar-track { background:var(--bg-2); }
        .event-item { padding:4px 0; border-bottom:1px dashed var(--border); font-size:9px; display:flex; justify-content:space-between; gap:8px; text-transform:uppercase; letter-spacing:.8px; }
        .event-item:last-child { border-bottom:none; }
        .event-time { color:var(--text-dim); flex-shrink:0; }
        .event-msg  { color:#fca5a5; flex:1; font-weight:700; }

        .log-actions { display:flex; gap:3px; }
        .log-btn { background:var(--bg-3); color:var(--olive-bright); border:1px solid var(--border); padding:2px 7px; font-family:inherit; font-size:8px; font-weight:700; letter-spacing:1px; text-decoration:none; cursor:pointer; text-transform:uppercase; }
        .log-btn:hover { border-color:var(--olive-bright); }
        .log-btn.off { color:var(--text-faint); }

        /* ── Fullscreen ──────────────────────────────── */
        .feed-container.fullscreen { position:fixed; inset:0; z-index:300; border:none; }
        .feed-container.fullscreen .feed-video { width:100vw; height:100vh; object-fit:contain; }
        .feed-container.fullscreen .feed-frame { height:100vh; }
        .feed-container.fullscreen .feed-actions { display:none; }
        .feed-container.fullscreen .feed-header { position:absolute; top:0; left:0; right:0; background:rgba(11,17,10,.85); z-index:2; }

        /* ── Login modal ─────────────────────────────── */
        .modal-bg { display:none; position:fixed; top:0; left:0; right:0; bottom:0; background:rgba(0,0,0,.85); backdrop-filter:blur(2px); z-index:500; justify-content:center; align-items:center; }
        .modal-bg.active { display:flex; }
        .modal { background:var(--bg-1); border:2px solid var(--olive-bright); padding:24px; width:320px; text-align:center; position:relative; box-shadow:0 0 40px rgba(155,204,74,.25); }
        .modal::before { content:''; position:absolute; top:-2px; left:-2px; width:16px; height:16px; border-top:3px solid var(--amber-bright); border-left:3px solid var(--amber-bright); }
        .modal::after  { content:''; position:absolute; bottom:-2px; right:-2px; width:16px; height:16px; border-bottom:3px solid var(--amber-bright); border-right:3px solid var(--amber-bright); }
        .modal .lock-icon { font-size:28px; color:var(--amber-bright); margin-bottom:4px; }
        .modal h2 { color:var(--olive-bright); margin-bottom:3px; font-size:13px; letter-spacing:4px; text-transform:uppercase; }
        .modal p { color:var(--text-dim); font-size:9px; margin-bottom:14px; letter-spacing:1.5px; text-transform:uppercase; }
        .modal input { width:100%; padding:12px; background:#000; border:1px solid var(--border-bright); color:var(--olive-bright); font-size:20px; text-align:center; letter-spacing:14px; margin-bottom:10px; font-family:'Consolas',monospace; font-weight:700; }
        .modal input:focus { outline:none; border-color:var(--olive-bright); box-shadow:0 0 8px rgba(155,204,74,.4); }
        .modal .err { color:var(--red-bright); font-size:9px; min-height:12px; margin-bottom:10px; letter-spacing:1.5px; text-transform:uppercase; font-weight:700; }
        .modal-actions { display:flex; gap:8px; }
        .modal-actions .btn { flex:1; }
        .modal .btn.cancel { background:var(--bg-3); color:var(--text-dim); border-color:var(--border); }

    </style>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
</head>
<body><div class="page">

    <div class="banner">// UNCLASSIFIED // FOR OFFICIAL USE ONLY //</div>

    <div class="header">
        <div class="header-left">
            <div class="insignia"><span>&#9876;</span></div>
            <div>
                <h1><span class="accent">[</span> TACTICAL WEAPON DETECTION <span class="accent">]</span></h1>
                <div class="subtitle">YOLOv12 // OPS UNIT 01 // SECTOR 07-G</div>
            </div>
        </div>
        <div class="header-right">
            <div class="meta-chip">SYS <strong id="sysClock">--:--:--</strong></div>
            <div class="meta-chip">FPS <strong id="hdrFps">0</strong></div>
            <div class="serial-chip offline" id="serialChip" title="Click to reconnect">
                <div class="sdot"></div>
                <span id="serialChipTxt">SERIAL OFFLINE</span>
            </div>
            <div class="status-badge"><div class="status-dot"></div> OPERATIONAL</div>
        </div>
    </div>

    <div class="main" id="mainGrid">

        <!-- ═══ LEFT COLUMN ═══════════════════════════════ -->
        <div class="left-col">

            <div class="feed-container">
                <div class="feed-header">
                    <h2>LIVE OPTICAL FEED</h2>
                    <span class="cam-label">CAM-01 // RECON</span>
                </div>
                <div class="feed-frame">
                    <img src="/video_feed" class="feed-video" alt="Live Feed">
                    <div class="crosshair-tr"></div>
                    <div class="crosshair-bl"></div>
                    <div class="feed-overlay">REC &#9679;<br>ZOOM 1.0x<br>AI: ARMED</div>
                </div>
                <div class="feed-actions">
                    <div class="conf-wrap">
                        <label for="confSlider">THRESHOLD</label>
                        <input type="range" id="confSlider" min="10" max="90" value="35" step="5">
                        <span class="conf-value" id="confValue">35%</span>
                    </div>
                    <div class="view-modes" id="viewModes">
                        <button class="vm-btn active" data-mode="normal">NRML</button>
                        <button class="vm-btn" data-mode="night">NIGHT</button>
                        <button class="vm-btn" data-mode="thermal">THRM</button>
                        <button class="vm-btn" data-mode="gray">GRAY</button>
                    </div>
                    <button class="btn sm" id="fullscreenBtn" title="Fullscreen [F]">&#9974; FULL</button>
                    <button class="btn sm unlock" id="unlockBtn">&#9919; CONTROL</button>
                    <button class="btn sm danger" id="lockBtn" style="display:none;">&#9919; LOCK</button>
                </div>
            </div>

            <!-- Drive + Arm (revealed on unlock) -->
            <div class="controls-panel" id="controlsPanel">
                <div class="ctrl-section">
                    <div class="ctrl-title">VEHICLE DRIVE</div>
                    <div class="joystick-wrap">
                        <div class="joystick-base" id="joystickBase">
                            <div class="joystick-knob" id="joystickKnob"></div>
                        </div>
                        <div class="joystick-right">
                            <div class="joystick-cmd-display" id="joystickCmdDisplay">STANDBY</div>
                            <div class="servo-row">
                                <label>SPEED</label>
                                <input type="range" id="speedSlider" min="60" max="255" value="220" step="5">
                                <span class="val" id="speedVal">220</span>
                            </div>
                            <div class="pad-hint"><kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> / ARROWS</div>
                        </div>
                    </div>
                </div>
                <div class="ctrl-section">
                    <div class="ctrl-title">ROBOTIC ARM</div>
                    <div class="servo-row">
                        <label>BASE</label>
                        <input type="range" min="0" max="180" value="0" id="baseSlider">
                        <span class="deg-val" id="baseValDisplay">0&deg;</span>
                    </div>
                    <div class="servo-row">
                        <label>ELBOW</label>
                        <input type="range" min="0" max="180" value="0" id="elbowSlider">
                        <span class="deg-val" id="elbowValDisplay">0&deg;</span>
                    </div>
                    <div class="servo-row" style="margin-top:2px;">
                        <label>GRIPPER</label>
                        <button class="btn grip-toggle" id="gripToggleBtn" style="flex:1;padding:5px 6px;">&#9632; CLOSED</button>
                    </div>
                    <div class="arm-actions">
                        <button class="btn" id="armHome">HOME</button>
                    </div>
                </div>
            </div>

        </div><!-- end left-col -->

        <!-- ═══ RIGHT SIDEBAR ══════════════════════════════ -->
        <div class="sidebar" id="sidebar">

            <div class="alert-banner" id="alertBanner">
                <h3>&#9888; HOSTILE DETECTED</h3>
                <p id="alertText">Threat acquired in optical feed</p>
            </div>

            <div class="stats-grid">
                <div class="stat-card"><div class="stat-value danger" id="weaponCount">0</div><div class="stat-label">WEAPONS</div></div>
                <div class="stat-card"><div class="stat-value danger" id="armedCount">0</div><div class="stat-label">ARMED</div></div>
                <div class="stat-card"><div class="stat-value military-stat" id="militaryCount">0</div><div class="stat-label">MILITARY</div></div>
                <div class="stat-card"><div class="stat-value person-stat" id="personCount">0</div><div class="stat-label">PERSONS</div></div>
                <div class="stat-card"><div class="stat-value success" id="totalSeen">0</div><div class="stat-label">ALERTS</div></div>
                <div class="stat-card"><div class="stat-value purple" id="fpsCount">0</div><div class="stat-label">FPS</div></div>
            </div>

            <div class="panel map-panel">
                <div class="panel-header">TACTICAL MAP <span id="gpsCoords">NO FIX</span></div>
                <div id="mapContainer">
                    <div id="tacticalMap"></div>
                    <div class="map-gps-status acquiring" id="mapGpsStatus">&#11044; ACQUIRING GPS...</div>
                </div>
            </div>

            <div class="panel det-panel">
                <div class="panel-header">TARGET ACQUISITION</div>
                <ul class="detection-list" id="detectionList">
                    <li class="no-detections">SCANNING...</li>
                </ul>
            </div>

            <div class="panel events-panel">
                <div class="panel-header">EVENT LOG
                    <span class="log-actions">
                        <a href="/gallery" target="_blank" class="log-btn">GALLERY</a>
                        <a href="/events.csv" class="log-btn">CSV</a>
                        <button id="soundBtn" class="log-btn">SND&nbsp;ON</button>
                    </span>
                </div>
                <ul class="event-list" id="eventList">
                    <li class="no-detections">NO EVENTS</li>
                </ul>
            </div>

        </div><!-- end sidebar -->

    </div><!-- end main -->

    <div class="banner bottom">// UNCLASSIFIED // FOR OFFICIAL USE ONLY //</div>

</div><!-- end page -->

<!-- Login modal -->
<div class="modal-bg" id="modalBg">
    <div class="modal">
        <div class="lock-icon">&#9919;</div>
        <h2>RESTRICTED ACCESS</h2>
        <p>// AUTHORIZATION REQUIRED //</p>
        <input type="password" id="pwInput" maxlength="8" placeholder="&bull;&bull;&bull;&bull;" autocomplete="off">
        <div class="err" id="pwErr"></div>
        <div class="modal-actions">
            <button class="btn cancel" id="pwCancel">CANCEL</button>
            <button class="btn" id="pwOk">AUTHORIZE</button>
        </div>
    </div>
</div>

<script>
    // ═══════════════════════════════════════════════════
    //  TACTICAL MAP (Leaflet)
    // ═══════════════════════════════════════════════════
    let tacticalMap = null;
    let roverMarker = null;
    let alertMarkers = [];

    function initMap(lat, lon) {
        if (tacticalMap) return;
        const startLat = (lat !== null && lat !== undefined) ? lat : 20;
        const startLon = (lon !== null && lon !== undefined) ? lon : 0;
        const zoom     = (lat !== null && lat !== undefined) ? 16 : 3;
        tacticalMap = L.map('tacticalMap', {
            zoomControl: true,
            attributionControl: false,
            scrollWheelZoom: false,
            dragging: true,
            tap: true
        }).setView([startLat, startLon], zoom);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19
        }).addTo(tacticalMap);

        const roverIcon = L.divIcon({
            html: '<div style="width:14px;height:14px;background:#9bcc4a;border:2px solid #000;border-radius:50%;box-shadow:0 0 8px #9bcc4a;"></div>',
            iconSize: [14, 14], iconAnchor: [7, 7]
        });
        roverMarker = L.marker([startLat, startLon], { icon: roverIcon })
            .addTo(tacticalMap)
            .bindPopup('<b>ROVER</b>');
    }

    function ensureMap() {
        if (!tacticalMap) initMap(null, null);
        setTimeout(() => { if (tacticalMap) tacticalMap.invalidateSize(); }, 100);
    }

    function updateRoverMarker(lat, lon) {
        if (!tacticalMap) { initMap(lat, lon); return; }
        roverMarker.setLatLng([lat, lon]);
        tacticalMap.setView([lat, lon], tacticalMap.getZoom() < 14 ? 16 : tacticalMap.getZoom());
    }

    function addAlertMarker(lat, lon, msg, time) {
        if (!tacticalMap) return;
        const alertIcon = L.divIcon({
            html: '<div style="width:12px;height:12px;background:#dc2626;border:2px solid #fff;border-radius:50%;box-shadow:0 0 8px #dc2626;"></div>',
            iconSize: [12, 12], iconAnchor: [6, 6]
        });
        const m = L.marker([lat, lon], { icon: alertIcon })
            .addTo(tacticalMap)
            .bindPopup(`<b>${msg}</b><br>${time}`);
        alertMarkers.push(m);
    }

    function syncMapMarkers(serverMarkers) {
        if (!tacticalMap || serverMarkers.length === alertMarkers.length) return;
        const toAdd = serverMarkers.slice(alertMarkers.length);
        toAdd.forEach(m => addAlertMarker(m.lat, m.lon, m.msg, m.time));
    }

    setInterval(() => {
        fetch('/gps_markers').then(r => r.json()).then(data => {
            const g = data.gps;
            const status = document.getElementById('mapGpsStatus');
            const coords = document.getElementById('gpsCoords');
            if (g.lat !== null) {
                updateRoverMarker(g.lat, g.lon, g.accuracy);
                status.textContent = '\u25CF GPS LOCKED  acc:' + Math.round(g.accuracy||0) + 'm';
                status.classList.remove('acquiring');
                coords.textContent = g.lat.toFixed(5) + ', ' + g.lon.toFixed(5);
            } else {
                status.textContent = '\u25CF ACQUIRING GPS...';
                status.classList.add('acquiring');
                coords.textContent = 'NO FIX';
            }
            syncMapMarkers(data.markers);
        }).catch(() => {});
    }, 3000);

    (function startGPS() {
        if (!navigator.geolocation) return;
        navigator.geolocation.watchPosition(
            pos => {
                fetch('/location', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ lat: pos.coords.latitude, lon: pos.coords.longitude, accuracy: pos.coords.accuracy })
                }).catch(() => {});
            },
            err => console.warn('GPS:', err.message),
            { enableHighAccuracy: true, maximumAge: 2000, timeout: 10000 }
        );
    })();

    window.addEventListener('resize', () => { if (tacticalMap) tacticalMap.invalidateSize(); });

    const WEAPON_CLASSES = ['gun', 'guns', 'pistol', 'rifle', 'knife', 'Knife', 'weapon'];
    let unlocked = false;

    function getIcon(cls) {
        if (cls === 'military personnel')  return ['★', 'military'];
        if (cls === 'armed person')         return ['!', 'weapon'];
        if (cls === 'person')               return ['◉', 'person'];
        if (WEAPON_CLASSES.includes(cls))   return ['⚠', 'weapon'];
        return ['?', 'other'];
    }

    // Confidence slider
    const slider = document.getElementById('confSlider');
    const confValue = document.getElementById('confValue');
    slider.addEventListener('input', () => { confValue.textContent = slider.value + '%'; });
    slider.addEventListener('change', () => { fetch('/set_conf?value=' + (slider.value / 100)); });

    // ---- Login modal ----
    const modalBg  = document.getElementById('modalBg');
    const pwInput  = document.getElementById('pwInput');
    const pwErr    = document.getElementById('pwErr');
    const unlockBtn = document.getElementById('unlockBtn');
    const lockBtn   = document.getElementById('lockBtn');

    function openModal() {
        pwInput.value = ''; pwErr.textContent = '';
        modalBg.classList.add('active');
        setTimeout(() => pwInput.focus(), 50);
    }
    function closeModal() { modalBg.classList.remove('active'); }

    function setUnlocked(val) {
        unlocked = val;
        const ctrl = document.getElementById('controlsPanel');
        if (val) {
            ctrl.classList.add('active');
            unlockBtn.style.display = 'none';
            lockBtn.style.display = 'inline-block';
            setTimeout(initJoystick, 50);
        } else {
            ctrl.classList.remove('active');
            unlockBtn.style.display = 'inline-block';
            lockBtn.style.display = 'none';
        }
    }

    unlockBtn.addEventListener('click', openModal);
    lockBtn.addEventListener('click', () => setUnlocked(false));
    document.getElementById('pwCancel').addEventListener('click', closeModal);
    document.getElementById('pwOk').addEventListener('click', tryUnlock);
    pwInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') tryUnlock(); });

    function tryUnlock() {
        fetch('/unlock', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({password: pwInput.value})
        }).then(r => r.json()).then(d => {
            if (d.ok) { setUnlocked(true); closeModal(); }
            else { pwErr.textContent = 'Incorrect password'; pwInput.value = ''; pwInput.focus(); }
        });
    }

    // Serial status chip
    const serialChip = document.getElementById('serialChip');
    const serialTxt  = document.getElementById('serialChipTxt');
    function refreshSerial() {
        fetch('/serial_status').then(r => r.json()).then(d => {
            serialChip.className = 'serial-chip ' + (d.connected ? 'online' : 'offline');
            serialTxt.textContent = d.connected ? ('SERIAL \u25B8 ' + d.port) : 'SERIAL OFFLINE';
        }).catch(() => {});
    }
    refreshSerial();
    setInterval(refreshSerial, 4000);
    serialChip.addEventListener('click', () => {
        serialTxt.textContent = 'CONNECTING\u2026';
        fetch('/serial_reconnect').then(r => r.json()).then(d => {
            serialChip.className = 'serial-chip ' + (d.connected ? 'online' : 'offline');
            serialTxt.textContent = d.connected ? ('SERIAL \u25B8 ' + d.port) : 'SERIAL OFFLINE';
        }).catch(() => { serialTxt.textContent = 'SERIAL OFFLINE'; });
    });

    function sendCmd(cmd) {
        if (!unlocked) return;
        fetch('/cmd?c=' + encodeURIComponent(cmd))
            .then(r => r.json())
            .then(d => { if (!d.sent) console.warn('[cmd] not sent to Arduino:', cmd, d); })
            .catch(e => console.error('[cmd] fetch error:', cmd, e));
    }

    // ════════════════════════════════════════════════════
    //  DIGITAL JOYSTICK
    // ════════════════════════════════════════════════════
    let joystickReady = false;
    function initJoystick() {
        if (joystickReady) return;
        const base  = document.getElementById('joystickBase');
        const knob  = document.getElementById('joystickKnob');
        const disp  = document.getElementById('joystickCmdDisplay');
        if (!base) return;
        joystickReady = true;

        const DEAD = 0.22;
        const DIAG = 0.42;
        let dragging = false, lastCmd = null, sendTimer = null, baseRect = null;

        function getRadius() { return base.offsetWidth / 2 * 0.72; }

        function getCmd(nx, ny) {
            if (Math.sqrt(nx*nx + ny*ny) < DEAD) return 'W';
            const ax = Math.abs(nx), ay = Math.abs(ny);
            if (ay > ax * (1 + DIAG)) return ny < 0 ? 'F' : 'B';
            if (ax > ay * (1 + DIAG)) return nx < 0 ? 'L' : 'R';
            if (ny < 0 && nx < 0) return 'H';
            if (ny < 0 && nx > 0) return 'J';
            if (ny > 0 && nx < 0) return 'G';
            return 'I';
        }

        const CMD_LABELS = {
            F:'FWD ▲', B:'BACK ▼', L:'◄ LEFT', R:'RIGHT ►',
            H:'◄ FWD-L', J:'FWD-R ►', G:'◄ BACK-L', I:'BACK-R ►', W:'STANDBY'
        };

        function move(cx, cy) {
            const R = getRadius();
            const bx = baseRect.left + baseRect.width  / 2;
            const by = baseRect.top  + baseRect.height / 2;
            let dx = cx - bx, dy = cy - by;
            const dist = Math.sqrt(dx*dx + dy*dy);
            if (dist > R) { dx *= R/dist; dy *= R/dist; }
            knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
            const cmd = getCmd(dx/R, dy/R);
            disp.textContent = CMD_LABELS[cmd] || cmd;
            disp.className   = 'joystick-cmd-display' + (cmd !== 'W' ? ' moving' : '');
            if (cmd !== lastCmd) { lastCmd = cmd; sendCmd(cmd); }
        }

        function release() {
            if (!dragging) return;
            dragging = false;
            knob.classList.remove('active');
            knob.style.transform = 'translate(-50%, -50%)';
            disp.textContent = 'STANDBY';
            disp.className   = 'joystick-cmd-display';
            if (sendTimer) { clearInterval(sendTimer); sendTimer = null; }
            lastCmd = 'W';
            // Send stop 3 times to guarantee Arduino receives it
            sendCmd('W');
            setTimeout(() => sendCmd('W'), 120);
            setTimeout(() => sendCmd('W'), 280);
        }

        function startDrag(cx, cy) {
            baseRect = base.getBoundingClientRect();
            dragging = true;
            knob.classList.add('active');
            move(cx, cy);
            if (sendTimer) clearInterval(sendTimer);
            // Repeat current cmd including 'W' (dead zone) so Arduino always knows state
            sendTimer = setInterval(() => {
                if (dragging && lastCmd) sendCmd(lastCmd);
            }, 120);
        }

        base.addEventListener('mousedown', e => { e.preventDefault(); startDrag(e.clientX, e.clientY); });
        window.addEventListener('mousemove', e => { if (dragging) move(e.clientX, e.clientY); });
        window.addEventListener('mouseup', release);
        base.addEventListener('touchstart', e => { e.preventDefault(); const t = e.changedTouches[0]; startDrag(t.clientX, t.clientY); }, {passive: false});
        window.addEventListener('touchmove', e => { if (!dragging) return; e.preventDefault(); const t = e.changedTouches[0]; move(t.clientX, t.clientY); }, {passive: false});
        window.addEventListener('touchend',   release);
        window.addEventListener('touchcancel', release);

        // Safety: stop motors if page loses focus or user switches tab
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) { release(); sendCmd('W'); }
        });
        window.addEventListener('blur', () => { release(); sendCmd('W'); });
    }

    // Keyboard control
    const keyMap = {ArrowUp:'F', ArrowDown:'B', ArrowLeft:'L', ArrowRight:'R', w:'F', s:'B', a:'L', d:'R', W:'F', S:'B', A:'L', D:'R'};
    const keysDown = new Set();
    document.addEventListener('keydown', (e) => {
        if (!unlocked) return;
        const tag = document.activeElement && document.activeElement.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA') return;
        const c = keyMap[e.key];
        if (c && !keysDown.has(e.key)) { keysDown.add(e.key); sendCmd(c); }
    });
    document.addEventListener('keyup', (e) => {
        if (!unlocked) return;
        if (keyMap[e.key]) {
            keysDown.delete(e.key);
            if (keysDown.size === 0) {
                sendCmd('W');
                setTimeout(() => sendCmd('W'), 120);
                setTimeout(() => sendCmd('W'), 280);
            } else sendCmd(keyMap[keysDown.values().next().value]);
        }
    });

    // Robotic arm
    const armState = {base: 0, elbow: 0, gripper: 90};
    let armSendPending = false, armLastSent = 0;
    const ARM_INTERVAL = 120;
    function sendArm(force) {
        const now = Date.now(), dt = now - armLastSent;
        if (force || dt >= ARM_INTERVAL) {
            armLastSent = now; armSendPending = false;
            sendCmd(`ARM:${armState.base},${armState.elbow},${armState.gripper}`);
        } else if (!armSendPending) {
            armSendPending = true;
            setTimeout(() => sendArm(true), ARM_INTERVAL - dt);
        }
    }
    // ── Arm sliders ───────────────────────────────────────
    const baseSlider  = document.getElementById('baseSlider');
    const elbowSlider = document.getElementById('elbowSlider');

    function syncSliderDisplay(id, val) {
        document.getElementById(id).textContent = val + '\u00B0';
    }

    baseSlider.addEventListener('input', () => {
        armState.base = parseInt(baseSlider.value);
        syncSliderDisplay('baseValDisplay', armState.base);
        sendArm(false);
    });
    baseSlider.addEventListener('change', () => sendArm(true));

    elbowSlider.addEventListener('input', () => {
        armState.elbow = parseInt(elbowSlider.value);
        syncSliderDisplay('elbowValDisplay', armState.elbow);
        sendArm(false);
    });
    elbowSlider.addEventListener('change', () => sendArm(true));

    function setArm(b, el, gr) {
        armState.base = b; armState.elbow = el; armState.gripper = gr;
        baseSlider.value = b; syncSliderDisplay('baseValDisplay', b);
        elbowSlider.value = el; syncSliderDisplay('elbowValDisplay', el);
        const isClose = (gr >= 90);
        const gb = document.getElementById('gripToggleBtn');
        if (gb) {
            gb.textContent = isClose ? '\u25A0 CLOSED' : '\u25A1 OPEN';
            gb.style.borderColor = isClose ? 'var(--red)' : 'var(--olive-bright)';
            gb.style.color = isClose ? 'var(--red-bright)' : 'var(--olive-bright)';
        }
        sendArm(true);
    }
    document.getElementById('armHome').addEventListener('click', () => setArm(0, 0, 90));

    // Gripper toggle button
    const gripBtn = document.getElementById('gripToggleBtn');
    gripBtn.addEventListener('click', () => {
        const isOpen = (armState.gripper < 90);   // currently open → click = close
        armState.gripper = isOpen ? 90 : 0;
        gripBtn.textContent = isOpen ? '\u25A0 CLOSED' : '\u25A1 OPEN';
        gripBtn.style.borderColor = isOpen ? 'var(--red)' : 'var(--olive-bright)';
        gripBtn.style.color = isOpen ? 'var(--red-bright)' : 'var(--olive-bright)';
        sendArm(true);
    });
    // Init arm display
    syncSliderDisplay('baseValDisplay', armState.base);
    syncSliderDisplay('elbowValDisplay', armState.elbow);

    // Header clock
    function tickClock() {
        const d = new Date(), pad = n => (n < 10 ? '0' : '') + n;
        const t = pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds());
        const el = document.getElementById('sysClock');
        if (el) el.textContent = t;
    }
    tickClock();
    setInterval(tickClock, 1000);

    // Speed slider
    const speedSlider = document.getElementById('speedSlider');
    const speedVal = document.getElementById('speedVal');
    let speedSendPending = false, speedLastSent = 0;
    function sendSpeed(force) {
        const now = Date.now(), dt = now - speedLastSent;
        if (force || dt >= 150) {
            speedLastSent = now; speedSendPending = false;
            sendCmd('SPD:' + speedSlider.value);
        } else if (!speedSendPending) {
            speedSendPending = true;
            setTimeout(() => sendSpeed(true), 150 - dt);
        }
    }
    speedSlider.addEventListener('input', () => { speedVal.textContent = speedSlider.value; sendSpeed(false); });
    speedSlider.addEventListener('change', () => sendSpeed(true));

    // View mode buttons
    document.querySelectorAll('.vm-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.vm-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            fetch('/set_view?mode=' + btn.dataset.mode).catch(() => {});
        });
    });

    // Fullscreen toggle
    const feedContainer = document.querySelector('.feed-container');
    const fsBtn = document.getElementById('fullscreenBtn');
    function toggleFullscreen() { feedContainer.classList.toggle('fullscreen'); }
    fsBtn.addEventListener('click', toggleFullscreen);
    document.addEventListener('keydown', (e) => {
        if (e.key === 'f' || e.key === 'F') {
            const tag = document.activeElement && document.activeElement.tagName;
            if (tag === 'INPUT' || tag === 'TEXTAREA') return;
            toggleFullscreen();
        }
        if (e.key === 'Escape' && feedContainer.classList.contains('fullscreen')) {
            feedContainer.classList.remove('fullscreen');
        }
    });

    // Alarm sound
    let soundOn = true, audioCtx = null, lastAlarmAt = 0;
    const soundBtn = document.getElementById('soundBtn');
    soundBtn.addEventListener('click', () => {
        soundOn = !soundOn;
        soundBtn.textContent = soundOn ? 'SND\u00A0ON' : 'SND\u00A0OFF';
        soundBtn.classList.toggle('off', !soundOn);
        if (soundOn && !audioCtx) {
            try { audioCtx = new (window.AudioContext || window.webkitAudioContext)(); } catch(e) { audioCtx = null; }
        }
    });
    function playAlarm() {
        if (!soundOn) return;
        const now = Date.now();
        if (now - lastAlarmAt < 1500) return;
        lastAlarmAt = now;
        if (!audioCtx) {
            try { audioCtx = new (window.AudioContext || window.webkitAudioContext)(); } catch(e) { return; }
        }
        const beep = (freq, start, dur) => {
            const osc = audioCtx.createOscillator();
            const gain = audioCtx.createGain();
            osc.type = 'square'; osc.frequency.value = freq;
            osc.connect(gain); gain.connect(audioCtx.destination);
            gain.gain.setValueAtTime(0.0001, audioCtx.currentTime + start);
            gain.gain.exponentialRampToValueAtTime(0.25, audioCtx.currentTime + start + 0.01);
            gain.gain.exponentialRampToValueAtTime(0.0001, audioCtx.currentTime + start + dur);
            osc.start(audioCtx.currentTime + start);
            osc.stop(audioCtx.currentTime + start + dur + 0.02);
        };
        beep(880, 0, 0.12);
        beep(660, 0.16, 0.12);
    }

    // Detection polling
    let prevAlertActive = false;
    function updateDetections() {
        fetch('/detections')
            .then(r => r.json())
            .then(data => {
                const list = document.getElementById('detectionList');
                const alert = document.getElementById('alertBanner');
                const alertText = document.getElementById('alertText');

                let weapons = 0, armed = 0, military = 0, persons = 0;

                if (data.detections.length === 0) {
                    list.innerHTML = '<li class="no-detections">No threats detected</li>';
                    alert.classList.remove('active');
                    prevAlertActive = false;
                } else {
                    let html = '';
                    data.detections.forEach(d => {
                        const [icon, type] = getIcon(d.class);
                        if (WEAPON_CLASSES.includes(d.class)) weapons++;
                        if (d.class === 'armed person') armed++;
                        if (d.class === 'military personnel') military++;
                        if (d.class === 'person') persons++;
                        const confDisplay = (d.class === 'armed person' || d.class === 'military personnel' || d.class === 'person')
                            ? (d.class === 'military personnel' ? 'MILITARY' : d.class === 'armed person' ? 'THREAT' : 'CIVILIAN')
                            : (d.confidence * 100).toFixed(0) + '%';
                        html += `<li class="detection-item">
                            <div class="det-name">
                                <div class="det-icon ${type}">${icon}</div>
                                <span class="det-label">${d.class}</span>
                            </div>
                            <span class="det-conf">${confDisplay}</span>
                        </li>`;
                    });
                    list.innerHTML = html;

                    if (weapons > 0 || armed > 0 || military > 0) {
                        alert.classList.add('active');
                        const parts = [];
                        if (military > 0) parts.push(`${military} military`);
                        if (armed > 0) parts.push(`${armed} armed`);
                        if (weapons > 0) parts.push(`${weapons} weapon(s)`);
                        alertText.textContent = parts.join(' & ') + ' detected';
                        if (!prevAlertActive) playAlarm();
                        prevAlertActive = true;
                    } else {
                        alert.classList.remove('active');
                        prevAlertActive = false;
                    }
                }

                document.getElementById('weaponCount').textContent = weapons;
                document.getElementById('armedCount').textContent = armed;
                document.getElementById('militaryCount').textContent = data.military || military;
                document.getElementById('personCount').textContent = data.persons || persons;
                document.getElementById('totalSeen').textContent = data.total_weapons_seen;
                document.getElementById('fpsCount').textContent = data.fps;
                const hdrFps = document.getElementById('hdrFps');
                if (hdrFps) hdrFps.textContent = data.fps;

                const eventList = document.getElementById('eventList');
                if (data.events.length === 0) {
                    eventList.innerHTML = '<li class="no-detections">No events yet</li>';
                } else {
                    eventList.innerHTML = data.events.slice().reverse().map(e =>
                        `<li class="event-item"><span class="event-msg">${e.msg}</span><span class="event-time">${e.time}</span></li>`
                    ).join('');
                }
            });
    }
    setInterval(updateDetections, 500);

    window.addEventListener('load', () => ensureMap());
</script>
</body>
</html>
"""
def boxes_overlap(b1, b2):
    px1, py1, px2, py2 = b1
    wx1, wy1, wx2, wy2 = b2
    cx = (wx1 + wx2) / 2
    cy = (wy1 + wy2) / 2
    return px1 <= cx <= px2 and py1 <= cy <= py2


def _inference_loop():
    """
    Single background thread — camera capture + YOLO inference + annotation.
    Stores the latest encoded JPEG in _latest_jpeg so ALL HTTP clients share
    one inference pass (no duplicate work per viewer tab).
    """
    global _latest_jpeg

    # ── Camera init: try PiCamera2 first, fall back to OpenCV ────────────────
    use_picam = False
    cap = None
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "BGR888"})
        picam2.configure(cfg)
        picam2.start()
        use_picam = True
        print("  [camera] PiCamera2 active (640×480)", flush=True)
    except Exception:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS,          30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # ← critical on RPi: prevents stale frames
        print("  [camera] OpenCV VideoCapture (640×480)", flush=True)

    fps_time    = time.time()
    frame_count = 0
    current_fps = 0
    prev_alert  = False
    skip_ctr    = 0

    last_weapon_boxes: list = []   # reused on skipped frames
    last_person_boxes: list = []

    while True:
        # ── Capture ──────────────────────────────────────────────────────────
        if use_picam:
            frame = picam2.capture_array()
        else:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue

        frame = cv2.flip(frame, 1)
        skip_ctr += 1
        run_infer = (skip_ctr % _INFER_EVERY == 0)

        with state_lock:
            conf = state["conf_threshold"]

        # ── Inference (every _INFER_EVERY frames) ────────────────────────────
        if run_infer:
            fast_r = person_model.predict(
                frame, conf=0.4, classes=[0], imgsz=_INFER_SIZE, verbose=False)[0]
            has_person = len(fast_r.boxes) > 0

            weapon_boxes: list = []
            person_boxes: list = []
            if has_person:
                results = model.predict(
                    frame, conf=conf, imgsz=_INFER_SIZE, verbose=False)
                r = results[0]
                for box in r.boxes:
                    cls_name   = r.names[int(box.cls)]
                    confidence = float(box.conf)
                    xyxy       = box.xyxy[0].cpu().numpy().tolist()
                    if cls_name in WEAPON_CLASSES:
                        weapon_boxes.append((xyxy, cls_name, confidence))
                    elif cls_name == "person":
                        person_boxes.append(xyxy)
            last_weapon_boxes = weapon_boxes
            last_person_boxes = person_boxes
        else:
            weapon_boxes = last_weapon_boxes
            person_boxes = last_person_boxes

        # ── Annotation ───────────────────────────────────────────────────────
        annotated = frame.copy()
        dets: list = []

        armed_idx    = set()
        military_idx = set()
        unarmed_idx  = set()
        for i, pbox in enumerate(person_boxes):
            has_mil = has_wpn = False
            for wbox, wname, _ in weapon_boxes:
                if boxes_overlap(pbox, wbox):
                    has_wpn = True
                    if wname in MILITARY_WEAPON_CLASSES:
                        has_mil = True
            if has_mil:   military_idx.add(i)
            elif has_wpn: armed_idx.add(i)
            else:         unarmed_idx.add(i)

        F, S = cv2.FONT_HERSHEY_SIMPLEX, 0.55  # font + scale (smaller = faster)
        for i in military_idx:
            x1,y1,x2,y2 = map(int, person_boxes[i])
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,180,255),2)
            (tw,th),_ = cv2.getTextSize("MILITARY",F,S,1)
            cv2.rectangle(annotated,(x1,y1-th-6),(x1+tw+6,y1),(0,180,255),-1)
            cv2.putText(annotated,"MILITARY",(x1+3,y1-3),F,S,(0,0,0),1)
            dets.append({"class":"military personnel","confidence":1.0})

        for i in armed_idx:
            x1,y1,x2,y2 = map(int, person_boxes[i])
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,0,255),2)
            (tw,th),_ = cv2.getTextSize("ARMED",F,S,1)
            cv2.rectangle(annotated,(x1,y1-th-6),(x1+tw+6,y1),(0,0,255),-1)
            cv2.putText(annotated,"ARMED",(x1+3,y1-3),F,S,(255,255,255),1)
            dets.append({"class":"armed person","confidence":1.0})

        for i in unarmed_idx:
            x1,y1,x2,y2 = map(int, person_boxes[i])
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(180,100,40),2)
            (tw,th),_ = cv2.getTextSize("PERSON",F,S,1)
            cv2.rectangle(annotated,(x1,y1-th-6),(x1+tw+6,y1),(180,100,40),-1)
            cv2.putText(annotated,"PERSON",(x1+3,y1-3),F,S,(255,255,255),1)
            dets.append({"class":"person","confidence":1.0})

        for xyxy, cls_name, confidence in weapon_boxes:
            x1,y1,x2,y2 = map(int, xyxy)
            cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,165,255),2)
            lbl = f"{cls_name} {int(confidence*100)}%"
            (tw,th),_ = cv2.getTextSize(lbl,F,S,1)
            cv2.rectangle(annotated,(x1,y1-th-6),(x1+tw+6,y1),(0,165,255),-1)
            cv2.putText(annotated,lbl,(x1+3,y1-3),F,S,(0,0,0),1)
            dets.append({"class":cls_name,"confidence":round(confidence,2)})

        # ── FPS counter ───────────────────────────────────────────────────────
        frame_count += 1
        elapsed = time.time() - fps_time
        if elapsed >= 1.0:
            current_fps = round(frame_count / elapsed)
            frame_count = 0
            fps_time    = time.time()

        # ── State + alert logic ───────────────────────────────────────────────
        weapons_now  = [d for d in dets if d["class"] in WEAPON_CLASSES]
        armed_now    = [d for d in dets if d["class"] == "armed person"]
        military_now = [d for d in dets if d["class"] == "military personnel"]
        persons_now  = [d for d in dets if d["class"] == "person"]
        is_alert     = bool(weapons_now or armed_now or military_now)

        with state_lock:
            state["detections"] = dets
            state["fps"]        = current_fps
            state["alert"]      = is_alert
            state["persons"]    = len(persons_now)
            state["military"]   = len(military_now)

            if is_alert and not prev_alert:
                state["total_weapons_seen"] += 1
                ts = datetime.now()
                if military_now:
                    msg = f"MILITARY PERSONNEL ({len(military_now)})"
                elif armed_now:
                    msg = f"ARMED PERSON ({len(armed_now)})"
                else:
                    top = max(weapons_now, key=lambda d: d["confidence"])
                    msg = f"{top['class']} ({int(top['confidence']*100)}%)"
                gps_snap = dict(state["gps"])
                state["events"].append({
                    "msg": msg, "time": ts.strftime("%H:%M:%S"),
                    "lat": gps_snap["lat"], "lon": gps_snap["lon"],
                })
                if gps_snap["lat"] is not None:
                    state["alert_markers"].append({
                        "lat": gps_snap["lat"], "lon": gps_snap["lon"],
                        "msg": msg, "time": ts.strftime("%H:%M:%S"),
                    })
                    if len(state["alert_markers"]) > 50:
                        state["alert_markers"].pop(0)
                if time.time() - state["last_snapshot_time"] > 5:
                    fname = f"alert_{ts.strftime('%Y%m%d_%H%M%S')}.jpg"
                    cv2.imwrite(os.path.join(SNAPSHOT_DIR, fname), annotated)
                    state["last_snapshot_time"] = time.time()

        prev_alert = is_alert

        # ── View mode transform (only when non-normal) ────────────────────────
        with state_lock:
            view = state["view_mode"]
        if view != "normal":
            gray = cv2.cvtColor(annotated, cv2.COLOR_BGR2GRAY)
            if view == "night":
                z = np.zeros_like(gray)
                annotated = cv2.merge([z, gray, z])
            elif view == "thermal":
                annotated = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)
            elif view == "gray":
                annotated = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # ── Encode + store in shared buffer ───────────────────────────────────
        _, buf = cv2.imencode('.jpg', annotated,
                              [cv2.IMWRITE_JPEG_QUALITY, _JPEG_QUALITY])
        with _frame_lock:
            _latest_jpeg = buf.tobytes()

    if cap is not None:
        cap.release()


def generate_frames():
    """
    Lightweight MJPEG generator — reads the shared buffer, zero inference here.
    All viewers get the same pre-encoded frame; adding more tabs costs nothing.
    """
    while True:
        with _frame_lock:
            jpeg = _latest_jpeg
        if jpeg:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
        time.sleep(0.030)   # ~33 FPS ceiling for the stream


@app.route('/')
def index():
    return render_template_string(HTML_PAGE)


@app.route('/video_feed')
def video_feed():
    resp = Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
    resp.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate, private'
    resp.headers['Pragma'] = 'no-cache'
    resp.headers['Expires'] = '0'
    return resp


@app.route('/detections')
def detections():
    with state_lock:
        return jsonify({
            "detections": state["detections"],
            "fps": state["fps"],
            "alert": state["alert"],
            "total_weapons_seen": state["total_weapons_seen"],
            "events": list(state["events"]),
            "persons": state["persons"],
            "military": state["military"],
        })


@app.route('/set_conf')
def set_conf():
    try:
        value = float(request.args.get("value", 0.35))
        value = max(0.05, min(0.95, value))
        with state_lock:
            state["conf_threshold"] = value
        return jsonify({"ok": True, "conf": value})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400


@app.route('/location', methods=['POST'])
def location():
    data = request.get_json(silent=True) or {}
    try:
        lat = float(data["lat"])
        lon = float(data["lon"])
        acc = float(data.get("accuracy", 0))
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False}), 400
    with state_lock:
        state["gps"]["lat"] = lat
        state["gps"]["lon"] = lon
        state["gps"]["accuracy"] = acc
        state["gps"]["ts"] = datetime.now().strftime("%H:%M:%S")
    return jsonify({"ok": True})


@app.route('/gps_markers')
def gps_markers():
    with state_lock:
        return jsonify({
            "gps": state["gps"],
            "markers": list(state["alert_markers"]),
        })


@app.route('/unlock', methods=['POST'])
def unlock():
    data = request.get_json(silent=True) or {}
    pw = str(data.get("password", ""))
    if pw == CONTROL_PASSWORD:
        return jsonify({"ok": True})
    return jsonify({"ok": False}), 401


@app.route('/serial_status')
def serial_status():
    connected = arduino is not None
    port = ""
    if connected:
        try:
            port = arduino.port
        except Exception:
            pass
    return jsonify({"connected": connected, "port": port})


@app.route('/serial_reconnect')
def serial_reconnect():
    global arduino
    if arduino is not None:
        try:
            arduino.close()
        except Exception:
            pass
        arduino = None
    init_serial()
    connected = arduino is not None
    port = arduino.port if connected else ""
    return jsonify({"connected": connected, "port": port})


VEHICLE_CHARS = set("FBLRWGHIJ")  # forward/back/left/right/stop + diagonals


@app.route('/cmd')
def cmd():
    """
    Forward a control command to the Arduino.
    Vehicle:  single char  F=fwd  B=back  L=left  R=right  W=stop
                            G=back-left  H=fwd-left  I=back-right  J=fwd-right
    Arm:      ARM:<base>,<elbow>,<gripper>   each 0-180
    Speed:    SPD:<0-255>                    drive motor PWM
    """
    c = request.args.get("c", "").strip()
    if not c or len(c) > 32:
        return jsonify({"ok": False, "error": "bad cmd"}), 400

    # Whitelist
    if len(c) == 1 and c in VEHICLE_CHARS:
        pass
    elif c.startswith("ARM:"):
        try:
            parts = c[4:].split(",")
            if len(parts) != 3:
                raise ValueError
            for p in parts:
                v = int(p)
                if not 0 <= v <= 180:
                    raise ValueError
        except ValueError:
            return jsonify({"ok": False, "error": "bad arm cmd"}), 400
    elif c.startswith("SPD:"):
        try:
            v = int(c[4:])
            if not 0 <= v <= 255:
                raise ValueError
            with state_lock:
                state["vehicle_speed"] = v
        except ValueError:
            return jsonify({"ok": False, "error": "bad speed"}), 400
    else:
        return jsonify({"ok": False, "error": "rejected"}), 400

    sent = send_serial(c)
    if not sent:
        print(f"  [cmd] NOT sent (arduino disconnected): {c!r}")
    else:
        print(f"  [cmd] → {c!r}")
    return jsonify({"ok": True, "sent": sent, "cmd": c})


@app.route('/set_view')
def set_view():
    mode = request.args.get("mode", "normal").lower()
    if mode not in VIEW_MODES:
        return jsonify({"ok": False, "error": "bad mode"}), 400
    with state_lock:
        state["view_mode"] = mode
    return jsonify({"ok": True, "mode": mode})


@app.route('/snapshots/<path:filename>')
def get_snapshot(filename):
    return send_from_directory(SNAPSHOT_DIR, filename)


@app.route('/gallery')
def gallery():
    files = []
    try:
        for fname in sorted(os.listdir(SNAPSHOT_DIR), reverse=True):
            if fname.lower().endswith(('.jpg', '.jpeg', '.png')):
                files.append(fname)
    except FileNotFoundError:
        pass

    items = "".join(
        f'<a class="snap" href="/snapshots/{f}" target="_blank">'
        f'<img src="/snapshots/{f}" loading="lazy"><span>{f}</span></a>'
        for f in files
    ) or '<p class="empty">// NO SNAPSHOTS RECORDED //</p>'

    page = """<!DOCTYPE html><html><head><meta charset="UTF-8">
<title>SNAPSHOT GALLERY // TWDS</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#060a05; color:#c8d6b9; font-family:'Consolas',monospace; padding:24px; }
h1 { color:#9bcc4a; font-size:16px; letter-spacing:3px; text-transform:uppercase;
     border-bottom:1px solid #2a3a1a; padding-bottom:10px; margin-bottom:18px; }
h1 a { color:#d4a017; text-decoration:none; font-size:11px; float:right; letter-spacing:2px; }
.grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(220px,1fr)); gap:14px; }
.snap { background:#11180e; border:1px solid #2a3a1a; padding:6px; text-decoration:none;
        color:#9bcc4a; display:block; position:relative; }
.snap::before { content:''; position:absolute; top:-1px; left:-1px; width:10px; height:10px;
                border-top:2px solid #9bcc4a; border-left:2px solid #9bcc4a; }
.snap::after  { content:''; position:absolute; bottom:-1px; right:-1px; width:10px; height:10px;
                border-bottom:2px solid #9bcc4a; border-right:2px solid #9bcc4a; }
.snap:hover { background:#182112; box-shadow:0 0 10px rgba(155,204,74,0.3); }
.snap img { width:100%; display:block; }
.snap span { display:block; padding:6px 4px; font-size:10px; letter-spacing:1px; }
.empty { color:#4a5d3a; text-align:center; padding:60px; letter-spacing:2px; }
</style></head><body>
<h1>SNAPSHOT GALLERY <a href="/">&#x2190; BACK TO OPS</a></h1>
<div class="grid">__ITEMS__</div></body></html>"""
    return page.replace("__ITEMS__", items)


@app.route('/events.csv')
def events_csv():
    with state_lock:
        events = list(state["events"])
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(["time", "message"])
    for e in events:
        w.writerow([e.get("time", ""), e.get("msg", "")])
    return Response(
        buf.getvalue(),
        mimetype="text/csv",
        headers={"Content-Disposition": "attachment; filename=events.csv"},
    )


if __name__ == '__main__':
    print("\n  Weapon Detection + Control System")
    print(f"  Model classes: {list(CLASS_NAMES.values())}")
    init_serial()
    # Start the single shared inference loop — all browser clients share one frame buffer
    _t = threading.Thread(target=_inference_loop, daemon=True)
    _t.start()
    print("  Open http://localhost:5000 in your browser\n")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
