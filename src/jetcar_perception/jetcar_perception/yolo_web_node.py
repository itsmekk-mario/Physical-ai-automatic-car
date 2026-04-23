import threading
import time
from collections import deque
from pathlib import Path
import re

import cv2
import gi
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from flask import Flask, Response, jsonify, render_template_string, request
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)


class GStreamerCapture:
    def __init__(self, pipeline: str, sink_name: str = 'capture_sink'):
        self.pipeline_text = pipeline
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name(sink_name)
        if self.appsink is None:
            raise RuntimeError(f'appsink "{sink_name}" not found in pipeline')
        self.appsink.set_property('emit-signals', False)
        self.appsink.set_property('sync', False)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self):
        if not self.opened:
            return False, None
        sample = self.appsink.emit('try-pull-sample', 1_000_000_000)
        if sample is None:
            return False, None

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = int(structure.get_value('width'))
        height = int(structure.get_value('height'))
        fmt = str(structure.get_value('format'))

        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return False, None

        try:
            channels = 4 if fmt == 'BGRx' else 3
            frame = np.ndarray(
                (height, width, channels),
                dtype=np.uint8,
                buffer=map_info.data,
            ).copy()
        finally:
            buf.unmap(map_info)

        if fmt == 'BGRx':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return True, frame

    def release(self):
        if not self.opened:
            return
        self.opened = False
        self.pipeline.set_state(Gst.State.NULL)


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>JetCar Dashboard</title>
    <style>
        :root {
            --bg: #101214;
            --panel: #171a1d;
            --panel-2: #1e2226;
            --border: #2c3238;
            --text: #f2f4f5;
            --muted: #b1b8bf;
            --accent: #4da3ff;
            --danger: #d9534f;
            --ok: #42a66a;
        }
        * {
            box-sizing: border-box;
        }
        body {
            margin: 0;
            padding: 16px;
            font-family: "Segoe UI", "Noto Sans KR", sans-serif;
            background: var(--bg);
            color: var(--text);
        }
        .shell {
            max-width: 1320px;
            margin: 0 auto;
        }
        .topbar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 12px;
            margin-bottom: 14px;
        }
        .title h1 {
            margin: 0;
            font-size: 1.8rem;
        }
        .title p {
            margin: 6px 0 0;
            color: var(--muted);
            font-size: 0.95rem;
        }
        .status-pill {
            padding: 10px 14px;
            border: 1px solid var(--border);
            border-radius: 999px;
            background: var(--panel);
            white-space: nowrap;
        }
        .layout {
            display: grid;
            grid-template-columns: minmax(0, 1.2fr) minmax(360px, 0.8fr);
            gap: 16px;
        }
        .panel, .card {
            background: var(--panel);
            border: 1px solid var(--border);
            border-radius: 8px;
        }
        .panel {
            padding: 14px;
        }
        .card {
            padding: 12px;
            margin-bottom: 12px;
            background: var(--panel-2);
        }
        .stream {
            width: 100%;
            background: #000;
            border-radius: 8px;
            min-height: 300px;
            object-fit: contain;
        }
        .stats, .mode-row, .btn-row, .pad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            margin: 8px 0;
        }
        .pad {
            grid-template-columns: repeat(3, minmax(0, 1fr));
        }
        button, input[type="range"] {
            width: 100%;
        }
        button {
            min-height: 48px;
            border: 1px solid var(--border);
            border-radius: 8px;
            background: #252a2f;
            color: var(--text);
            font-weight: 700;
            cursor: pointer;
        }
        button:hover {
            background: #2b3137;
        }
        button.active {
            outline: 2px solid var(--accent);
            outline-offset: 0;
            background: #313944;
        }
        button.primary {
            border-color: #3b6ea8;
        }
        button.danger {
            border-color: #7d3b38;
            background: #3a2423;
        }
        button.ok {
            border-color: #3e6f4d;
            background: #233126;
        }
        button.stop {
            border-color: #5c6670;
            background: #2c3238;
        }
        pre {
            white-space: pre-wrap;
            word-break: break-word;
            margin: 0;
            color: var(--muted);
            line-height: 1.45;
        }
        .meta {
            color: var(--muted);
            font-size: 0.92rem;
            margin-top: 8px;
        }
        .stat {
            background: #20252a;
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 10px;
        }
        .stat label {
            display: block;
            font-size: 0.82rem;
            color: var(--muted);
            margin-bottom: 6px;
        }
        .stat strong {
            font-size: 1.2rem;
        }
        .section-title {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .section-title h2 {
            margin: 0;
            font-size: 1rem;
        }
        .section-title span {
            color: var(--muted);
            font-size: 0.9rem;
        }
        .badge-row {
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
            margin: 8px 0 0;
        }
        .badge {
            padding: 6px 10px;
            border-radius: 999px;
            border: 1px solid var(--border);
            background: #20252a;
            color: var(--muted);
            font-size: 0.86rem;
        }
        .badge.mode {
            color: var(--text);
            border-color: #3b6ea8;
        }
        .badge.ai {
            border-color: #5f6b7a;
        }
        .control-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 12px;
            margin-top: 8px;
        }
        .axis-box {
            padding: 12px;
            border-radius: 10px;
            border: 1px solid var(--border);
            background: #20252a;
        }
        .axis-box h3 {
            margin: 0 0 4px;
            font-size: 0.96rem;
        }
        .axis-box p {
            margin: 0 0 10px;
            color: var(--muted);
            font-size: 0.86rem;
        }
        .axis-stack {
            display: grid;
            gap: 8px;
        }
        .axis-stack button {
            min-height: 56px;
        }
        .keycap {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            min-width: 28px;
            height: 28px;
            padding: 0 8px;
            margin-right: 8px;
            border-radius: 999px;
            border: 1px solid #54606b;
            background: #15191d;
            color: #dbe4ea;
            font-size: 0.82rem;
            font-weight: 800;
        }
        .button-copy {
            display: inline-flex;
            align-items: center;
            justify-content: center;
        }
        .emergency-card {
            border: 1px solid #6f2f34;
            background:
                radial-gradient(circle at top, rgba(217, 83, 79, 0.18), transparent 45%),
                linear-gradient(180deg, #2a1719 0%, #201315 100%);
        }
        .emergency-actions {
            display: grid;
            grid-template-columns: 1.4fr 1fr 1fr;
            gap: 10px;
            margin-top: 10px;
        }
        .emergency-button {
            min-height: 84px;
            border-radius: 14px;
            border: 1px solid #a94b4e;
            background: linear-gradient(180deg, #dc5a56 0%, #8d2727 100%);
            color: #fff7f7;
            letter-spacing: 0.04em;
            box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.12);
        }
        .emergency-button:hover {
            background: linear-gradient(180deg, #e36763 0%, #9e2d2d 100%);
        }
        .emergency-button strong {
            display: block;
            font-size: 1.08rem;
        }
        .emergency-button span {
            display: block;
            margin-top: 4px;
            font-size: 0.82rem;
            color: #ffe5e4;
        }
        .release-button,
        .secondary-button {
            min-height: 84px;
            border-radius: 14px;
        }
        a {
            color: var(--accent);
        }
        @media (max-width: 980px) {
            .layout {
                grid-template-columns: 1fr;
            }
        }
        @media (max-width: 720px) {
            .stats, .mode-row, .btn-row {
                grid-template-columns: repeat(2, 1fr);
            }
            .control-grid,
            .emergency-actions {
                grid-template-columns: 1fr;
            }
            .topbar {
                flex-direction: column;
                align-items: flex-start;
            }
        }
    </style>
</head>
<body>
    <div class="shell">
        <div class="topbar">
            <div class="title">
                <h1>JetCar Dashboard</h1>
                <p>YOLO 영상, 수동 제어, AI 개입 모드를 한 화면에서 확인합니다.</p>
            </div>
            <div class="status-pill" id="connectState">Connecting...</div>
        </div>

        <div class="layout">
            <div class="panel">
                <div class="section-title">
                    <h2>Camera</h2>
                    <span><a href="/stream" target="_blank" rel="noopener">Stream only</a></span>
                </div>
                <img class="stream" id="streamImage" alt="stream">
                <div class="stats">
                    <div class="stat"><label>Detections</label><strong id="detCount">0</strong></div>
                    <div class="stat"><label>FPS</label><strong id="fpsValue">0.0</strong></div>
                    <div class="stat"><label>Det FPS</label><strong id="detFpsValue">0.0</strong></div>
                </div>
                <pre id="visionText">Loading...</pre>
            </div>

            <div>
                <div class="card">
                    <div class="section-title">
                        <h2>Drive Mode</h2>
                        <span id="modeValue">MANUAL</span>
                    </div>
                    <div class="badge-row">
                        <div class="badge mode" id="selectedSourceBadge">selected: MANUAL</div>
                        <div class="badge ai" id="safetyBadge">AI override: false</div>
                    </div>
                    <div class="mode-row">
                        <button class="primary" data-mode="MANUAL">MANUAL</button>
                        <button class="primary" data-mode="AI_INTERVENTION">AI INTERVENTION</button>
                        <button class="primary" data-mode="AUTONOMOUS">AUTONOMOUS</button>
                    </div>
                </div>

                <div class="card">
                    <div class="section-title">
                        <h2>Throttle</h2>
                        <span id="throttleValue">0.00</span>
                    </div>
                    <input id="throttleSlider" type="range" min="-100" max="100" value="0" step="1">
                    <div class="btn-row">
                        <button data-throttle="-40">REV 40%</button>
                        <button data-throttle="-20">REV 20%</button>
                        <button data-action="stop">STOP</button>
                        <button data-throttle="20">FWD 20%</button>
                        <button data-throttle="40">FWD 40%</button>
                    </div>
                </div>

                <div class="card">
                    <div class="section-title">
                        <h2>Steering</h2>
                        <span id="steeringValue">0.00</span>
                    </div>
                    <input id="steeringSlider" type="range" min="-100" max="100" value="0" step="1">
                    <div class="btn-row">
                        <button data-steering="-100">HARD L</button>
                        <button data-steering="-40">LEFT</button>
                        <button data-action="center">CENTER</button>
                        <button data-steering="40">RIGHT</button>
                        <button data-steering="100">HARD R</button>
                    </div>
                </div>

                <div class="card">
                    <div class="section-title">
                        <h2>Manual Control</h2>
                        <span>Hold to repeat</span>
                    </div>
                    <div class="control-grid">
                        <div class="axis-box">
                            <h3>Throttle Axis</h3>
                            <p><span class="keycap">W</span>forward / <span class="keycap">S</span>reverse</p>
                            <div class="axis-stack">
                                <button data-key="w"><span class="button-copy"><span class="keycap">W</span>Forward</span></button>
                                <button class="stop" data-key="x"><span class="button-copy"><span class="keycap">X</span>Brake</span></button>
                                <button data-key="s"><span class="button-copy"><span class="keycap">S</span>Reverse</span></button>
                            </div>
                        </div>
                        <div class="axis-box">
                            <h3>Steering Axis</h3>
                            <p><span class="keycap">A</span>left / <span class="keycap">D</span>right</p>
                            <div class="axis-stack">
                                <button data-key="a"><span class="button-copy"><span class="keycap">A</span>Left</span></button>
                                <button data-key="c"><span class="button-copy"><span class="keycap">C</span>Center</span></button>
                                <button data-key="d"><span class="button-copy"><span class="keycap">D</span>Right</span></button>
                            </div>
                        </div>
                    </div>
                    <div class="meta">Keyboard: `W/S`는 전진/후진, `A/D`는 좌/우 조향, `X`는 브레이크, `C`는 조향 중앙 복귀입니다.</div>
                </div>

                <div class="card emergency-card">
                    <div class="section-title">
                        <h2>Emergency</h2>
                        <span>Immediate stop path</span>
                    </div>
                    <div class="emergency-actions">
                        <button class="emergency-button" data-action="estop">
                            <strong>EMERGENCY STOP</strong>
                            <span>Motor stop + steering neutral</span>
                        </button>
                        <button class="ok release-button" data-action="release">RELEASE</button>
                        <button class="secondary-button" data-action="reset-all">RESET</button>
                    </div>
                    <pre id="controlText">Loading...</pre>
                </div>
            </div>
        </div>
        <div class="meta">다른 기기에서 접속할 때는 `127.0.0.1`이 아니라 Jetson IP를 사용해야 합니다.</div>
    </div>

    <script>
        document.getElementById('streamImage').src = '/stream?ts=' + Date.now();

        const throttleSlider = document.getElementById('throttleSlider');
        const steeringSlider = document.getElementById('steeringSlider');

        async function fetchJson(url, options) {
            const res = await fetch(url, options);
            return res.json();
        }

        function updateVision(data) {
            document.getElementById('detCount').textContent = data.detection_count;
            document.getElementById('fpsValue').textContent = Number(data.fps).toFixed(1);
            document.getElementById('detFpsValue').textContent = Number(data.detection_fps).toFixed(1);
            document.getElementById('visionText').textContent =
                `ready=${data.ready}\n` +
                `camera=${data.camera_source}\n` +
                `model=${data.model_path}\n` +
                `hazard=${data.hazard}\n` +
                `status=${data.status}`;
        }

        function updateControl(data) {
            document.getElementById('modeValue').textContent = data.drive_mode;
            document.getElementById('throttleValue').textContent = Number(data.throttle).toFixed(2);
            document.getElementById('steeringValue').textContent = Number(data.steering).toFixed(2);
            document.getElementById('selectedSourceBadge').textContent = `selected: ${data.selected_source}`;
            document.getElementById('safetyBadge').textContent = `AI override: ${data.safety_override}`;
            throttleSlider.value = Math.round(Number(data.throttle) * 100);
            steeringSlider.value = Math.round(Number(data.steering) * 100);
            document.getElementById('controlText').textContent =
                `mode=${data.drive_mode}\n` +
                `selected=${data.selected_source}\n` +
                `estop=${data.estop}\n` +
                `safety_override=${data.safety_override}\n` +
                `reason=${data.safety_reason}\n` +
                `distance=${Number(data.min_distance_m).toFixed(2)}\n` +
                `status=${data.status}`;
        }

        async function refreshAll() {
            try {
                const [vision, control] = await Promise.all([
                    fetchJson('/api/status'),
                    fetchJson('/api/control_state')
                ]);
                updateVision(vision);
                updateControl(control);
                document.getElementById('connectState').textContent = 'Connected';
            } catch (e) {
                document.getElementById('connectState').textContent = 'Disconnected';
            }
        }

        async function postControl(url, payload) {
            const data = await fetchJson(url, {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(payload || {})
            });
            updateControl(data);
        }

        throttleSlider.addEventListener('change', function() {
            postControl('/api/set_control_state', {throttle: Number(throttleSlider.value) / 100.0});
        });
        steeringSlider.addEventListener('change', function() {
            postControl('/api/set_control_state', {steering: Number(steeringSlider.value) / 100.0});
        });
        document.querySelectorAll('[data-throttle]').forEach(function(button) {
            button.addEventListener('click', function() {
                postControl('/api/set_control_state', {throttle: Number(button.dataset.throttle) / 100.0});
            });
        });
        document.querySelectorAll('[data-steering]').forEach(function(button) {
            button.addEventListener('click', function() {
                postControl('/api/set_control_state', {steering: Number(button.dataset.steering) / 100.0});
            });
        });
        document.querySelectorAll('[data-mode]').forEach(function(button) {
            button.addEventListener('click', function() {
                postControl('/api/set_drive_mode', {mode: button.dataset.mode});
            });
        });
        document.querySelectorAll('[data-action]').forEach(function(button) {
            button.addEventListener('click', function() {
                const action = button.dataset.action;
                if (action === 'stop') postControl('/api/set_control_state', {throttle: 0.0});
                else if (action === 'center') postControl('/api/set_control_state', {steering: 0.0});
                else if (action === 'reset-all') postControl('/api/set_control_state', {throttle: 0.0, steering: 0.0});
                else if (action === 'estop') postControl('/api/control_command', {key: 'e'});
                else if (action === 'release') postControl('/api/control_command', {key: 'r'});
            });
        });
        const repeatKeys = new Set(['w', 'a', 's', 'd']);
        const singleKeys = new Set(['x', 'c', 'e', 'r']);
        const activeKeys = new Set();

        function markKey(key, active) {
            document.querySelectorAll(`[data-key="${key}"]`).forEach(function(button) {
                button.classList.toggle('active', active);
            });
        }

        function sendKey(key) {
            postControl('/api/control_command', {key: key});
        }

        function pressKey(key) {
            if (repeatKeys.has(key)) {
                if (activeKeys.has(key)) {
                    return;
                }
                activeKeys.add(key);
            }
            markKey(key, true);
            sendKey(key);
        }

        function releaseKey(key) {
            activeKeys.delete(key);
            markKey(key, false);
            if (key === 'w' || key === 's') {
                sendKey('x');
            } else if (key === 'a' || key === 'd') {
                sendKey('c');
            }
        }

        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            if (!(repeatKeys.has(key) || singleKeys.has(key))) {
                return;
            }
            event.preventDefault();
            pressKey(key);
        });

        document.addEventListener('keyup', function(event) {
            const key = event.key.toLowerCase();
            if (!(repeatKeys.has(key) || singleKeys.has(key))) {
                return;
            }
            event.preventDefault();
            releaseKey(key);
        });

        window.addEventListener('blur', function() {
            Array.from(activeKeys).forEach(function(key) {
                releaseKey(key);
            });
        });

        setInterval(function() {
            activeKeys.forEach(function(key) {
                sendKey(key);
            });
        }, 90);

        document.querySelectorAll('[data-key]').forEach(function(button) {
            const key = button.dataset.key;
            const start = function(event) {
                event.preventDefault();
                pressKey(key);
            };
            const end = function(event) {
                if (event) {
                    event.preventDefault();
                }
                releaseKey(key);
            };
            button.addEventListener('pointerdown', start);
            button.addEventListener('pointerup', end);
            button.addEventListener('pointercancel', end);
            button.addEventListener('pointerleave', end);
            button.addEventListener('click', function(event) {
                event.preventDefault();
                if (repeatKeys.has(key)) {
                    return;
                }
                pressKey(key);
                releaseKey(key);
            });
        });

        setInterval(refreshAll, 500);
        refreshAll();
    </script>
</body>
</html>
"""
class YoloWebNode(Node):
    def __init__(self):
        super().__init__('yolo_web_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('camera_source', 'auto')
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('camera_sensor_id', 0)
        self.declare_parameter('camera_flip_method', 0)
        self.declare_parameter('usb_fourcc', 'default')
        self.declare_parameter('model_path', 'auto')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('left_image_topic', '')
        self.declare_parameter('right_image_topic', '')
        self.declare_parameter('target_classes', ['person', 'car', 'stop sign'])
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 60)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('stream_height', 360)
        self.declare_parameter('stream_delay_ms', 5)
        self.declare_parameter('camera_retry_sec', 1.5)
        self.declare_parameter('camera_read_failures_before_reopen', 5)
        self.declare_parameter('detection_rate_hz', 4.0)
        self.declare_parameter('inference_device', 'cuda:0')
        self.declare_parameter('prefer_half', True)
        self.declare_parameter('throttle_step', 0.1)
        self.declare_parameter('steering_step_cmd', 0.2)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.camera_source = str(self.get_parameter('camera_source').value)
        self.camera_backend = str(self.get_parameter('camera_backend').value).lower().strip()
        self.camera_sensor_id = int(self.get_parameter('camera_sensor_id').value)
        self.camera_flip_method = int(self.get_parameter('camera_flip_method').value)
        self.usb_fourcc = str(self.get_parameter('usb_fourcc').value).upper().strip()
        self.model_path = str(self.get_parameter('model_path').value)
        self.image_topic = str(self.get_parameter('image_topic').value).strip()
        self.left_image_topic = str(self.get_parameter('left_image_topic').value).strip()
        self.right_image_topic = str(self.get_parameter('right_image_topic').value).strip()
        self.target_classes = [str(name) for name in self.get_parameter('target_classes').value]
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)
        self.stream_width = int(self.get_parameter('stream_width').value)
        self.stream_height = int(self.get_parameter('stream_height').value)
        self.stream_delay = max(0.0, int(self.get_parameter('stream_delay_ms').value) / 1000.0)
        self.camera_retry_sec = max(0.2, float(self.get_parameter('camera_retry_sec').value))
        self.camera_read_failures_before_reopen = max(
            1, int(self.get_parameter('camera_read_failures_before_reopen').value)
        )
        self.detection_rate_hz = float(self.get_parameter('detection_rate_hz').value)
        self.inference_device = str(self.get_parameter('inference_device').value).strip()
        self.prefer_half = bool(self.get_parameter('prefer_half').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.steering_step_cmd = float(self.get_parameter('steering_step_cmd').value)

        self.ready_pub = self.create_publisher(Bool, '/perception/detections/ready', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/detections/hazard', 10)
        self.closest_pub = self.create_publisher(Float32, '/perception/detections/closest_confidence', 10)
        self.status_pub = self.create_publisher(String, '/perception/detections/status', 10)
        self.manual_throttle_pub = self.create_publisher(Float32, '/input/manual/throttle', 10)
        self.manual_steering_pub = self.create_publisher(Float32, '/input/manual/steering', 10)
        self.estop_pub = self.create_publisher(Bool, '/system/estop_cmd', 10)
        self.drive_mode_pub = self.create_publisher(String, '/system/drive_mode_cmd', 10)
        self.create_subscription(Bool, '/vehicle/emergency_stop_state', self.estop_state_cb, 10)
        self.create_subscription(String, '/system/drive_mode', self.drive_mode_cb, 10)
        self.create_subscription(String, '/system/selected_control_source', self.selected_source_cb, 10)
        self.create_subscription(Bool, '/system/safety_override_active', self.safety_override_cb, 10)
        self.create_subscription(String, '/system/safety_override_reason', self.safety_reason_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Bool, '/perception/depth/ready', self.depth_ready_cb, 10)
        if self.image_topic:
            self.create_subscription(Image, self.image_topic, self.image_topic_cb, 10)
        if self.left_image_topic:
            self.create_subscription(Image, self.left_image_topic, self.left_image_topic_cb, 10)
        if self.right_image_topic:
            self.create_subscription(Image, self.right_image_topic, self.right_image_topic_cb, 10)

        self.frame_lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_frame_id = 0
        self.latest_status = 'starting'
        self.latest_detection_count = 0
        self.latest_confidence = 0.0
        self.latest_hazard = False
        self.latest_ready = False
        self.latest_fps = 0.0
        self.latest_detection_fps = 0.0
        self.model_loaded = False
        self.model_error = ''
        self.active_model_path = ''
        self.model_runtime = 'uninitialized'
        self.model_search_paths = []
        self.active_camera_source = ''
        self.latest_detections = []
        self.pending_frame = None
        self.stop_event = threading.Event()
        self.detection_lock = threading.Lock()
        self.detector_frame = None
        self.detector_event = threading.Event()
        self.next_detection_time = 0.0
        self.control_lock = threading.Lock()
        self.control_queue = deque()
        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.estop_active = False
        self.control_status = 'ready'
        self.drive_mode = 'MANUAL'
        self.selected_source = 'UNKNOWN'
        self.safety_override_active = False
        self.safety_reason = 'clear'
        self.min_distance_m = 99.0
        self.depth_ready = False
        self.camera_error_log = ''
        self.topic_frame_lock = threading.Lock()
        self.topic_frame = None
        self.topic_frame_time = None
        self.left_topic_frame = None
        self.left_topic_frame_time = None
        self.right_topic_frame = None
        self.right_topic_frame_time = None

        self.model = self.load_model()
        self.app = Flask(__name__)
        self.setup_routes()

        self.worker_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.worker_thread.start()
        self.detector_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detector_thread.start()
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'yolo_web_node started | '
            f'url=http://{self.host}:{self.port}, camera_source={self.camera_source}, model={self.model_path}'
        )

    def estop_state_cb(self, msg: Bool):
        self.estop_active = bool(msg.data)
        if self.estop_active:
            self.current_throttle = 0.0
            self.current_steering = 0.0
        self.control_status = f'estop_state={self.estop_active}'

    def drive_mode_cb(self, msg: String):
        self.drive_mode = str(msg.data).upper().strip()

    def selected_source_cb(self, msg: String):
        self.selected_source = str(msg.data)

    def safety_override_cb(self, msg: Bool):
        self.safety_override_active = bool(msg.data)

    def safety_reason_cb(self, msg: String):
        self.safety_reason = str(msg.data)

    def min_distance_cb(self, msg: Float32):
        self.min_distance_m = float(msg.data)

    def depth_ready_cb(self, msg: Bool):
        self.depth_ready = bool(msg.data)

    def image_topic_cb(self, msg: Image):
        frame = self.image_to_bgr(msg)
        with self.topic_frame_lock:
            self.topic_frame = frame
            self.topic_frame_time = time.monotonic()
        self.active_camera_source = f'ros:{self.image_topic}'

    def left_image_topic_cb(self, msg: Image):
        frame = self.image_to_bgr(msg)
        with self.topic_frame_lock:
            self.left_topic_frame = frame
            self.left_topic_frame_time = time.monotonic()
        self.active_camera_source = f'ros:{self.left_image_topic}+{self.right_image_topic}'

    def right_image_topic_cb(self, msg: Image):
        frame = self.image_to_bgr(msg)
        with self.topic_frame_lock:
            self.right_topic_frame = frame
            self.right_topic_frame_time = time.monotonic()
        self.active_camera_source = f'ros:{self.left_image_topic}+{self.right_image_topic}'

    def compose_stereo_frame(self, left_frame, right_frame):
        if left_frame is None and right_frame is None:
            return None
        if left_frame is None:
            left_frame = np.zeros_like(right_frame)
        if right_frame is None:
            right_frame = np.zeros_like(left_frame)
        if left_frame.shape[:2] != right_frame.shape[:2]:
            right_frame = cv2.resize(right_frame, (left_frame.shape[1], left_frame.shape[0]), interpolation=cv2.INTER_AREA)
        return np.hstack((left_frame, right_frame))

    def control_state_payload(self):
        return {
            'ok': True,
            'throttle': float(self.current_throttle),
            'steering': float(self.current_steering),
            'estop': bool(self.estop_active),
            'drive_mode': self.drive_mode,
            'selected_source': self.selected_source,
            'safety_override': bool(self.safety_override_active),
            'safety_reason': self.safety_reason,
            'min_distance_m': float(self.min_distance_m),
            'depth_ready': bool(self.depth_ready),
            'status': self.control_status,
        }

    def clamp_control(self, value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def publish_control(self):
        throttle_msg = Float32()
        throttle_msg.data = float(self.current_throttle)
        self.manual_throttle_pub.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = float(self.current_steering)
        self.manual_steering_pub.publish(steering_msg)

    def publish_estop(self):
        estop_msg = Bool()
        estop_msg.data = bool(self.estop_active)
        self.estop_pub.publish(estop_msg)

    def publish_drive_mode(self, mode: str):
        msg = String()
        msg.data = str(mode).upper().strip()
        self.drive_mode_pub.publish(msg)
        self.drive_mode = msg.data
        self.control_status = f'drive_mode={msg.data}'

    def image_to_bgr(self, msg: Image):
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        channels = 1 if msg.encoding == 'mono8' else 3
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def publish_status_frame(self, text: str):
        jpeg = self.encode_frame(self.make_status_frame(text))
        with self.frame_lock:
            self.latest_jpeg = jpeg
            self.latest_frame_id += 1
            self.latest_ready = False
            self.latest_status = text

    def update_camera_status(self, status: str, log_level: str = 'info', force_log: bool = False):
        with self.frame_lock:
            self.latest_ready = False
            self.latest_status = status
        if force_log or status != self.camera_error_log:
            getattr(self.get_logger(), log_level)(status)
            self.camera_error_log = status

    def handle_control_key(self, key: str):
        if key == 'w':
            if self.estop_active:
                self.control_status = 'ignored forward command while estop is active'
                return
            self.current_throttle = self.clamp_control(self.current_throttle + self.throttle_step)
            self.control_status = f'throttle increased to {self.current_throttle:.2f}'
        elif key == 's':
            if self.estop_active:
                self.control_status = 'ignored reverse command while estop is active'
                return
            self.current_throttle = self.clamp_control(self.current_throttle - self.throttle_step)
            self.control_status = f'throttle decreased to {self.current_throttle:.2f}'
        elif key == 'x':
            self.current_throttle = 0.0
            self.control_status = 'throttle set to 0.00'
        elif key == 'a':
            if self.estop_active:
                self.control_status = 'ignored left steering while estop is active'
                return
            self.current_steering = self.clamp_control(self.current_steering - self.steering_step_cmd)
            self.control_status = f'steering moved to {self.current_steering:.2f}'
        elif key == 'd':
            if self.estop_active:
                self.control_status = 'ignored right steering while estop is active'
                return
            self.current_steering = self.clamp_control(self.current_steering + self.steering_step_cmd)
            self.control_status = f'steering moved to {self.current_steering:.2f}'
        elif key == 'c':
            if self.estop_active:
                self.control_status = 'ignored center steering while estop is active'
                return
            self.current_steering = 0.0
            self.control_status = 'steering centered'
        elif key == 'e':
            self.estop_active = True
            self.current_throttle = 0.0
            self.current_steering = 0.0
            self.publish_estop()
            self.control_status = 'emergency stop engaged'
        elif key == 'r':
            self.estop_active = False
            self.publish_estop()
            self.control_status = 'emergency stop released'

    def apply_control_patch(self, data):
        throttle = data.get('throttle')
        steering = data.get('steering')

        if throttle is not None:
            if self.estop_active:
                self.current_throttle = 0.0
                self.control_status = 'ignored throttle set while estop is active'
            else:
                self.current_throttle = self.clamp_control(throttle)
                self.control_status = f'throttle set to {self.current_throttle:.2f}'

        if steering is not None:
            if self.estop_active:
                self.current_steering = 0.0
                self.control_status = 'ignored steering set while estop is active'
            else:
                self.current_steering = self.clamp_control(steering)
                self.control_status = f'steering set to {self.current_steering:.2f}'

    def load_model(self):
        try:
            try:
                import torch
                if torch.cuda.is_available():
                    torch.backends.cudnn.benchmark = True
            except Exception:
                pass

            from ultralytics import YOLO

            model_path = self.resolve_model_path()
            model = YOLO(model_path)
            runtime_bits = [Path(model_path).suffix.lstrip('.').lower() or 'model']
            if self.inference_device:
                runtime_bits.append(self.inference_device)
            if self.prefer_half and str(model_path).lower().endswith('.pt'):
                runtime_bits.append('half')
            self.model_runtime = ', '.join(runtime_bits)
            self.model_loaded = True
            self.active_model_path = model_path
            self.latest_status = 'model loaded'
            self.get_logger().info(f'YOLO model loaded: {model_path}')
            return model
        except Exception as exc:
            self.model_loaded = False
            searched = ', '.join(self.model_search_paths)
            self.model_error = self.format_model_error(exc, searched)
            self.model_runtime = 'unavailable'
            self.latest_status = f'model unavailable: {self.model_error}'
            self.get_logger().warning(self.latest_status)
            return None

    def format_model_error(self, exc: Exception, searched: str) -> str:
        text = str(exc).strip()
        lowered = text.lower()
        if 'compiled using numpy 1.x' in lowered or '_array_api not found' in lowered:
            text = (
                'NumPy 2.x ABI mismatch with torch/onnxruntime. '
                'Install numpy<2, for example numpy==1.26.4.'
            )
        if searched:
            return f'{text}; searched: {searched}'
        return text

    def resolve_model_path(self):
        if self.model_path and self.model_path.lower() != 'auto':
            return str(Path(self.model_path).expanduser())

        self.model_search_paths = []
        for root in self.model_search_roots():
            for filename in ('yolov8n.engine', 'yolov8n.pt'):
                for candidate in (root / 'models' / filename, root / filename):
                    self.model_search_paths.append(str(candidate))
                    if candidate.exists():
                        return str(candidate)

        self.model_search_paths.append('yolov8n.pt (ultralytics default/cache/download)')
        return 'yolov8n.pt'

    def model_search_roots(self):
        roots = []

        def add_root(path):
            resolved = Path(path).expanduser().resolve()
            if resolved not in roots:
                roots.append(resolved)

        add_root(Path.cwd())
        for parent in Path.cwd().parents:
            if parent.name.endswith('_ws'):
                add_root(parent)
                break

        source_path = Path(__file__).resolve()
        for parent in source_path.parents:
            if parent.name.endswith('_ws'):
                add_root(parent)
                break

        try:
            add_root(get_package_share_directory('jetcar_perception'))
        except PackageNotFoundError:
            pass

        return roots

    def parse_sensor_id(self, value, default=None):
        if default is None:
            default = self.camera_sensor_id
        if value is None:
            return max(0, int(default))
        text = str(value).strip()
        if not text:
            return max(0, int(default))
        match = re.search(r'-?\d+', text)
        if match is None:
            return max(0, int(default))
        try:
            return max(0, int(match.group(0)))
        except ValueError:
            return max(0, int(default))

    def make_csi_pipeline(self, sensor_id=None):
        sensor_id = self.parse_sensor_id(sensor_id, self.camera_sensor_id)
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width={self.camera_width}, height={self.camera_height}, '
            f'framerate={self.camera_fps}/1, format=NV12 ! '
            f'nvvidconv flip-method={self.camera_flip_method} ! '
            'video/x-raw, format=BGRx ! videoconvert ! '
            'video/x-raw, format=BGR ! appsink name=capture_sink max-buffers=1 drop=true sync=false'
        )

    def open_capture(self):
        errors = []
        for source in self.camera_candidates():
            try:
                capture, active_source = self.open_capture_candidate(source)
            except Exception as exc:
                errors.append(f'{source}: {exc}')
                continue
            if not capture.isOpened():
                errors.append(f'{active_source}: not opened')
                capture.release()
                continue

            ok, frame = capture.read()
            if not ok or frame is None:
                errors.append(f'{active_source}: no frame')
                capture.release()
                continue

            self.pending_frame = self.normalize_frame(frame)
            self.active_camera_source = active_source
            self.camera_error_log = ''
            self.get_logger().info(f'camera opened: {active_source}')
            return capture

        self.active_camera_source = '; '.join(errors) if errors else self.camera_source
        return cv2.VideoCapture()

    def camera_candidates(self):
        source = self.camera_source.lower().strip()
        if source == 'auto':
            return [
                ('csi', self.camera_sensor_id),
                ('csi', 1 if self.camera_sensor_id == 0 else 0),
                '0',
                '1',
            ]
        if source.startswith('csi:'):
            return [('csi', self.parse_sensor_id(source.split(':', 1)[1], self.camera_sensor_id))]
        return [self.camera_source]

    def open_capture_candidate(self, source):
        if isinstance(source, tuple) and source[0] == 'csi':
            pipeline = self.make_csi_pipeline(sensor_id=source[1])
            return GStreamerCapture(pipeline), pipeline

        source = str(source)
        if source.lower() in ('csi', 'nvargus', 'nvarguscamerasrc'):
            pipeline = self.make_csi_pipeline()
            return GStreamerCapture(pipeline), pipeline
        if '!' in source or self.camera_backend == 'gstreamer':
            return GStreamerCapture(source), source
        if source.isdigit():
            return self.open_usb_capture(int(source))

        capture = cv2.VideoCapture(source)
        self.apply_capture_size(capture)
        return capture, source

    def open_usb_capture(self, camera_index: int):
        fourcc_name = self.usb_fourcc or 'DEFAULT'
        if fourcc_name in ('AUTO', 'DEFAULT'):
            capture = cv2.VideoCapture(camera_index)
            self.apply_capture_size(capture)
            return capture, f'/dev/video{camera_index}'

        capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        self.apply_capture_size(capture)
        if len(fourcc_name) == 4:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc_name))
        return capture, f'/dev/video{camera_index} fourcc={fourcc_name}'

    def read_frame(self, capture):
        if self.pending_frame is not None:
            frame = self.pending_frame
            self.pending_frame = None
            return True, frame
        ok, frame = capture.read()
        if not ok or frame is None:
            return False, None
        return True, self.normalize_frame(frame)

    def make_capture_unavailable_text(self):
        if self.camera_source.lower().strip() == 'auto':
            return f'Camera unavailable after trying: {self.active_camera_source}'
        if self.active_camera_source:
            return f'Camera unavailable: {self.active_camera_source}'
        return f'Camera unavailable: {self.camera_source}'

    def apply_capture_size(self, capture):
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.camera_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if self.camera_fps > 0:
            capture.set(cv2.CAP_PROP_FPS, self.camera_fps)

    def capture_loop(self):
        if self.left_image_topic or self.right_image_topic:
            last_time = time.monotonic()
            waiting_logged = False
            while not self.stop_event.is_set():
                with self.topic_frame_lock:
                    left_frame = None if self.left_topic_frame is None else self.left_topic_frame.copy()
                    right_frame = None if self.right_topic_frame is None else self.right_topic_frame.copy()
                    left_time = self.left_topic_frame_time
                    right_time = self.right_topic_frame_time
                left_age = 999.0 if left_time is None else max(0.0, time.monotonic() - left_time)
                right_age = 999.0 if right_time is None else max(0.0, time.monotonic() - right_time)
                frame = self.compose_stereo_frame(
                    left_frame if left_age <= 2.0 else None,
                    right_frame if right_age <= 2.0 else None,
                )
                if frame is None:
                    status = f'waiting for stereo image topics: {self.left_image_topic} | {self.right_image_topic}'
                    self.publish_status_frame(status)
                    self.update_camera_status(status, log_level='info', force_log=not waiting_logged)
                    waiting_logged = True
                    self.stop_event.wait(0.05)
                    continue

                waiting_logged = False
                frame = self.prepare_stream_frame(frame)
                if self.model is not None and left_frame is not None:
                    detector_input = self.prepare_stream_frame(left_frame)
                    self.submit_detection_frame(detector_input)
                    with self.detection_lock:
                        detections = list(self.latest_detections)
                    self.draw_detections(frame, detections)

                now = time.monotonic()
                elapsed = max(now - last_time, 0.001)
                last_time = now
                jpeg = self.encode_frame(frame)
                if jpeg is None:
                    continue
                with self.frame_lock:
                    self.latest_jpeg = jpeg
                    self.latest_frame_id += 1
                    self.latest_ready = True
                    self.latest_fps = 1.0 / elapsed
            return

        if self.image_topic:
            last_time = time.monotonic()
            waiting_logged = False
            while not self.stop_event.is_set():
                with self.topic_frame_lock:
                    frame = None if self.topic_frame is None else self.topic_frame.copy()
                    frame_time = self.topic_frame_time
                frame_age = 999.0 if frame_time is None else max(0.0, time.monotonic() - frame_time)
                if frame is None or frame_age > 2.0:
                    status = f'waiting for image topic: {self.image_topic}'
                    self.publish_status_frame(status)
                    self.update_camera_status(status, log_level='info', force_log=not waiting_logged)
                    waiting_logged = True
                    self.stop_event.wait(0.05)
                    continue

                waiting_logged = False
                frame = self.prepare_stream_frame(frame)
                if self.model is not None:
                    self.submit_detection_frame(frame)
                    with self.detection_lock:
                        detections = list(self.latest_detections)
                    self.draw_detections(frame, detections)

                now = time.monotonic()
                elapsed = max(now - last_time, 0.001)
                last_time = now
                jpeg = self.encode_frame(frame)
                if jpeg is None:
                    continue
                with self.frame_lock:
                    self.latest_jpeg = jpeg
                    self.latest_frame_id += 1
                    self.latest_ready = True
                    self.latest_fps = 1.0 / elapsed
            return

        while not self.stop_event.is_set():
            capture = self.open_capture()
            if not capture.isOpened():
                status = self.make_capture_unavailable_text()
                self.publish_status_frame(status)
                self.update_camera_status(status, log_level='error')
                self.stop_event.wait(self.camera_retry_sec)
                continue

            last_time = time.monotonic()
            read_failures = 0
            while not self.stop_event.is_set():
                ok, frame = self.read_frame(capture)
                if not ok or frame is None:
                    read_failures += 1
                    if read_failures >= self.camera_read_failures_before_reopen:
                        status = f'camera frame read failed, reopening: {self.active_camera_source}'
                        self.publish_status_frame(status)
                        self.update_camera_status(status, log_level='warning')
                        break
                    self.stop_event.wait(0.1)
                    continue

                read_failures = 0
                frame = self.prepare_stream_frame(frame)
                if self.model is not None:
                    self.submit_detection_frame(frame)
                    with self.detection_lock:
                        detections = list(self.latest_detections)
                    self.draw_detections(frame, detections)
                else:
                    cv2.putText(
                        frame,
                        'YOLO model unavailable',
                        (20, 36),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                now = time.monotonic()
                elapsed = max(now - last_time, 0.001)
                last_time = now
                jpeg = self.encode_frame(frame)
                if jpeg is None:
                    continue

                with self.frame_lock:
                    self.latest_jpeg = jpeg
                    self.latest_frame_id += 1
                    self.latest_ready = True
                    self.latest_fps = 1.0 / elapsed

            capture.release()
            if not self.stop_event.is_set():
                self.stop_event.wait(self.camera_retry_sec)

    def prepare_stream_frame(self, frame):
        frame = self.normalize_frame(frame)
        if self.stream_width > 0 and self.stream_height > 0:
            current_height, current_width = frame.shape[:2]
            if current_width != self.stream_width or current_height != self.stream_height:
                return cv2.resize(frame, (self.stream_width, self.stream_height), interpolation=cv2.INTER_AREA)
        return frame

    def normalize_frame(self, frame):
        if frame is None:
            return None
        if frame.dtype != np.uint8:
            frame = np.clip(frame, 0, 255).astype(np.uint8)
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.ndim == 3 and frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f'unsupported frame shape: {frame.shape}')
        if not frame.flags['C_CONTIGUOUS']:
            frame = np.ascontiguousarray(frame)
        return frame

    def submit_detection_frame(self, frame):
        if self.model is None:
            return
        now = time.monotonic()
        min_interval = 0.0 if self.detection_rate_hz <= 0 else 1.0 / self.detection_rate_hz
        if now < self.next_detection_time:
            return
        self.next_detection_time = now + min_interval
        if frame.shape[1] > 640:
            scale = 640.0 / float(frame.shape[1])
            resized = cv2.resize(
                frame,
                (640, max(1, int(round(frame.shape[0] * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            resized = frame
        with self.detection_lock:
            self.detector_frame = resized.copy()
        self.detector_event.set()

    def detection_loop(self):
        while not self.stop_event.is_set():
            if self.model is None:
                time.sleep(0.2)
                continue
            self.detector_event.wait(timeout=0.2)
            self.detector_event.clear()

            while not self.stop_event.is_set():
                with self.detection_lock:
                    frame = self.detector_frame
                    self.detector_frame = None
                if frame is None:
                    break

                started = time.monotonic()
                try:
                    detections = self.run_detection(frame)
                    detection_error = None
                except Exception as exc:
                    detections = []
                    detection_error = f'detection failed: {exc}'
                    self.get_logger().warning(detection_error)

                elapsed = max(time.monotonic() - started, 0.001)
                confidence = max((detection['confidence'] for detection in detections), default=0.0)
                with self.detection_lock:
                    self.latest_detections = detections
                with self.frame_lock:
                    self.latest_detection_count = len(detections)
                    self.latest_hazard = bool(detections)
                    self.latest_confidence = confidence
                    self.latest_detection_fps = 1.0 / elapsed
                    self.latest_status = detection_error or 'ok'

    def make_status_frame(self, text):
        frame = np.full((360, 640, 3), (20, 20, 20), dtype=np.uint8)
        for index, line in enumerate(self.wrap_text(text, 48)[:5]):
            cv2.putText(
                frame,
                line,
                (24, 130 + index * 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (240, 240, 240),
                2,
                cv2.LINE_AA,
            )
        return frame

    def wrap_text(self, text, max_chars):
        words = str(text).split()
        lines = []
        current = ''
        for word in words:
            if len(current) + len(word) + 1 > max_chars:
                if current:
                    lines.append(current)
                current = word
            else:
                current = f'{current} {word}'.strip()
        if current:
            lines.append(current)
        return lines or ['']

    def run_detection(self, frame):
        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            imgsz=self.image_size,
            device=self.inference_device or None,
            half=bool(self.prefer_half and str(self.active_model_path).lower().endswith('.pt')),
            verbose=False,
        )
        detections = []
        if not results:
            return detections

        result = results[0]
        names = result.names
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(names.get(class_id, class_id))
            confidence = float(box.conf[0])
            if self.target_classes and class_name not in self.target_classes:
                continue
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            detections.append({
                'class_name': class_name,
                'confidence': confidence,
                'box': (x1, y1, x2, y2),
            })
        return detections

    def draw_detections(self, frame, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection['box']
            label = f"{detection['class_name']} {detection['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (20, 220, 20), 2)
            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (20, 220, 20),
                2,
                cv2.LINE_AA,
            )

    def encode_frame(self, frame):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.jpeg_quality))]
        ok, encoded = cv2.imencode('.jpg', frame, params)
        if not ok:
            return None
        return encoded.tobytes()

    def setup_routes(self):
        @self.app.route('/', methods=['GET'])
        def index():
            return render_template_string(HTML_PAGE)

        @self.app.route('/api/status', methods=['GET'])
        def api_status():
            with self.frame_lock:
                return jsonify({
                    'ready': self.latest_ready,
                    'model_loaded': self.model_loaded,
                    'fps': self.latest_fps,
                    'detection_fps': self.latest_detection_fps,
                    'detection_count': self.latest_detection_count,
                    'hazard': self.latest_hazard,
                    'confidence': self.latest_confidence,
                    'model_error': self.model_error,
                    'model_path': self.active_model_path or self.model_path,
                    'model_search_paths': self.model_search_paths,
                    'camera_source': self.active_camera_source or self.camera_source,
                    'status': self.latest_status,
                })

        @self.app.route('/api/control_state', methods=['GET'])
        def api_control_state():
            return jsonify(self.control_state_payload())

        @self.app.route('/api/control_command', methods=['POST'])
        def api_control_command():
            data = request.get_json(silent=True) or {}
            key = str(data.get('key', '')).lower().strip()
            if key:
                with self.control_lock:
                    self.control_queue.append(key)
            return jsonify(self.control_state_payload())

        @self.app.route('/api/set_control_state', methods=['POST'])
        def api_set_control_state():
            data = request.get_json(silent=True) or {}
            with self.control_lock:
                self.apply_control_patch(data)
            return jsonify(self.control_state_payload())

        @self.app.route('/api/set_drive_mode', methods=['POST'])
        def api_set_drive_mode():
            data = request.get_json(silent=True) or {}
            mode = str(data.get('mode', 'MANUAL')).upper().strip()
            if mode in ('MANUAL', 'AI_INTERVENTION', 'AUTONOMOUS'):
                self.publish_drive_mode(mode)
            return jsonify(self.control_state_payload())

        @self.app.route('/stream', methods=['GET'])
        def stream():
            response = Response(self.stream_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
            response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
            response.headers['Pragma'] = 'no-cache'
            response.headers['X-Accel-Buffering'] = 'no'
            return response

    def stream_frames(self):
        last_frame_id = -1
        while not self.stop_event.is_set():
            with self.frame_lock:
                jpeg = self.latest_jpeg
                frame_id = self.latest_frame_id
            if jpeg is None or frame_id == last_frame_id:
                time.sleep(0.002)
                continue
            last_frame_id = frame_id
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
            if self.stream_delay > 0:
                time.sleep(self.stream_delay)

    def run_flask(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)

    def timer_callback(self):
        with self.control_lock:
            while self.control_queue:
                self.handle_control_key(self.control_queue.popleft())
            self.publish_control()
            self.publish_estop()

        with self.frame_lock:
            ready = self.latest_ready
            hazard = self.latest_hazard
            confidence = self.latest_confidence
            status = (
                f'model_loaded={self.model_loaded}, camera_source={self.active_camera_source or self.camera_source}, '
                f'detections={self.latest_detection_count}, fps={self.latest_fps:.1f}, '
                f'detection_fps={self.latest_detection_fps:.1f}, '
                f'status={self.latest_status}'
            )

        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)

        hazard_msg = Bool()
        hazard_msg.data = hazard
        self.hazard_pub.publish(hazard_msg)

        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.closest_pub.publish(confidence_msg)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def shutdown(self):
        self.stop_event.set()
        self.detector_event.set()
        self.worker_thread.join(timeout=1.0)
        self.detector_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
