import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from flask import Flask, Response, jsonify, render_template_string, request
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>JetCar YOLO Dashboard</title>
    <style>
        :root {
            color-scheme: dark;
            --bg: #091113;
            --panel: rgba(15, 26, 29, 0.92);
            --line: rgba(133, 196, 180, 0.18);
            --text: #edf5f2;
            --muted: #9db2ac;
            --accent: #3fe0ab;
            --danger: #ef6258;
            --shadow: 0 22px 56px rgba(0, 0, 0, 0.35);
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            font-family: "Segoe UI", "Noto Sans KR", sans-serif;
            background:
                radial-gradient(circle at top left, rgba(63, 224, 171, 0.14), transparent 28%),
                radial-gradient(circle at top right, rgba(239, 98, 88, 0.12), transparent 22%),
                linear-gradient(180deg, #071012 0%, #0d1719 100%);
            color: var(--text);
        }
        .shell {
            max-width: 1400px;
            margin: 0 auto;
            padding: 18px;
        }
        .hero {
            display: flex;
            justify-content: space-between;
            gap: 16px;
            align-items: flex-start;
            margin-bottom: 18px;
        }
        .hero h1 {
            margin: 0;
            font-size: clamp(2rem, 4vw, 3.2rem);
            letter-spacing: 0.04em;
        }
        .hero p {
            margin: 8px 0 0;
            color: var(--muted);
            max-width: 760px;
        }
        .pill {
            padding: 10px 14px;
            border-radius: 999px;
            border: 1px solid rgba(63, 224, 171, 0.26);
            background: rgba(63, 224, 171, 0.1);
            white-space: nowrap;
        }
        .layout {
            display: grid;
            grid-template-columns: 1.25fr 0.95fr;
            gap: 18px;
        }
        .panel {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 24px;
            box-shadow: var(--shadow);
            overflow: hidden;
        }
        .panel-inner {
            padding: 18px;
        }
        .stream-wrap {
            position: relative;
            background: #000;
            border-radius: 18px;
            overflow: hidden;
        }
        .stream-wrap img {
            display: block;
            width: 100%;
            min-height: 320px;
            background: #000;
            object-fit: contain;
        }
        .overlay {
            position: absolute;
            left: 14px;
            bottom: 14px;
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }
        .overlay .pill {
            background: rgba(7, 16, 18, 0.8);
            border-color: rgba(255, 255, 255, 0.08);
            font-size: 0.92rem;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 10px;
            margin-top: 14px;
        }
        .stat {
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.06);
            border-radius: 18px;
            padding: 12px;
        }
        .stat label {
            display: block;
            color: var(--muted);
            font-size: 0.8rem;
            text-transform: uppercase;
            letter-spacing: 0.08em;
            margin-bottom: 6px;
        }
        .stat strong {
            display: block;
            font-size: 1.45rem;
        }
        .cards {
            display: grid;
            gap: 14px;
        }
        .card {
            background: rgba(255, 255, 255, 0.025);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 20px;
            padding: 16px;
        }
        .card-head {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 10px;
            margin-bottom: 10px;
        }
        .card-head h2 {
            margin: 0;
            font-size: 1rem;
            text-transform: uppercase;
            letter-spacing: 0.06em;
        }
        .card-head span {
            color: var(--muted);
            font-size: 0.92rem;
        }
        input[type="range"] {
            width: 100%;
            accent-color: var(--accent);
        }
        .btn-row {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 10px;
            margin-top: 10px;
        }
        .pad {
            display: grid;
            grid-template-columns: repeat(3, minmax(70px, 1fr));
            gap: 10px;
        }
        button {
            min-height: 58px;
            border: 1px solid rgba(255, 255, 255, 0.08);
            border-radius: 16px;
            background: linear-gradient(180deg, rgba(28, 49, 53, 0.96), rgba(16, 29, 32, 0.98));
            color: var(--text);
            font-size: 0.98rem;
            font-weight: 700;
            cursor: pointer;
            transition: transform 100ms ease, border-color 100ms ease;
            touch-action: manipulation;
        }
        button:hover { border-color: rgba(63, 224, 171, 0.28); }
        button:active, button.active { transform: translateY(1px) scale(0.99); }
        button.stop {
            background: linear-gradient(180deg, rgba(73, 81, 86, 0.98), rgba(42, 50, 55, 0.98));
        }
        button.release {
            background: linear-gradient(180deg, rgba(63, 224, 171, 0.95), rgba(30, 147, 113, 0.98));
            color: #04100c;
        }
        button.danger {
            background: linear-gradient(180deg, rgba(223, 92, 80, 0.98), rgba(129, 37, 37, 0.98));
        }
        pre {
            margin: 0;
            white-space: pre-wrap;
            word-break: break-word;
            color: var(--muted);
            line-height: 1.5;
        }
        @media (max-width: 960px) {
            .layout { grid-template-columns: 1fr; }
            .grid { grid-template-columns: repeat(2, 1fr); }
            .btn-row { grid-template-columns: repeat(2, 1fr); }
        }
        @media (max-width: 560px) {
            .shell { padding: 12px; }
            .hero { flex-direction: column; }
            .grid, .btn-row { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="shell">
        <div class="hero">
            <div>
                <h1>JetCar YOLO Dashboard</h1>
                <p>YOLO 객체 인식 영상과 수동 주행 제어를 한 화면에서 처리합니다. 브라우저만으로 조작 가능하고, `E-STOP`은 즉시 `/system/estop_cmd`로 전송됩니다.</p>
            </div>
            <div class="pill" id="connectState">Connecting...</div>
        </div>

        <div class="layout">
            <div class="panel">
                <div class="panel-inner">
                    <div class="stream-wrap">
                        <img src="/stream" alt="YOLO stream">
                        <div class="overlay">
                            <div class="pill" id="overlayStatus">status</div>
                            <div class="pill" id="overlayCamera">camera</div>
                            <div class="pill" id="overlayModel">model</div>
                        </div>
                    </div>
                    <div class="grid">
                        <div class="stat"><label>Detections</label><strong id="detCount">0</strong></div>
                        <div class="stat"><label>Hazard</label><strong id="hazardValue">false</strong></div>
                        <div class="stat"><label>FPS</label><strong id="fpsValue">0.0</strong></div>
                        <div class="stat"><label>Det FPS</label><strong id="detFpsValue">0.0</strong></div>
                    </div>
                </div>
            </div>

            <div class="panel">
                <div class="panel-inner cards">
                    <div class="card">
                        <div class="card-head">
                            <h2>Manual Control</h2>
                            <span id="controlStatus">ready</span>
                        </div>
                        <div class="grid" style="grid-template-columns: repeat(3, 1fr); margin-top: 0;">
                            <div class="stat"><label>Throttle</label><strong id="throttleValue">0.00</strong></div>
                            <div class="stat"><label>Steering</label><strong id="steeringValue">0.00</strong></div>
                            <div class="stat"><label>E-Stop</label><strong id="estopValue">OFF</strong></div>
                        </div>
                    </div>

                    <div class="card">
                        <div class="card-head">
                            <h2>Throttle</h2>
                            <span id="throttlePercent">0%</span>
                        </div>
                        <input id="throttleSlider" type="range" min="-100" max="100" value="0" step="1">
                        <div class="btn-row">
                            <button data-throttle="-40">REV 40%</button>
                            <button data-throttle="-20">REV 20%</button>
                            <button class="stop" data-action="stop">STOP</button>
                            <button data-throttle="20">FWD 20%</button>
                            <button data-throttle="40">FWD 40%</button>
                        </div>
                    </div>

                    <div class="card">
                        <div class="card-head">
                            <h2>Steering</h2>
                            <span id="steeringPercent">0%</span>
                        </div>
                        <input id="steeringSlider" type="range" min="-100" max="100" value="0" step="1">
                        <div class="btn-row">
                            <button data-steering="-100">HARD L</button>
                            <button data-steering="-40">LEFT</button>
                            <button class="stop" data-action="center">CENTER</button>
                            <button data-steering="40">RIGHT</button>
                            <button data-steering="100">HARD R</button>
                        </div>
                    </div>

                    <div class="card">
                        <div class="card-head">
                            <h2>Touch Drive Pad</h2>
                            <span>press and hold</span>
                        </div>
                        <div class="pad">
                            <div></div>
                            <button data-key="w">Forward</button>
                            <div></div>
                            <button data-key="a">Left</button>
                            <button class="stop" data-key="x">Brake</button>
                            <button data-key="d">Right</button>
                            <div></div>
                            <button data-key="s">Reverse</button>
                            <div></div>
                        </div>
                    </div>

                    <div class="card">
                        <div class="card-head">
                            <h2>Safety</h2>
                            <span>Immediate action</span>
                        </div>
                        <div class="btn-row">
                            <button class="danger" data-action="estop">E-STOP</button>
                            <button class="release" data-action="release">RELEASE</button>
                            <button class="stop" data-action="reset-all">RESET ALL</button>
                        </div>
                    </div>

                    <div class="card">
                        <div class="card-head">
                            <h2>System</h2>
                            <span>Live state</span>
                        </div>
                        <pre id="systemText">Loading...</pre>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        const throttleSlider = document.getElementById('throttleSlider');
        const steeringSlider = document.getElementById('steeringSlider');
        const throttleValue = document.getElementById('throttleValue');
        const steeringValue = document.getElementById('steeringValue');
        const estopValue = document.getElementById('estopValue');
        const throttlePercent = document.getElementById('throttlePercent');
        const steeringPercent = document.getElementById('steeringPercent');
        const controlStatus = document.getElementById('controlStatus');
        const connectState = document.getElementById('connectState');
        const systemText = document.getElementById('systemText');

        async function fetchJson(url, options) {
            const response = await fetch(url, options);
            return response.json();
        }

        function updateVision(data) {
            document.getElementById('detCount').textContent = data.detection_count;
            document.getElementById('hazardValue').textContent = String(data.hazard);
            document.getElementById('fpsValue').textContent = Number(data.fps).toFixed(1);
            document.getElementById('detFpsValue').textContent = Number(data.detection_fps).toFixed(1);
            document.getElementById('overlayStatus').textContent = data.status;
            document.getElementById('overlayCamera').textContent = data.camera_source;
            document.getElementById('overlayModel').textContent = data.model_path;
            systemText.textContent =
                `ready=${data.ready}\n` +
                `model_loaded=${data.model_loaded}\n` +
                `camera=${data.camera_source}\n` +
                `model=${data.model_path}\n` +
                `confidence=${Number(data.confidence).toFixed(2)}\n` +
                `status=${data.status}`;
        }

        function updateControl(data) {
            throttleValue.textContent = Number(data.throttle).toFixed(2);
            steeringValue.textContent = Number(data.steering).toFixed(2);
            estopValue.textContent = data.estop ? 'ON' : 'OFF';
            throttleSlider.value = Math.round(Number(data.throttle) * 100);
            steeringSlider.value = Math.round(Number(data.steering) * 100);
            throttlePercent.textContent = `${Math.round(Number(data.throttle) * 100)}%`;
            steeringPercent.textContent = `${Math.round(Number(data.steering) * 100)}%`;
            controlStatus.textContent = data.status;
        }

        async function refreshAll() {
            try {
                const [vision, control] = await Promise.all([
                    fetchJson('/api/status'),
                    fetchJson('/api/control_state')
                ]);
                updateVision(vision);
                updateControl(control);
                connectState.textContent = 'Connected';
            } catch (error) {
                connectState.textContent = 'Disconnected';
                systemText.textContent = 'poll failed: ' + error;
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

        throttleSlider.addEventListener('input', function() {
            throttlePercent.textContent = `${throttleSlider.value}%`;
        });
        steeringSlider.addEventListener('input', function() {
            steeringPercent.textContent = `${steeringSlider.value}%`;
        });
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
        document.querySelectorAll('[data-action]').forEach(function(button) {
            button.addEventListener('click', function() {
                const action = button.dataset.action;
                if (action === 'stop') {
                    postControl('/api/set_control_state', {throttle: 0.0});
                } else if (action === 'center') {
                    postControl('/api/set_control_state', {steering: 0.0});
                } else if (action === 'reset-all') {
                    postControl('/api/set_control_state', {throttle: 0.0, steering: 0.0});
                } else if (action === 'estop') {
                    postControl('/api/control_command', {key: 'e'});
                } else if (action === 'release') {
                    postControl('/api/control_command', {key: 'r'});
                }
            });
        });

        const repeatKeys = new Set(['w', 'a', 's', 'd']);
        const singleKeys = new Set(['x', 'e', 'r', 'c']);
        const activeKeys = new Set();

        function pressKey(key) {
            if (repeatKeys.has(key)) {
                if (!activeKeys.has(key)) {
                    activeKeys.add(key);
                    postControl('/api/control_command', {key});
                }
            } else if (singleKeys.has(key)) {
                postControl('/api/control_command', {key});
            }
        }

        function releaseKey(key) {
            activeKeys.delete(key);
            document.querySelectorAll(`[data-key="${key}"]`).forEach(function(button) {
                button.classList.remove('active');
            });
        }

        setInterval(function() {
            activeKeys.forEach(function(key) {
                postControl('/api/control_command', {key});
            });
        }, 80);

        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            if (repeatKeys.has(key) || singleKeys.has(key)) {
                event.preventDefault();
                pressKey(key);
                document.querySelectorAll(`[data-key="${key}"]`).forEach(function(button) {
                    button.classList.add('active');
                });
            }
        });
        document.addEventListener('keyup', function(event) {
            releaseKey(event.key.toLowerCase());
        });

        document.querySelectorAll('button[data-key]').forEach(function(button) {
            const key = button.dataset.key;
            const start = function(event) {
                event.preventDefault();
                pressKey(key);
                button.classList.add('active');
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
        });

        setInterval(refreshAll, 1000);
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
        self.declare_parameter('usb_fourcc', 'auto')
        self.declare_parameter('model_path', 'auto')
        self.declare_parameter('target_classes', ['person', 'car', 'stop sign'])
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_size', 416)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('jpeg_quality', 65)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 60)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('stream_height', 360)
        self.declare_parameter('stream_delay_ms', 5)
        self.declare_parameter('detection_rate_hz', 8.0)
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
        self.detection_rate_hz = float(self.get_parameter('detection_rate_hz').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.steering_step_cmd = float(self.get_parameter('steering_step_cmd').value)

        self.ready_pub = self.create_publisher(Bool, '/perception/detections/ready', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/detections/hazard', 10)
        self.closest_pub = self.create_publisher(Float32, '/perception/detections/closest_confidence', 10)
        self.status_pub = self.create_publisher(String, '/perception/detections/status', 10)
        self.manual_throttle_pub = self.create_publisher(Float32, '/input/manual/throttle', 10)
        self.manual_steering_pub = self.create_publisher(Float32, '/input/manual/steering', 10)
        self.estop_pub = self.create_publisher(Bool, '/system/estop_cmd', 10)
        self.create_subscription(Bool, '/vehicle/emergency_stop_state', self.estop_state_cb, 10)

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
        self.control_status = f'estop_state={self.estop_active}'

    def control_state_payload(self):
        return {
            'ok': True,
            'throttle': float(self.current_throttle),
            'steering': float(self.current_steering),
            'estop': bool(self.estop_active),
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
            from ultralytics import YOLO

            model_path = self.resolve_model_path()
            model = YOLO(model_path)
            self.model_loaded = True
            self.active_model_path = model_path
            self.latest_status = 'model loaded'
            self.get_logger().info(f'YOLO model loaded: {model_path}')
            return model
        except Exception as exc:
            self.model_loaded = False
            searched = ', '.join(self.model_search_paths)
            self.model_error = self.format_model_error(exc, searched)
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

    def make_csi_pipeline(self, sensor_id=None):
        if sensor_id is None:
            sensor_id = self.camera_sensor_id
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width={self.camera_width}, height={self.camera_height}, '
            f'framerate={self.camera_fps}/1, format=NV12 ! '
            f'nvvidconv flip-method={self.camera_flip_method} ! '
            'video/x-raw, format=BGRx ! videoconvert ! '
            'video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false'
        )

    def open_capture(self):
        errors = []
        for source in self.camera_candidates():
            capture, active_source = self.open_capture_candidate(source)
            if not capture.isOpened():
                errors.append(f'{active_source}: not opened')
                capture.release()
                continue

            ok, frame = capture.read()
            if not ok or frame is None:
                errors.append(f'{active_source}: no frame')
                capture.release()
                continue

            self.pending_frame = frame
            self.active_camera_source = active_source
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
        return [self.camera_source]

    def open_capture_candidate(self, source):
        if isinstance(source, tuple) and source[0] == 'csi':
            pipeline = self.make_csi_pipeline(sensor_id=source[1])
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), pipeline

        source = str(source)
        if source.lower() in ('csi', 'nvargus', 'nvarguscamerasrc'):
            pipeline = self.make_csi_pipeline()
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), pipeline
        if '!' in source or self.camera_backend == 'gstreamer':
            return cv2.VideoCapture(source, cv2.CAP_GSTREAMER), source
        if source.isdigit():
            return self.open_usb_capture(int(source))

        capture = cv2.VideoCapture(source)
        self.apply_capture_size(capture)
        return capture, source

    def open_usb_capture(self, camera_index: int):
        candidates = []
        if self.usb_fourcc == 'AUTO':
            candidates = ['DEFAULT', 'MJPG', 'YUYV']
        else:
            candidates = [self.usb_fourcc]

        last_capture = None
        last_source = f'/dev/video{camera_index}'
        for fourcc_name in candidates:
            capture = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            self.apply_capture_size(capture)
            if fourcc_name != 'DEFAULT' and len(fourcc_name) == 4:
                capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc_name))

            active_source = f'/dev/video{camera_index} fourcc={fourcc_name}'
            if capture.isOpened():
                ok, frame = capture.read()
                if ok and frame is not None:
                    self.pending_frame = self.normalize_frame(frame)
                    return capture, active_source

            if last_capture is not None:
                last_capture.release()
            last_capture = capture
            last_source = active_source

        if last_capture is None:
            last_capture = cv2.VideoCapture(camera_index)
        return last_capture, last_source

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
        capture = self.open_capture()
        if not capture.isOpened():
            status = self.make_capture_unavailable_text()
            jpeg = self.encode_frame(self.make_status_frame(status))
            with self.frame_lock:
                self.latest_status = status
                self.latest_jpeg = jpeg
            self.get_logger().error(self.latest_status)
            return

        last_time = time.monotonic()
        while not self.stop_event.is_set():
            ok, frame = self.read_frame(capture)
            if not ok or frame is None:
                with self.frame_lock:
                    self.latest_ready = False
                    self.latest_status = 'camera frame read failed'
                time.sleep(0.1)
                continue

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
            fps = 1.0 / elapsed
            jpeg = self.encode_frame(frame)
            if jpeg is None:
                continue

            with self.frame_lock:
                self.latest_jpeg = jpeg
                self.latest_frame_id += 1
                self.latest_ready = True
                self.latest_fps = fps

        capture.release()

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
        with self.detection_lock:
            self.detector_frame = frame.copy()
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
            if self.estop_active:
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
