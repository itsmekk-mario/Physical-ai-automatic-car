import threading
from collections import deque

from flask import Flask, jsonify, render_template_string, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>JetCar Web Control</title>
    <style>
        :root {
            color-scheme: dark;
            --bg: #090b0d;
            --panel: #121517;
            --panel-2: #171b1f;
            --panel-3: #1e2428;
            --border: #2b3338;
            --text: #f4f7f5;
            --muted: #9ca8a3;
            --accent: #44d7b6;
            --accent-2: #e8c55a;
            --danger: #ef615b;
            --ok: #58c77c;
            --shadow: 0 18px 46px rgba(0, 0, 0, 0.42);
        }
        * {
            box-sizing: border-box;
        }
        body {
            font-family: "Segoe UI", "Noto Sans KR", sans-serif;
            background:
                linear-gradient(180deg, #090b0d 0%, #111618 52%, #090b0d 100%);
            color: var(--text);
            margin: 0;
            min-height: 100vh;
        }
        .shell {
            max-width: 1180px;
            margin: 0 auto;
            padding: 18px;
        }
        .hero {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 12px;
            margin-bottom: 16px;
        }
        .hero h1 {
            margin: 0;
            font-size: 1.35rem;
            letter-spacing: 0;
            font-weight: 800;
        }
        .hero p {
            margin: 5px 0 0;
            color: var(--muted);
            font-size: 0.86rem;
        }
        .badge {
            min-width: 124px;
            padding: 10px 14px;
            border-radius: 999px;
            background: #101416;
            border: 1px solid var(--border);
            color: var(--accent);
            font-size: 0.94rem;
            font-weight: 800;
            white-space: nowrap;
            text-align: center;
        }
        .layout {
            display: grid;
            grid-template-columns: 1.2fr 0.9fr;
            gap: 16px;
        }
        .panel {
            background: var(--panel);
            border: 1px solid var(--border);
            border-radius: 8px;
            box-shadow: var(--shadow);
        }
        .panel-inner {
            padding: 14px;
        }
        .meter-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 12px;
            margin-bottom: 16px;
        }
        .stat {
            background: #101416;
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 14px;
        }
        .stat label {
            display: block;
            color: var(--muted);
            font-size: 0.84rem;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 0;
        }
        .stat strong {
            font-size: 1.75rem;
            display: block;
        }
        .status-line {
            color: var(--muted);
            min-height: 1.4em;
            margin-top: 6px;
        }
        .controls {
            display: grid;
            gap: 16px;
        }
        .control-card {
            background: var(--panel-2);
            border-radius: 8px;
            border: 1px solid var(--border);
            padding: 18px;
        }
        .control-head {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .control-head h2 {
            margin: 0;
            font-size: 1rem;
            text-transform: uppercase;
            letter-spacing: 0;
        }
        .control-head span {
            color: var(--muted);
            font-size: 0.94rem;
        }
        input[type="range"] {
            width: 100%;
            accent-color: var(--accent);
        }
        input.vertical-range {
            width: 38px;
            height: 260px;
            writing-mode: vertical-lr;
            direction: rtl;
        }
        .drive-axes {
            display: grid;
            grid-template-columns: 128px minmax(0, 1fr);
            gap: 14px;
            align-items: stretch;
        }
        .axis-panel {
            background: var(--panel-3);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 14px;
        }
        .throttle-column {
            display: grid;
            grid-template-rows: auto 1fr auto;
            gap: 10px;
            min-height: 382px;
        }
        .throttle-rail {
            display: grid;
            place-items: center;
            position: relative;
        }
        .throttle-rail::before {
            content: "";
            position: absolute;
            width: 8px;
            height: 76%;
            border-radius: 999px;
            background: linear-gradient(0deg, #7b3636 0%, #2e3438 50%, #2ebd9e 100%);
        }
        .steering-console {
            display: grid;
            grid-template-rows: auto 1fr auto;
            gap: 12px;
            min-height: 382px;
        }
        .steering-display {
            position: relative;
            min-height: 146px;
            border-radius: 8px;
            border: 1px solid #334047;
            background:
                linear-gradient(90deg, rgba(239, 97, 91, 0.12), transparent 34%, transparent 66%, rgba(68, 215, 182, 0.12)),
                #111517;
        }
        .steering-display::before {
            content: "";
            position: absolute;
            left: 20px;
            right: 20px;
            top: 70px;
            height: 4px;
            border-radius: 999px;
            background: #334047;
        }
        .steering-dot {
            position: absolute;
            left: calc(var(--steer-left, 50%) - 15px);
            top: 57px;
            width: 30px;
            height: 30px;
            border-radius: 999px;
            background: var(--accent);
            border: 2px solid #06100d;
            box-shadow: 0 0 24px rgba(68, 215, 182, 0.44);
            transition: left 120ms ease;
        }
        .axis-actions {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        .preset-row {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 10px;
            margin-top: 12px;
        }
        .mode-row {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        .drive-pad {
            display: grid;
            grid-template-columns: repeat(3, minmax(72px, 1fr));
            gap: 10px;
        }
        button {
            min-height: 58px;
            font-size: 1rem;
            font-weight: 700;
            border: 1px solid var(--border);
            border-radius: 8px;
            cursor: pointer;
            background: #242b2f;
            color: var(--text);
            transition: transform 120ms ease, border-color 120ms ease, background 120ms ease;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }
        button:hover {
            background: #2c3439;
            border-color: #405158;
        }
        button:active,
        button.active {
            transform: translateY(1px) scale(0.99);
            background: #213833;
            border-color: var(--accent);
        }
        button.subtle {
            min-height: 52px;
            font-size: 0.95rem;
        }
        button.stop {
            background: #2c3238;
        }
        button.danger {
            background: #432121;
            border-color: #8d3432;
        }
        button.release {
            background: linear-gradient(180deg, #65e0c2 0%, #2ea98d 100%);
            color: #03100b;
        }
        .telemetry {
            display: grid;
            gap: 12px;
        }
        .telemetry-box {
            padding: 14px;
            border-radius: 8px;
            background: var(--panel-2);
            border: 1px solid var(--border);
        }
        .telemetry-box h3 {
            margin: 0 0 10px;
            font-size: 0.92rem;
            text-transform: uppercase;
            letter-spacing: 0;
            color: var(--muted);
        }
        .telemetry-box pre {
            margin: 0;
            white-space: pre-wrap;
            word-break: break-word;
            font-family: inherit;
            line-height: 1.45;
        }
        @media (max-width: 900px) {
            .layout {
                grid-template-columns: 1fr;
            }
            .meter-grid,
            .preset-row,
            .mode-row,
            .drive-axes {
                grid-template-columns: repeat(2, 1fr);
            }
        }
        @media (max-width: 560px) {
            .shell {
                padding: 14px 12px 32px;
            }
            .hero {
                flex-direction: column;
                align-items: flex-start;
            }
            .meter-grid,
            .preset-row,
            .mode-row,
            .drive-axes {
                grid-template-columns: 1fr;
            }
            input.vertical-range {
                width: 100%;
                height: auto;
                writing-mode: horizontal-tb;
                direction: ltr;
            }
            .throttle-rail::before {
                display: none;
            }
        }
    </style>
</head>
<body>
    <div class="shell">
        <div class="hero">
            <div>
                <h1>JetCar Control Deck</h1>
                <p>Manual drive control</p>
            </div>
            <div class="badge" id="connectionBadge">Connecting...</div>
        </div>

        <div class="layout">
            <div class="panel">
                <div class="panel-inner">
                    <div class="meter-grid">
                        <div class="stat">
                            <label>Throttle</label>
                            <strong id="throttleValue">0.00</strong>
                        </div>
                        <div class="stat">
                            <label>Steering</label>
                            <strong id="steeringValue">0.00</strong>
                        </div>
                        <div class="stat">
                            <label>E-Stop</label>
                            <strong id="estopValue">OFF</strong>
                        </div>
                        <div class="stat">
                            <label>Mode</label>
                            <strong id="driveModeValue">MANUAL</strong>
                        </div>
                    </div>

                    <div class="controls">
                        <div class="control-card">
                            <div class="control-head">
                                <h2>Drive Mode</h2>
                                <span id="driveModeHint">MANUAL</span>
                            </div>
                            <div class="mode-row">
                                <button class="subtle" data-mode="MANUAL">MANUAL</button>
                                <button class="subtle" data-mode="AI_INTERVENTION">AI INTERVENTION</button>
                                <button class="subtle" data-mode="AUTONOMOUS">AUTONOMOUS</button>
                            </div>
                        </div>

                        <div class="control-card">
                            <div class="control-head">
                                <h2>Drive Axes</h2>
                                <span><span id="throttlePercent">0%</span> / <span id="steeringPercent">0%</span></span>
                            </div>
                            <div class="drive-axes">
                                <div class="axis-panel throttle-column">
                                    <button data-key="w">FWD</button>
                                    <div class="throttle-rail">
                                        <input class="vertical-range" id="throttleSlider" type="range" min="-100" max="100" value="0" step="1">
                                    </div>
                                    <button data-key="s">REV</button>
                                </div>
                                <div class="axis-panel steering-console">
                                    <div class="axis-actions">
                                        <button data-key="a">LEFT</button>
                                        <button class="stop" data-key="c">CENTER</button>
                                        <button data-key="d">RIGHT</button>
                                    </div>
                                    <div class="steering-display">
                                        <div class="steering-dot" id="steeringDot"></div>
                                    </div>
                                    <input id="steeringSlider" type="range" min="-100" max="100" value="0" step="1">
                                </div>
                            </div>
                            <div class="preset-row">
                                <button class="subtle" data-throttle="-30">REV 30%</button>
                                <button class="subtle stop" data-action="stop">STOP</button>
                                <button class="subtle" data-throttle="30">FWD 30%</button>
                                <button class="subtle" data-steering="-40">LEFT</button>
                                <button class="subtle" data-steering="40">RIGHT</button>
                            </div>
                        </div>

                        <div class="control-card">
                            <div class="control-head">
                                <h2>Touch Pad</h2>
                                <span>live input</span>
                            </div>
                            <div class="drive-pad">
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

                        <div class="control-card">
                            <div class="control-head">
                                <h2>Safety</h2>
                                <span>E-stop path</span>
                            </div>
                            <div class="preset-row">
                                <button class="danger" data-action="estop">E-STOP</button>
                                <button class="release" data-action="release">RELEASE</button>
                                <button class="stop" data-action="reset-all">RESET ALL</button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <div class="panel">
                <div class="panel-inner telemetry">
                    <div class="telemetry-box">
                        <h3>Node Status</h3>
                        <pre id="statusText">Waiting for state...</pre>
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
        const driveModeValue = document.getElementById('driveModeValue');
        const driveModeHint = document.getElementById('driveModeHint');
        const statusText = document.getElementById('statusText');
        const throttlePercent = document.getElementById('throttlePercent');
        const steeringPercent = document.getElementById('steeringPercent');
        const connectionBadge = document.getElementById('connectionBadge');

        async function postJson(url, payload) {
            const response = await fetch(url, {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(payload || {})
            });
            return response.json();
        }

        function setConnectionState(connected) {
            connectionBadge.textContent = connected ? 'Connected' : 'Disconnected';
            connectionBadge.style.background = connected ? 'rgba(77, 215, 168, 0.12)' : 'rgba(241, 92, 82, 0.12)';
            connectionBadge.style.borderColor = connected ? 'rgba(77, 215, 168, 0.3)' : 'rgba(241, 92, 82, 0.35)';
            connectionBadge.style.color = connected ? '#c8ffef' : '#ffd4d0';
        }

        function updateUi(state) {
            throttleValue.textContent = Number(state.throttle).toFixed(2);
            steeringValue.textContent = Number(state.steering).toFixed(2);
            estopValue.textContent = state.estop ? 'ON' : 'OFF';
            driveModeValue.textContent = state.drive_mode;
            driveModeHint.textContent = state.drive_mode;
            throttleSlider.value = Math.round(Number(state.throttle) * 100);
            const steeringPercentValue = Math.round(Number(state.steering) * 100);
            steeringSlider.value = steeringPercentValue;
            document.getElementById('steeringDot').style.setProperty('--steer-left', `${50 + steeringPercentValue * 0.4}%`);
            throttlePercent.textContent = `${Math.round(Number(state.throttle) * 100)}%`;
            steeringPercent.textContent = `${steeringPercentValue}%`;
            statusText.textContent = state.status;
            document.querySelectorAll('[data-mode]').forEach(function(button) {
                button.classList.toggle('active', button.dataset.mode === state.drive_mode);
            });
        }

        async function refreshState() {
            try {
                const response = await fetch('/api/state');
                const state = await response.json();
                updateUi(state);
                setConnectionState(true);
            } catch (error) {
                setConnectionState(false);
                statusText.textContent = 'State poll failed: ' + error;
            }
        }

        async function sendCommand(key) {
            const state = await postJson('/api/command', {key});
            updateUi(state);
        }

        async function setManualState(patch) {
            const state = await postJson('/api/set_state', patch);
            updateUi(state);
        }

        async function setDriveMode(mode) {
            const state = await postJson('/api/set_drive_mode', {mode});
            updateUi(state);
        }

        throttleSlider.addEventListener('input', function() {
            throttlePercent.textContent = `${throttleSlider.value}%`;
        });

        steeringSlider.addEventListener('input', function() {
            steeringPercent.textContent = `${steeringSlider.value}%`;
        });

        throttleSlider.addEventListener('change', function() {
            setManualState({throttle: Number(throttleSlider.value) / 100.0});
        });

        steeringSlider.addEventListener('change', function() {
            setManualState({steering: Number(steeringSlider.value) / 100.0});
        });

        document.querySelectorAll('[data-throttle]').forEach(function(button) {
            button.addEventListener('click', function() {
                setManualState({throttle: Number(button.dataset.throttle) / 100.0});
            });
        });

        document.querySelectorAll('[data-steering]').forEach(function(button) {
            button.addEventListener('click', function() {
                setManualState({steering: Number(button.dataset.steering) / 100.0});
            });
        });

        document.querySelectorAll('[data-mode]').forEach(function(button) {
            button.addEventListener('click', function() {
                setDriveMode(button.dataset.mode);
            });
        });

        document.querySelectorAll('[data-action]').forEach(function(button) {
            button.addEventListener('click', function() {
                const action = button.dataset.action;
                if (action === 'stop') {
                    setManualState({throttle: 0.0});
                } else if (action === 'center') {
                    setManualState({steering: 0.0});
                } else if (action === 'estop') {
                    sendCommand('e');
                } else if (action === 'release') {
                    sendCommand('r');
                } else if (action === 'reset-all') {
                    setManualState({throttle: 0.0, steering: 0.0});
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
                    sendCommand(key);
                }
            } else if (singleKeys.has(key)) {
                sendCommand(key);
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
                sendCommand(key);
            });
        }, 80);

        setInterval(refreshState, 500);

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

        refreshState();
    </script>
</body>
</html>
"""


class WebControlNode(Node):
    def __init__(self):
        super().__init__('web_control_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)
        self.declare_parameter('throttle_step', 0.1)
        self.declare_parameter('steering_step_cmd', 0.2)
        self.declare_parameter('manual_override_mode', 'MANUAL')
        self.declare_parameter('auto_switch_mode_on_manual_input', True)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.steering_step_cmd = float(self.get_parameter('steering_step_cmd').value)
        self.manual_override_mode = str(self.get_parameter('manual_override_mode').value).upper().strip()
        self.auto_switch_mode_on_manual_input = bool(self.get_parameter('auto_switch_mode_on_manual_input').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.estop = False
        self.drive_mode = self.manual_override_mode if self.manual_override_mode in ('MANUAL', 'AI_INTERVENTION', 'AUTONOMOUS') else 'MANUAL'
        self.last_published_estop = None
        self.command_queue = deque()
        self.command_lock = threading.Lock()
        self.last_status = 'ready'

        self.pub_throttle = self.create_publisher(Float32, '/input/manual/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/input/manual/steering', 10)
        self.pub_estop = self.create_publisher(Bool, '/system/estop_cmd', 10)
        self.pub_drive_mode = self.create_publisher(String, '/system/drive_mode_cmd', 10)
        self.pub_autonomy_enable = self.create_publisher(Bool, '/system/autonomy_enable', 10)
        self.sub_estop_state = self.create_subscription(
            Bool, '/vehicle/emergency_stop_state', self.estop_state_callback, 10
        )
        self.sub_drive_mode = self.create_subscription(
            String, '/system/drive_mode', self.drive_mode_callback, 10
        )

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.app = Flask(__name__)
        self.setup_routes()

        self.get_logger().info(f'web_control_node started on http://{self.host}:{self.port}')

    def clamp_throttle(self):
        self.current_throttle = max(-1.0, min(1.0, self.current_throttle))

    def publish_throttle(self):
        msg = Float32()
        msg.data = float(self.current_throttle)
        self.pub_throttle.publish(msg)

    def publish_steering(self):
        msg = Float32()
        msg.data = float(self.current_steering)
        self.pub_steering.publish(msg)

    def publish_estop(self):
        msg = Bool()
        msg.data = bool(self.estop)
        self.pub_estop.publish(msg)
        self.last_published_estop = msg.data

    def publish_drive_mode(self, mode: str):
        msg = String()
        msg.data = str(mode).upper().strip()
        if msg.data not in ('MANUAL', 'AI_INTERVENTION', 'AUTONOMOUS'):
            self.last_status = f'ignored invalid drive mode: {msg.data}'
            return
        self.pub_drive_mode.publish(msg)

        enable = Bool()
        enable.data = msg.data == 'AUTONOMOUS'
        self.pub_autonomy_enable.publish(enable)

        self.drive_mode = msg.data
        self.last_status = f'drive_mode={msg.data}, autonomy_enable={enable.data}'

    def maybe_switch_to_manual_override_mode(self):
        if not self.auto_switch_mode_on_manual_input:
            return
        if self.manual_override_mode not in ('MANUAL', 'AI_INTERVENTION', 'AUTONOMOUS'):
            return
        if self.drive_mode == self.manual_override_mode:
            return
        self.publish_drive_mode(self.manual_override_mode)

    def state_payload(self):
        return {
            'ok': True,
            'throttle': float(self.current_throttle),
            'steering': float(self.current_steering),
            'estop': bool(self.estop),
            'drive_mode': self.drive_mode,
            'status': self.last_status,
        }

    def clamp_steering(self):
        self.current_steering = max(-1.0, min(1.0, self.current_steering))

    def estop_state_callback(self, msg: Bool):
        self.estop = bool(msg.data)
        self.last_status = f'estop_state={self.estop}'

    def drive_mode_callback(self, msg: String):
        self.drive_mode = str(msg.data).upper().strip()

    def handle_key(self, key: str):
        if key == 'w':
            if self.estop:
                self.last_status = 'ignored forward command while estop is active'
                return
            self.maybe_switch_to_manual_override_mode()
            self.current_throttle += self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.last_status = f'throttle increased to {self.current_throttle:.2f}'
            self.get_logger().info(f'web key=w throttle={self.current_throttle:.2f}')

        elif key == 's':
            if self.estop:
                self.last_status = 'ignored reverse command while estop is active'
                return
            self.maybe_switch_to_manual_override_mode()
            self.current_throttle -= self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.last_status = f'throttle decreased to {self.current_throttle:.2f}'
            self.get_logger().info(f'web key=s throttle={self.current_throttle:.2f}')

        elif key == 'x':
            self.maybe_switch_to_manual_override_mode()
            self.current_throttle = 0.0
            self.publish_throttle()
            self.last_status = 'throttle set to 0.00'
            self.get_logger().info('web key=x throttle=0')

        elif key == 'a':
            if self.estop:
                self.last_status = 'ignored left steering while estop is active'
                return
            self.maybe_switch_to_manual_override_mode()
            self.current_steering -= self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.last_status = f'steering moved to {self.current_steering:.2f}'
            self.get_logger().info(f'web key=a steering={self.current_steering:.2f}')

        elif key == 'd':
            if self.estop:
                self.last_status = 'ignored right steering while estop is active'
                return
            self.maybe_switch_to_manual_override_mode()
            self.current_steering += self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.last_status = f'steering moved to {self.current_steering:.2f}'
            self.get_logger().info(f'web key=d steering={self.current_steering:.2f}')

        elif key == 'c':
            if self.estop:
                self.last_status = 'ignored center steering while estop is active'
                return
            self.maybe_switch_to_manual_override_mode()
            self.current_steering = 0.0
            self.publish_steering()
            self.last_status = 'steering centered'
            self.get_logger().info('web key=c steering center')

        elif key == 'e':
            self.estop = True
            self.publish_estop()
            self.current_throttle = 0.0
            self.current_steering = 0.0
            self.publish_throttle()
            self.publish_steering()
            self.last_status = 'emergency stop engaged'
            self.get_logger().info('web key=e estop=True')

        elif key == 'r':
            self.estop = False
            self.publish_estop()
            self.last_status = 'emergency stop released'
            self.get_logger().info('web key=r estop=False')

    def apply_state_patch(self, data):
        throttle = data.get('throttle')
        steering = data.get('steering')

        if throttle is not None:
            if self.estop:
                self.current_throttle = 0.0
                self.last_status = 'ignored throttle set while estop is active'
            else:
                self.maybe_switch_to_manual_override_mode()
                self.current_throttle = float(throttle)
                self.clamp_throttle()
                self.publish_throttle()
                self.last_status = f'throttle set to {self.current_throttle:.2f}'

        if steering is not None:
            if self.estop:
                self.current_steering = 0.0
                self.last_status = 'ignored steering set while estop is active'
            else:
                self.maybe_switch_to_manual_override_mode()
                self.current_steering = float(steering)
                self.clamp_steering()
                self.publish_steering()
                self.last_status = f'steering set to {self.current_steering:.2f}'

    def setup_routes(self):
        @self.app.route('/', methods=['GET'])
        def index():
            return render_template_string(HTML_PAGE)

        @self.app.route('/api/state', methods=['GET'])
        def api_state():
            return jsonify(self.state_payload())

        @self.app.route('/api/command', methods=['POST'])
        def api_command():
            data = request.get_json(silent=True) or {}
            key = str(data.get('key', '')).lower().strip()
            if key:
                with self.command_lock:
                    self.command_queue.append(key)
            return jsonify(self.state_payload())

        @self.app.route('/api/set_state', methods=['POST'])
        def api_set_state():
            data = request.get_json(silent=True) or {}
            with self.command_lock:
                self.apply_state_patch(data)
            return jsonify(self.state_payload())

        @self.app.route('/api/set_drive_mode', methods=['POST'])
        def api_set_drive_mode():
            data = request.get_json(silent=True) or {}
            mode = str(data.get('mode', 'MANUAL')).upper().strip()
            self.publish_drive_mode(mode)
            return jsonify(self.state_payload())

    def run_flask(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)

    def timer_callback(self):
        with self.command_lock:
            while self.command_queue:
                self.handle_key(self.command_queue.popleft())
        self.publish_throttle()
        self.publish_steering()
        if self.last_published_estop != self.estop:
            self.publish_estop()


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()

    flask_thread = threading.Thread(target=node.run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Float32()
        stop_msg.data = 0.0
        node.pub_throttle.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
