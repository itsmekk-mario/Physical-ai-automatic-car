import threading
from collections import deque

from flask import Flask, jsonify, render_template_string, request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


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
            --bg: #0e1719;
            --panel: rgba(13, 29, 33, 0.88);
            --panel-strong: #173339;
            --line: rgba(165, 230, 214, 0.16);
            --text: #eef7f3;
            --muted: #9ab7b0;
            --accent: #4dd7a8;
            --accent-strong: #1bb980;
            --danger: #f15c52;
            --warn: #efb74b;
            --shadow: 0 24px 60px rgba(0, 0, 0, 0.35);
        }
        body {
            font-family: "Segoe UI", "Noto Sans KR", sans-serif;
            background:
                radial-gradient(circle at top left, rgba(77, 215, 168, 0.18), transparent 28%),
                radial-gradient(circle at top right, rgba(241, 92, 82, 0.14), transparent 22%),
                linear-gradient(180deg, #071012 0%, #0d191c 100%);
            color: var(--text);
            margin: 0;
            min-height: 100vh;
        }
        .shell {
            max-width: 1120px;
            margin: 0 auto;
            padding: 24px 18px 40px;
        }
        .hero {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 16px;
            margin-bottom: 18px;
        }
        .hero h1 {
            margin: 0;
            font-size: clamp(2rem, 5vw, 3.3rem);
            letter-spacing: 0.04em;
            font-weight: 800;
        }
        .hero p {
            margin: 6px 0 0;
            color: var(--muted);
            max-width: 720px;
        }
        .badge {
            padding: 10px 14px;
            border-radius: 999px;
            background: rgba(77, 215, 168, 0.12);
            border: 1px solid rgba(77, 215, 168, 0.3);
            color: #c8ffef;
            font-size: 0.95rem;
            white-space: nowrap;
        }
        .layout {
            display: grid;
            grid-template-columns: 1.2fr 0.9fr;
            gap: 18px;
        }
        .panel {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 24px;
            box-shadow: var(--shadow);
            backdrop-filter: blur(18px);
        }
        .panel-inner {
            padding: 20px;
        }
        .meter-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 12px;
            margin-bottom: 16px;
        }
        .stat {
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.06);
            border-radius: 18px;
            padding: 14px;
        }
        .stat label {
            display: block;
            color: var(--muted);
            font-size: 0.84rem;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 0.08em;
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
            background: rgba(255, 255, 255, 0.025);
            border-radius: 20px;
            border: 1px solid rgba(255, 255, 255, 0.05);
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
            letter-spacing: 0.04em;
            text-transform: uppercase;
        }
        .control-head span {
            color: var(--muted);
            font-size: 0.94rem;
        }
        input[type="range"] {
            width: 100%;
            accent-color: var(--accent);
        }
        .preset-row {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 10px;
            margin-top: 12px;
        }
        .drive-pad {
            display: grid;
            grid-template-columns: repeat(3, minmax(72px, 1fr));
            gap: 10px;
        }
        button {
            min-height: 66px;
            font-size: 1rem;
            font-weight: 700;
            border: 1px solid rgba(255, 255, 255, 0.08);
            border-radius: 18px;
            cursor: pointer;
            background: linear-gradient(180deg, rgba(29, 54, 58, 0.95), rgba(14, 29, 33, 0.95));
            color: var(--text);
            transition: transform 120ms ease, border-color 120ms ease, background 120ms ease;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }
        button:hover {
            border-color: rgba(77, 215, 168, 0.32);
        }
        button:active,
        button.active {
            transform: translateY(1px) scale(0.99);
            background: linear-gradient(180deg, rgba(42, 96, 86, 0.98), rgba(17, 58, 52, 0.98));
        }
        button.subtle {
            min-height: 52px;
            font-size: 0.95rem;
        }
        button.stop {
            background: linear-gradient(180deg, rgba(74, 82, 89, 0.98), rgba(43, 51, 56, 0.98));
        }
        button.danger {
            background: linear-gradient(180deg, rgba(210, 82, 71, 0.98), rgba(123, 35, 35, 0.98));
            border-color: rgba(255, 178, 168, 0.35);
        }
        button.release {
            background: linear-gradient(180deg, rgba(77, 215, 168, 0.95), rgba(27, 137, 102, 0.98));
            color: #03100b;
        }
        .footer-note {
            margin-top: 12px;
            color: var(--muted);
            font-size: 0.9rem;
            line-height: 1.5;
        }
        .telemetry {
            display: grid;
            gap: 12px;
        }
        .telemetry-box {
            padding: 16px;
            border-radius: 20px;
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.06);
        }
        .telemetry-box h3 {
            margin: 0 0 10px;
            font-size: 0.92rem;
            text-transform: uppercase;
            letter-spacing: 0.08em;
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
            .preset-row {
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
            .preset-row {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="shell">
        <div class="hero">
            <div>
                <h1>JetCar Control Deck</h1>
                <p>브라우저만으로 스로틀, 조향, 정지, E-stop을 직접 제어합니다. 모바일 터치와 데스크탑 마우스를 모두 지원합니다.</p>
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
                    </div>

                    <div class="controls">
                        <div class="control-card">
                            <div class="control-head">
                                <h2>Throttle Slider</h2>
                                <span id="throttlePercent">0%</span>
                            </div>
                            <input id="throttleSlider" type="range" min="-100" max="100" value="0" step="1">
                            <div class="preset-row">
                                <button class="subtle" data-throttle="-40">REV 40%</button>
                                <button class="subtle" data-throttle="-20">REV 20%</button>
                                <button class="subtle stop" data-action="stop">STOP</button>
                                <button class="subtle" data-throttle="20">FWD 20%</button>
                                <button class="subtle" data-throttle="40">FWD 40%</button>
                            </div>
                        </div>

                        <div class="control-card">
                            <div class="control-head">
                                <h2>Steering Slider</h2>
                                <span id="steeringPercent">0%</span>
                            </div>
                            <input id="steeringSlider" type="range" min="-100" max="100" value="0" step="1">
                            <div class="preset-row">
                                <button class="subtle" data-steering="-100">HARD L</button>
                                <button class="subtle" data-steering="-40">LEFT</button>
                                <button class="subtle stop" data-action="center">CENTER</button>
                                <button class="subtle" data-steering="40">RIGHT</button>
                                <button class="subtle" data-steering="100">HARD R</button>
                            </div>
                        </div>

                        <div class="control-card">
                            <div class="control-head">
                                <h2>Touch Pad</h2>
                                <span>Press and hold</span>
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
                                <span>Immediate actions</span>
                            </div>
                            <div class="preset-row">
                                <button class="danger" data-action="estop">E-STOP</button>
                                <button class="release" data-action="release">RELEASE</button>
                                <button class="stop" data-action="reset-all">RESET ALL</button>
                            </div>
                            <div class="footer-note">
                                `RESET ALL`은 스로틀 0, 조향 중앙으로 복귀합니다. E-stop이 켜져 있으면 먼저 `RELEASE`를 눌러야 주행 입력이 다시 반영됩니다.
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
                    <div class="telemetry-box">
                        <h3>Quick Guide</h3>
                        <pre>1. 브라우저에서 슬라이더 또는 버튼으로 조작
2. `/input/manual/*` 토픽으로 명령 발행
3. control_mux가 `/vehicle/*`로 중재
4. vehicle_hw가 실제 PWM 제어 수행</pre>
                    </div>
                    <div class="telemetry-box">
                        <h3>Keyboard Fallback</h3>
                        <pre>W/S throttle
A/D steering
X brake
C center
E estop
R release</pre>
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
            throttleSlider.value = Math.round(Number(state.throttle) * 100);
            steeringSlider.value = Math.round(Number(state.steering) * 100);
            throttlePercent.textContent = `${Math.round(Number(state.throttle) * 100)}%`;
            steeringPercent.textContent = `${Math.round(Number(state.steering) * 100)}%`;
            statusText.textContent = state.status;
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
        self.declare_parameter('publish_rate_hz', 20.0)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.steering_step_cmd = float(self.get_parameter('steering_step_cmd').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.current_throttle = 0.0
        self.current_steering = 0.0
        self.estop = False
        self.command_queue = deque()
        self.command_lock = threading.Lock()
        self.last_status = 'ready'

        self.pub_throttle = self.create_publisher(Float32, '/input/manual/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/input/manual/steering', 10)
        self.pub_estop = self.create_publisher(Bool, '/system/estop_cmd', 10)
        self.sub_estop_state = self.create_subscription(
            Bool, '/vehicle/emergency_stop_state', self.estop_state_callback, 10
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

    def state_payload(self):
        return {
            'ok': True,
            'throttle': float(self.current_throttle),
            'steering': float(self.current_steering),
            'estop': bool(self.estop),
            'status': self.last_status,
        }

    def clamp_steering(self):
        self.current_steering = max(-1.0, min(1.0, self.current_steering))

    def estop_state_callback(self, msg: Bool):
        self.estop = bool(msg.data)
        self.last_status = f'estop_state={self.estop}'

    def handle_key(self, key: str):
        if key == 'w':
            if self.estop:
                self.last_status = 'ignored forward command while estop is active'
                return
            self.current_throttle += self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.last_status = f'throttle increased to {self.current_throttle:.2f}'
            self.get_logger().info(f'web key=w throttle={self.current_throttle:.2f}')

        elif key == 's':
            if self.estop:
                self.last_status = 'ignored reverse command while estop is active'
                return
            self.current_throttle -= self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.last_status = f'throttle decreased to {self.current_throttle:.2f}'
            self.get_logger().info(f'web key=s throttle={self.current_throttle:.2f}')

        elif key == 'x':
            self.current_throttle = 0.0
            self.publish_throttle()
            self.last_status = 'throttle set to 0.00'
            self.get_logger().info('web key=x throttle=0')

        elif key == 'a':
            if self.estop:
                self.last_status = 'ignored left steering while estop is active'
                return
            self.current_steering -= self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.last_status = f'steering moved to {self.current_steering:.2f}'
            self.get_logger().info(f'web key=a steering={self.current_steering:.2f}')

        elif key == 'd':
            if self.estop:
                self.last_status = 'ignored right steering while estop is active'
                return
            self.current_steering += self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.last_status = f'steering moved to {self.current_steering:.2f}'
            self.get_logger().info(f'web key=d steering={self.current_steering:.2f}')

        elif key == 'c':
            if self.estop:
                self.last_status = 'ignored center steering while estop is active'
                return
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
                self.current_throttle = float(throttle)
                self.clamp_throttle()
                self.publish_throttle()
                self.last_status = f'throttle set to {self.current_throttle:.2f}'

        if steering is not None:
            if self.estop:
                self.current_steering = 0.0
                self.last_status = 'ignored steering set while estop is active'
            else:
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

    def run_flask(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False)

    def timer_callback(self):
        with self.command_lock:
            while self.command_queue:
                self.handle_key(self.command_queue.popleft())
        self.publish_throttle()
        self.publish_steering()


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
