import threading
from collections import deque

from flask import Flask, request, jsonify, render_template_string

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>JetCar Web Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #111;
            color: #eee;
            text-align: center;
            margin: 0;
            padding: 30px;
        }
        h1 {
            margin-bottom: 10px;
        }
        .status {
            margin: 20px auto;
            padding: 12px;
            width: 320px;
            background: #222;
            border-radius: 10px;
        }
        .grid {
            display: grid;
            grid-template-columns: 100px 100px 100px;
            gap: 10px;
            justify-content: center;
            margin-top: 20px;
        }
        button {
            height: 70px;
            font-size: 24px;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            background: #2a2a2a;
            color: white;
        }
        button:hover {
            background: #444;
        }
        .danger {
            background: #8b1e1e;
        }
        .danger:hover {
            background: #b52a2a;
        }
        .stop {
            background: #555;
        }
    </style>
</head>
<body>
    <h1>JetCar Web Control</h1>
    <div class="status">
        <div>W/S: throttle ±10%</div>
        <div>A/D: steering left/right</div>
        <div>C: steering center</div>
        <div>X: throttle 0</div>
        <div>E: emergency stop ON</div>
        <div>R: emergency stop OFF</div>
    </div>

    <div class="grid">
        <div></div>
        <button onclick="sendCommand('w')">W</button>
        <div></div>

        <button onclick="sendCommand('a')">A</button>
        <button class="stop" onclick="sendCommand('x')">X</button>
        <button onclick="sendCommand('d')">D</button>

        <div></div>
        <button onclick="sendCommand('s')">S</button>
        <div></div>
    </div>

    <div style="margin-top:20px;">
        <button class="danger" onclick="sendCommand('e')">E-STOP</button>
        <button onclick="sendCommand('r')">RELEASE</button>
        <button onclick="sendCommand('c')">CENTER</button>
    </div>

    <script>
        async function sendCommand(key) {
            await fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ key: key })
            });
        }

        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            if (['w', 'a', 's', 'd', 'x', 'e', 'r', 'c'].includes(key)) {
                sendCommand(key);
            }
        });
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
        self.declare_parameter('steering_step_cmd', 0.1)
        self.declare_parameter('publish_rate_hz', 10.0)

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

        self.pub_throttle = self.create_publisher(Float32, '/vehicle/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/vehicle/steering', 10)
        self.pub_estop = self.create_publisher(Bool, '/vehicle/emergency_stop', 10)
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

    def clamp_steering(self):
        self.current_steering = max(-1.0, min(1.0, self.current_steering))

    def estop_state_callback(self, msg: Bool):
        self.estop = bool(msg.data)

    def handle_key(self, key: str):
        if key == 'w':
            if self.estop:
                return
            self.current_throttle += self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.get_logger().info(f'web key=w throttle={self.current_throttle:.2f}')

        elif key == 's':
            if self.estop:
                return
            self.current_throttle -= self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            self.get_logger().info(f'web key=s throttle={self.current_throttle:.2f}')

        elif key == 'x':
            self.current_throttle = 0.0
            self.publish_throttle()
            self.get_logger().info('web key=x throttle=0')

        elif key == 'a':
            if self.estop:
                return
            self.current_steering -= self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.get_logger().info(f'web key=a steering={self.current_steering:.2f}')

        elif key == 'd':
            if self.estop:
                return
            self.current_steering += self.steering_step_cmd
            self.clamp_steering()
            self.publish_steering()
            self.get_logger().info(f'web key=d steering={self.current_steering:.2f}')

        elif key == 'c':
            if self.estop:
                return
            self.current_steering = 0.0
            self.publish_steering()
            self.get_logger().info('web key=c steering center')

        elif key == 'e':
            self.estop = True
            self.publish_estop()
            self.current_throttle = 0.0
            self.current_steering = 0.0
            self.publish_throttle()
            self.publish_steering()
            self.get_logger().info('web key=e estop=True')

        elif key == 'r':
            self.estop = False
            self.publish_estop()
            self.get_logger().info('web key=r estop=False')

    def setup_routes(self):
        @self.app.route('/', methods=['GET'])
        def index():
            return render_template_string(HTML_PAGE)

        @self.app.route('/api/command', methods=['POST'])
        def api_command():
            data = request.get_json(silent=True) or {}
            key = str(data.get('key', '')).lower().strip()
            if key:
                with self.command_lock:
                    self.command_queue.append(key)
            return jsonify({
                'ok': True,
                'key': key,
                'throttle': self.current_throttle,
                'steering': self.current_steering,
                'estop': self.estop
            })

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
