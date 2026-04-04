import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool


HELP_TEXT = """
========================================
JetCar Keyboard Control
----------------------------------------
w : throttle +10%
s : throttle -10%
a : steer left
d : steer right
x : throttle = 0
e : emergency stop toggle
q : quit
========================================
"""


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        self.declare_parameter('throttle_step', 0.1)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('steering_step_cmd', 1)

        self.throttle_step = float(self.get_parameter('throttle_step').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.steering_step_cmd = int(self.get_parameter('steering_step_cmd').value)

        self.pub_throttle = self.create_publisher(Float32, '/vehicle/throttle', 10)
        self.pub_steering = self.create_publisher(Int32, '/vehicle/steering_step', 10)
        self.pub_estop = self.create_publisher(Bool, '/vehicle/emergency_stop', 10)

        self.current_throttle = 0.0
        self.estop = False

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('keyboard_control_node started')
        print(HELP_TEXT)
        self.print_status()

    def print_status(self):
        print(f'[STATUS] throttle={self.current_throttle:.2f}, estop={"ON" if self.estop else "OFF"}')

    def publish_throttle(self):
        msg = Float32()
        msg.data = float(self.current_throttle)
        self.pub_throttle.publish(msg)

    def publish_steering_step(self, step: int):
        msg = Int32()
        msg.data = int(step)
        self.pub_steering.publish(msg)

    def publish_estop(self):
        msg = Bool()
        msg.data = bool(self.estop)
        self.pub_estop.publish(msg)

    def clamp_throttle(self):
        self.current_throttle = max(-1.0, min(1.0, self.current_throttle))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def handle_key(self, key: str):
        if key == 'w':
            if self.estop:
                print('[INFO] E-stop is ON. Release it with e first.')
                return
            self.current_throttle += self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            print('[KEY] w -> throttle +10%')
            self.print_status()

        elif key == 's':
            if self.estop:
                print('[INFO] E-stop is ON. Release it with e first.')
                return
            self.current_throttle -= self.throttle_step
            self.clamp_throttle()
            self.publish_throttle()
            print('[KEY] s -> throttle -10%')
            self.print_status()

        elif key == 'x':
            self.current_throttle = 0.0
            self.publish_throttle()
            print('[KEY] x -> throttle = 0')
            self.print_status()

        elif key == 'a':
            if self.estop:
                print('[INFO] E-stop is ON. Steering blocked.')
                return
            self.publish_steering_step(-self.steering_step_cmd)
            print('[KEY] a -> steer left')

        elif key == 'd':
            if self.estop:
                print('[INFO] E-stop is ON. Steering blocked.')
                return
            self.publish_steering_step(self.steering_step_cmd)
            print('[KEY] d -> steer right')

        elif key == 'e':
            self.estop = not self.estop
            self.publish_estop()

            if self.estop:
                self.current_throttle = 0.0
                self.publish_throttle()
                print('[KEY] e -> EMERGENCY STOP ON')
            else:
                print('[KEY] e -> EMERGENCY STOP OFF')

            self.print_status()

        elif key == 'q':
            print('[KEY] q -> quit')
            raise KeyboardInterrupt

    def timer_callback(self):
        key = self.get_key()
        if key:
            self.handle_key(key)

        self.publish_throttle()

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_pub = node.create_publisher(Float32, '/vehicle/throttle', 10)
        stop_msg = Float32()
        stop_msg.data = 0.0
        for _ in range(3):
            stop_pub.publish(stop_msg)
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
