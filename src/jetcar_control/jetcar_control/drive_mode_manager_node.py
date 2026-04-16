import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


VALID_MODES = {'MANUAL', 'AI_INTERVENTION', 'AUTONOMOUS'}


class DriveModeManagerNode(Node):
    def __init__(self):
        super().__init__('drive_mode_manager_node')

        self.declare_parameter('default_mode', 'MANUAL')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.current_mode = str(self.get_parameter('default_mode').value).upper()
        if self.current_mode not in VALID_MODES:
            raise ValueError(f'Invalid default_mode: {self.current_mode}')

        self.estop_active = False

        self.pub_mode = self.create_publisher(String, '/system/drive_mode', 10)
        self.pub_summary = self.create_publisher(String, '/system/control_summary', 10)

        self.sub_mode_cmd = self.create_subscription(
            String, '/system/drive_mode_cmd', self.mode_cmd_callback, 10
        )
        self.sub_estop_state = self.create_subscription(
            Bool, '/vehicle/emergency_stop_state', self.estop_state_callback, 10
        )

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'drive_mode_manager_node started | mode={self.current_mode}')

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.pub_mode.publish(msg)

        summary = String()
        summary.data = f'mode={self.current_mode}, estop={"ON" if self.estop_active else "OFF"}'
        self.pub_summary.publish(summary)

    def mode_cmd_callback(self, msg: String):
        requested_mode = str(msg.data).upper().strip()
        if requested_mode not in VALID_MODES:
            self.get_logger().warn(f'Ignoring invalid drive mode: {requested_mode}')
            return
        if requested_mode != self.current_mode:
            self.current_mode = requested_mode
            self.get_logger().info(f'drive mode -> {self.current_mode}')
            self.publish_mode()

    def estop_state_callback(self, msg: Bool):
        self.estop_active = bool(msg.data)

    def timer_callback(self):
        self.publish_mode()


def main(args=None):
    rclpy.init(args=args)
    node = DriveModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
