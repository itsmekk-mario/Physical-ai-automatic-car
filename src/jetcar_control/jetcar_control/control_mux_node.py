import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String


class ControlMuxNode(Node):
    def __init__(self):
        super().__init__('control_mux_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('default_mode', 'MANUAL')
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('manual_max_throttle', 0.35)
        self.declare_parameter('ai_max_throttle', 0.15)
        self.declare_parameter('autonomy_max_throttle', 0.10)

        self.drive_mode = str(self.get_parameter('default_mode').value).upper()
        self.input_timeout_sec = float(self.get_parameter('input_timeout_sec').value)
        self.manual_max_throttle = abs(float(self.get_parameter('manual_max_throttle').value))
        self.ai_max_throttle = abs(float(self.get_parameter('ai_max_throttle').value))
        self.autonomy_max_throttle = abs(float(self.get_parameter('autonomy_max_throttle').value))
        self.safety_override_active = False
        self.estop_requested = False
        self.estop_state = False

        self.manual_throttle = 0.0
        self.manual_steering = 0.0
        self.ai_throttle = 0.0
        self.ai_steering = 0.0
        self.autonomy_throttle = 0.0
        self.autonomy_steering = 0.0
        now = self.get_clock().now()
        self.manual_last_time = now
        self.ai_last_time = now
        self.autonomy_last_time = now

        self.pub_throttle = self.create_publisher(Float32, '/vehicle/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/vehicle/steering', 10)
        self.pub_estop = self.create_publisher(Bool, '/vehicle/emergency_stop', 10)
        self.pub_selected = self.create_publisher(String, '/system/selected_control_source', 10)

        self.create_subscription(Float32, '/input/manual/throttle', self.manual_throttle_cb, 10)
        self.create_subscription(Float32, '/input/manual/steering', self.manual_steering_cb, 10)
        self.create_subscription(Float32, '/input/ai/throttle', self.ai_throttle_cb, 10)
        self.create_subscription(Float32, '/input/ai/steering', self.ai_steering_cb, 10)
        self.create_subscription(Float32, '/input/autonomy/throttle', self.autonomy_throttle_cb, 10)
        self.create_subscription(Float32, '/input/autonomy/steering', self.autonomy_steering_cb, 10)
        self.create_subscription(String, '/system/drive_mode', self.drive_mode_cb, 10)
        self.create_subscription(Bool, '/system/safety_override_active', self.safety_override_cb, 10)
        self.create_subscription(Bool, '/system/estop_cmd', self.estop_cmd_cb, 10)
        self.create_subscription(Bool, '/vehicle/emergency_stop_state', self.estop_state_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'control_mux_node started | mode={self.drive_mode}')

    def clamp(self, value: float, limit: float = 1.0) -> float:
        limit = max(0.0, min(1.0, float(limit)))
        return max(-limit, min(limit, float(value)))

    def manual_throttle_cb(self, msg: Float32):
        self.manual_throttle = self.clamp(msg.data, self.manual_max_throttle)
        self.manual_last_time = self.get_clock().now()

    def manual_steering_cb(self, msg: Float32):
        self.manual_steering = self.clamp(msg.data)
        self.manual_last_time = self.get_clock().now()

    def ai_throttle_cb(self, msg: Float32):
        self.ai_throttle = self.clamp(msg.data, self.ai_max_throttle)
        self.ai_last_time = self.get_clock().now()

    def ai_steering_cb(self, msg: Float32):
        self.ai_steering = self.clamp(msg.data)
        self.ai_last_time = self.get_clock().now()

    def autonomy_throttle_cb(self, msg: Float32):
        self.autonomy_throttle = self.clamp(msg.data, self.autonomy_max_throttle)
        self.autonomy_last_time = self.get_clock().now()

    def autonomy_steering_cb(self, msg: Float32):
        self.autonomy_steering = self.clamp(msg.data)
        self.autonomy_last_time = self.get_clock().now()

    def drive_mode_cb(self, msg: String):
        self.drive_mode = str(msg.data).upper().strip()

    def safety_override_cb(self, msg: Bool):
        self.safety_override_active = bool(msg.data)

    def estop_cmd_cb(self, msg: Bool):
        self.estop_requested = bool(msg.data)

    def estop_state_cb(self, msg: Bool):
        self.estop_state = bool(msg.data)

    def source_is_fresh(self, stamp):
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= self.input_timeout_sec

    def clamp_source(self, throttle, steering, max_throttle):
        return self.clamp(throttle, max_throttle), self.clamp(steering)

    def select_source(self):
        if self.estop_requested or self.estop_state:
            return 'ESTOP', 0.0, 0.0
        if self.safety_override_active:
            if self.source_is_fresh(self.ai_last_time):
                throttle, steering = self.clamp_source(self.ai_throttle, self.ai_steering, self.ai_max_throttle)
                return 'AI_OVERRIDE', throttle, steering
            return 'AI_OVERRIDE_STALE', 0.0, 0.0
        if self.drive_mode == 'AUTONOMOUS':
            if self.source_is_fresh(self.autonomy_last_time):
                throttle, steering = self.clamp_source(
                    self.autonomy_throttle, self.autonomy_steering, self.autonomy_max_throttle
                )
                return 'AUTONOMOUS', throttle, steering
            return 'AUTONOMOUS_STALE', 0.0, 0.0
        if self.drive_mode == 'AI_INTERVENTION':
            if self.source_is_fresh(self.manual_last_time):
                throttle, steering = self.clamp_source(self.manual_throttle, self.manual_steering, self.manual_max_throttle)
                return 'MANUAL', throttle, steering
            return 'MANUAL_STALE', 0.0, 0.0
        if self.source_is_fresh(self.manual_last_time):
            throttle, steering = self.clamp_source(self.manual_throttle, self.manual_steering, self.manual_max_throttle)
            return 'MANUAL', throttle, steering
        return 'MANUAL_STALE', 0.0, 0.0

    def timer_callback(self):
        source, throttle, steering = self.select_source()

        estop_msg = Bool()
        estop_msg.data = self.estop_requested
        self.pub_estop.publish(estop_msg)

        throttle_msg = Float32()
        steering_msg = Float32()
        throttle_msg.data = 0.0 if source == 'ESTOP' else float(throttle)
        steering_msg.data = 0.0 if source == 'ESTOP' else float(steering)
        self.pub_throttle.publish(throttle_msg)
        self.pub_steering.publish(steering_msg)

        selected_msg = String()
        selected_msg.data = source
        self.pub_selected.publish(selected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
