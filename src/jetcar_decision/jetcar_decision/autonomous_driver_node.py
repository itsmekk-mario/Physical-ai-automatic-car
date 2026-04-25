import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class AutonomousDriverNode(Node):
    def __init__(self):
        super().__init__('autonomous_driver_node')

        self.declare_parameter('cruise_throttle', 0.25)
        self.declare_parameter('hazard_stop_distance_m', 0.8)
        self.declare_parameter('lane_follow_gain', 1.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('enabled_on_start', False)
        self.declare_parameter('startup_hold_sec', 2.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('max_throttle_step', 0.02)
        self.declare_parameter('autonomy_level', 2)
        self.declare_parameter('require_lane', False)
        self.declare_parameter('lane_follow_min_level', 2)
        self.declare_parameter('use_detection_hazard', False)

        self.cruise_throttle = float(self.get_parameter('cruise_throttle').value)
        self.hazard_stop_distance_m = float(self.get_parameter('hazard_stop_distance_m').value)
        self.lane_follow_gain = float(self.get_parameter('lane_follow_gain').value)
        self.enabled = bool(self.get_parameter('enabled_on_start').value)
        self.startup_hold_sec = float(self.get_parameter('startup_hold_sec').value)
        self.input_timeout_sec = float(self.get_parameter('input_timeout_sec').value)
        self.max_throttle_step = abs(float(self.get_parameter('max_throttle_step').value))
        self.autonomy_level = max(2, int(self.get_parameter('autonomy_level').value))
        self.require_lane = bool(self.get_parameter('require_lane').value)
        self.lane_follow_min_level = max(2, int(self.get_parameter('lane_follow_min_level').value))
        self.use_detection_hazard = bool(self.get_parameter('use_detection_hazard').value)

        self.depth_ready = False
        self.lane_ready = False
        self.min_distance_m = 99.0
        self.lane_suggested_steering = 0.0
        self.detection_hazard = False
        self.current_throttle_cmd = 0.0
        self.start_time = self.get_clock().now()
        self.last_depth_time = None
        self.last_lane_time = None

        self.pub_throttle = self.create_publisher(Float32, '/input/autonomy/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/input/autonomy/steering', 10)
        self.pub_status = self.create_publisher(String, '/autonomy/status', 10)

        self.create_subscription(Bool, '/perception/depth/ready', self.depth_ready_cb, 10)
        self.create_subscription(Bool, '/perception/lane/ready', self.lane_ready_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Float32, '/perception/lane/suggested_steering', self.suggested_steering_cb, 10)
        self.create_subscription(Bool, '/perception/detections/hazard', self.detection_hazard_cb, 10)
        self.create_subscription(Bool, '/system/autonomy_enable', self.enable_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'autonomous_driver_node started | '
            f'cruise_throttle={self.cruise_throttle:.2f}, stop_distance={self.hazard_stop_distance_m:.2f}, '
            f'level={self.autonomy_level}, require_lane={self.require_lane}, enabled={self.enabled}'
        )

    def depth_ready_cb(self, msg: Bool):
        self.depth_ready = bool(msg.data)
        self.last_depth_time = self.get_clock().now()

    def lane_ready_cb(self, msg: Bool):
        self.lane_ready = bool(msg.data)
        self.last_lane_time = self.get_clock().now()

    def min_distance_cb(self, msg: Float32):
        self.min_distance_m = float(msg.data)

    def suggested_steering_cb(self, msg: Float32):
        self.lane_suggested_steering = float(msg.data)

    def detection_hazard_cb(self, msg: Bool):
        self.detection_hazard = bool(msg.data)

    def enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.current_throttle_cmd = 0.0
        self.get_logger().warn(f'autonomy_enable={self.enabled}')

    def seconds_since(self, stamp):
        if stamp is None:
            return 999.0
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def ramp_throttle(self, target: float) -> float:
        target = max(0.0, min(self.cruise_throttle, float(target)))
        delta = target - self.current_throttle_cmd
        if abs(delta) <= self.max_throttle_step:
            self.current_throttle_cmd = target
        elif delta > 0.0:
            self.current_throttle_cmd += self.max_throttle_step
        else:
            self.current_throttle_cmd -= self.max_throttle_step
        return self.current_throttle_cmd

    def timer_callback(self):
        uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        depth_fresh = self.seconds_since(self.last_depth_time) <= self.input_timeout_sec
        lane_fresh = self.seconds_since(self.last_lane_time) <= self.input_timeout_sec
        lane_available = lane_fresh and self.lane_ready
        lane_follow_enabled = self.autonomy_level >= self.lane_follow_min_level
        lane_required = self.require_lane and lane_follow_enabled
        inputs_fresh = depth_fresh and (lane_available if lane_required else True)
        ready = self.enabled and uptime >= self.startup_hold_sec and inputs_fresh and self.depth_ready
        hazard = self.min_distance_m <= self.hazard_stop_distance_m
        if self.use_detection_hazard:
            hazard = hazard or self.detection_hazard

        throttle_cmd = 0.0
        steering_cmd = 0.0
        if ready and not hazard:
            throttle_cmd = self.ramp_throttle(self.cruise_throttle)
            if lane_follow_enabled and lane_available:
                steering_cmd = max(-1.0, min(1.0, self.lane_follow_gain * self.lane_suggested_steering))
        else:
            self.current_throttle_cmd = 0.0

        throttle = Float32()
        throttle.data = throttle_cmd
        self.pub_throttle.publish(throttle)

        steering = Float32()
        steering.data = steering_cmd
        self.pub_steering.publish(steering)

        status = String()
        status.data = (
            f'level={self.autonomy_level}, enabled={self.enabled}, ready={ready}, fresh={inputs_fresh}, '
            f'depth_fresh={depth_fresh}, lane_follow={lane_follow_enabled}, '
            f'lane_required={lane_required}, lane_available={lane_available}, hazard={hazard}, '
            f'throttle={throttle_cmd:.2f}, steering={steering_cmd:.2f}, min_distance_m={self.min_distance_m:.2f}'
        )
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
