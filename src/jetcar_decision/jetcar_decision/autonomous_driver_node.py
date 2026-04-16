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

        self.cruise_throttle = float(self.get_parameter('cruise_throttle').value)
        self.hazard_stop_distance_m = float(self.get_parameter('hazard_stop_distance_m').value)
        self.lane_follow_gain = float(self.get_parameter('lane_follow_gain').value)

        self.depth_ready = False
        self.lane_ready = False
        self.min_distance_m = 99.0
        self.lane_suggested_steering = 0.0
        self.detection_hazard = False

        self.pub_throttle = self.create_publisher(Float32, '/input/autonomy/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/input/autonomy/steering', 10)
        self.pub_status = self.create_publisher(String, '/autonomy/status', 10)

        self.create_subscription(Bool, '/perception/depth/ready', self.depth_ready_cb, 10)
        self.create_subscription(Bool, '/perception/lane/ready', self.lane_ready_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Float32, '/perception/lane/suggested_steering', self.suggested_steering_cb, 10)
        self.create_subscription(Bool, '/perception/detections/hazard', self.detection_hazard_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'autonomous_driver_node started | '
            f'cruise_throttle={self.cruise_throttle:.2f}, stop_distance={self.hazard_stop_distance_m:.2f}'
        )

    def depth_ready_cb(self, msg: Bool):
        self.depth_ready = bool(msg.data)

    def lane_ready_cb(self, msg: Bool):
        self.lane_ready = bool(msg.data)

    def min_distance_cb(self, msg: Float32):
        self.min_distance_m = float(msg.data)

    def suggested_steering_cb(self, msg: Float32):
        self.lane_suggested_steering = float(msg.data)

    def detection_hazard_cb(self, msg: Bool):
        self.detection_hazard = bool(msg.data)

    def timer_callback(self):
        ready = self.depth_ready and self.lane_ready
        hazard = self.detection_hazard or self.min_distance_m <= self.hazard_stop_distance_m

        throttle_cmd = 0.0
        steering_cmd = 0.0
        if ready and not hazard:
            throttle_cmd = self.cruise_throttle
            steering_cmd = max(-1.0, min(1.0, self.lane_follow_gain * self.lane_suggested_steering))

        throttle = Float32()
        throttle.data = throttle_cmd
        self.pub_throttle.publish(throttle)

        steering = Float32()
        steering.data = steering_cmd
        self.pub_steering.publish(steering)

        status = String()
        status.data = (
            f'ready={ready}, hazard={hazard}, throttle={throttle_cmd:.2f}, '
            f'steering={steering_cmd:.2f}, min_distance_m={self.min_distance_m:.2f}'
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
