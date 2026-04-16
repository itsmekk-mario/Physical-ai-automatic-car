import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('fx_px', 700.0)
        self.declare_parameter('disparity_min_px', 2.0)
        self.declare_parameter('disparity_nominal_px', 42.0)
        self.declare_parameter('safe_stop_distance_m', 0.8)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.fx_px = float(self.get_parameter('fx_px').value)
        self.disparity_min_px = float(self.get_parameter('disparity_min_px').value)
        self.disparity_nominal_px = float(self.get_parameter('disparity_nominal_px').value)
        self.safe_stop_distance_m = float(self.get_parameter('safe_stop_distance_m').value)

        self.rectified_ready = False
        self.camera_ready = False

        self.ready_pub = self.create_publisher(Bool, '/perception/depth/ready', 10)
        self.distance_pub = self.create_publisher(Float32, '/perception/depth/min_distance_m', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/depth/hazard_close', 10)
        self.status_pub = self.create_publisher(String, '/perception/depth/status', 10)

        self.create_subscription(Bool, '/sensors/stereo/ready', self.camera_ready_cb, 10)
        self.create_subscription(Bool, '/sensors/stereo/rectified/ready', self.rectified_ready_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_depth_node started | '
            f'baseline={self.baseline_m:.3f}m, fx={self.fx_px:.1f}, stop_distance={self.safe_stop_distance_m:.2f}m'
        )

    def camera_ready_cb(self, msg: Bool):
        self.camera_ready = bool(msg.data)

    def rectified_ready_cb(self, msg: Bool):
        self.rectified_ready = bool(msg.data)

    def compute_nominal_depth(self) -> float:
        disparity = max(self.disparity_min_px, self.disparity_nominal_px)
        return (self.fx_px * self.baseline_m) / disparity

    def timer_callback(self):
        depth_ready = self.camera_ready and self.rectified_ready
        nominal_depth_m = self.compute_nominal_depth()
        hazard_close = nominal_depth_m <= self.safe_stop_distance_m

        ready = Bool()
        ready.data = depth_ready
        self.ready_pub.publish(ready)

        distance = Float32()
        distance.data = nominal_depth_m
        self.distance_pub.publish(distance)

        hazard = Bool()
        hazard.data = hazard_close
        self.hazard_pub.publish(hazard)

        status = String()
        status.data = (
            f'ready={depth_ready}, min_distance_m={nominal_depth_m:.2f}, '
            f'safe_stop_distance_m={self.safe_stop_distance_m:.2f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
