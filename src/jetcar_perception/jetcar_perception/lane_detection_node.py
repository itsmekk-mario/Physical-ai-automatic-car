import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lane_center_offset_m', 0.0)
        self.declare_parameter('heading_error_deg', 0.0)
        self.declare_parameter('lane_confidence', 0.9)
        self.declare_parameter('assist_gain', 0.7)

        self.lane_center_offset_m = float(self.get_parameter('lane_center_offset_m').value)
        self.heading_error_deg = float(self.get_parameter('heading_error_deg').value)
        self.lane_confidence = float(self.get_parameter('lane_confidence').value)
        self.assist_gain = float(self.get_parameter('assist_gain').value)

        self.ready_pub = self.create_publisher(Bool, '/perception/lane/ready', 10)
        self.offset_pub = self.create_publisher(Float32, '/perception/lane/offset_m', 10)
        self.heading_pub = self.create_publisher(Float32, '/perception/lane/heading_error_deg', 10)
        self.confidence_pub = self.create_publisher(Float32, '/perception/lane/confidence', 10)
        self.suggested_pub = self.create_publisher(Float32, '/perception/lane/suggested_steering', 10)
        self.status_pub = self.create_publisher(String, '/perception/lane/status', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'lane_detection_node started | '
            f'offset={self.lane_center_offset_m:.2f}m, heading_error={self.heading_error_deg:.1f}deg'
        )

    def timer_callback(self):
        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)

        offset_msg = Float32()
        offset_msg.data = self.lane_center_offset_m
        self.offset_pub.publish(offset_msg)

        heading_msg = Float32()
        heading_msg.data = self.heading_error_deg
        self.heading_pub.publish(heading_msg)

        confidence_msg = Float32()
        confidence_msg.data = self.lane_confidence
        self.confidence_pub.publish(confidence_msg)

        suggested = Float32()
        suggested.data = max(-1.0, min(1.0, -self.assist_gain * self.lane_center_offset_m))
        self.suggested_pub.publish(suggested)

        status = String()
        status.data = (
            f'offset_m={self.lane_center_offset_m:.2f}, heading_error_deg={self.heading_error_deg:.1f}, '
            f'confidence={self.lane_confidence:.2f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
