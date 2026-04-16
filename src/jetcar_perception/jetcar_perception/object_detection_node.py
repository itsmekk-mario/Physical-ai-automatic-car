import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('engine_path', 'models/yolov8n.engine')
        self.declare_parameter('target_classes', ['person', 'car', 'stop sign'])
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.engine_path = str(self.get_parameter('engine_path').value)
        self.target_classes = list(self.get_parameter('target_classes').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)

        self.ready_pub = self.create_publisher(Bool, '/perception/detections/ready', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/detections/hazard', 10)
        self.closest_pub = self.create_publisher(Float32, '/perception/detections/closest_confidence', 10)
        self.status_pub = self.create_publisher(String, '/perception/detections/status', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'object_detection_node started | '
            f'engine_path={self.engine_path}, confidence_threshold={self.confidence_threshold:.2f}'
        )

    def timer_callback(self):
        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)

        hazard = Bool()
        hazard.data = False
        self.hazard_pub.publish(hazard)

        confidence = Float32()
        confidence.data = 0.0
        self.closest_pub.publish(confidence)

        status = String()
        status.data = (
            f'engine_path={self.engine_path}, '
            f'target_classes={",".join(self.target_classes)}, '
            f'confidence_threshold={self.confidence_threshold:.2f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
