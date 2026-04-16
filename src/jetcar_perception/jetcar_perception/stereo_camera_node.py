import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CameraInfo


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.declare_parameter('left_camera_name', 'left_imx219')
        self.declare_parameter('right_camera_name', 'right_imx219')
        self.declare_parameter('left_frame_id', 'stereo_left_optical_frame')
        self.declare_parameter('right_frame_id', 'stereo_right_optical_frame')
        self.declare_parameter('left_topic', '/sensors/stereo/left/image_raw')
        self.declare_parameter('right_topic', '/sensors/stereo/right/image_raw')
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.left_camera_name = str(self.get_parameter('left_camera_name').value)
        self.right_camera_name = str(self.get_parameter('right_camera_name').value)
        self.left_frame_id = str(self.get_parameter('left_frame_id').value)
        self.right_frame_id = str(self.get_parameter('right_frame_id').value)
        self.left_topic = str(self.get_parameter('left_topic').value)
        self.right_topic = str(self.get_parameter('right_topic').value)
        self.flip_horizontal = bool(self.get_parameter('flip_horizontal').value)
        self.flip_vertical = bool(self.get_parameter('flip_vertical').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)

        self.left_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/right/camera_info', 10)
        self.status_pub = self.create_publisher(String, '/sensors/stereo/status', 10)
        self.ready_pub = self.create_publisher(Bool, '/sensors/stereo/ready', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_camera_node started | '
            f'left={self.left_camera_name}, right={self.right_camera_name}, '
            f'size={self.image_width}x{self.image_height}@{self.fps:.1f}fps, '
            f'baseline={self.baseline_m:.3f}m'
        )

    def make_camera_info(self, camera_name: str, frame_id: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        msg.k = [1.0, 0.0, self.image_width / 2.0, 0.0, 1.0, self.image_height / 2.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, self.image_width / 2.0, 0.0, 0.0, 1.0, self.image_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.header.frame_id = frame_id
        return msg

    def timer_callback(self):
        self.left_info_pub.publish(self.make_camera_info(self.left_camera_name, self.left_frame_id))
        self.right_info_pub.publish(self.make_camera_info(self.right_camera_name, self.right_frame_id))

        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)

        status = String()
        status.data = (
            f'left_topic={self.left_topic}, right_topic={self.right_topic}, '
            f'flip_h={self.flip_horizontal}, flip_v={self.flip_vertical}, '
            f'baseline_m={self.baseline_m:.3f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
