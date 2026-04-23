import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import CameraInfo, Image


class StereoRectificationNode(Node):
    def __init__(self):
        super().__init__('stereo_rectification_node')

        self.declare_parameter('calibration_file', 'config/stereo_calibration.yaml')
        self.declare_parameter('fx_px', 700.0)
        self.declare_parameter('fy_px', 700.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)
        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('left_frame_id', 'stereo_left_optical_frame')
        self.declare_parameter('right_frame_id', 'stereo_right_optical_frame')
        self.declare_parameter('left_image_topic', '/sensors/stereo/left/image_raw')
        self.declare_parameter('right_image_topic', '/sensors/stereo/right/image_raw')
        self.declare_parameter('left_rectified_topic', '/sensors/stereo/left/image_rect')
        self.declare_parameter('right_rectified_topic', '/sensors/stereo/right/image_rect')
        self.declare_parameter('max_frame_age_sec', 0.5)

        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.fx_px = float(self.get_parameter('fx_px').value)
        self.fy_px = float(self.get_parameter('fy_px').value)
        self.cx_px = float(self.get_parameter('cx_px').value)
        self.cy_px = float(self.get_parameter('cy_px').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.left_frame_id = str(self.get_parameter('left_frame_id').value)
        self.right_frame_id = str(self.get_parameter('right_frame_id').value)
        self.left_image_topic = str(self.get_parameter('left_image_topic').value)
        self.right_image_topic = str(self.get_parameter('right_image_topic').value)
        self.left_rectified_topic = str(self.get_parameter('left_rectified_topic').value)
        self.right_rectified_topic = str(self.get_parameter('right_rectified_topic').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)

        self.left_frame = None
        self.right_frame = None
        self.left_stamp = None
        self.right_stamp = None

        self.left_rect_pub = self.create_publisher(Image, self.left_rectified_topic, 10)
        self.right_rect_pub = self.create_publisher(Image, self.right_rectified_topic, 10)
        self.left_rect_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/left/camera_info_rect', 10)
        self.right_rect_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/right/camera_info_rect', 10)
        self.ready_pub = self.create_publisher(Bool, '/sensors/stereo/rectified/ready', 10)
        self.status_pub = self.create_publisher(String, '/sensors/stereo/calibration/status', 10)
        self.focal_pub = self.create_publisher(Float32, '/sensors/stereo/calibration/fx_px', 10)

        self.create_subscription(Image, self.left_image_topic, self.left_image_cb, 10)
        self.create_subscription(Image, self.right_image_topic, self.right_image_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_rectification_node started | '
            f'calibration_file={self.calibration_file}, fx={self.fx_px:.1f}, baseline={self.baseline_m:.3f}'
        )

    def left_image_cb(self, msg: Image):
        self.left_frame = msg
        self.left_stamp = self.get_clock().now()

    def right_image_cb(self, msg: Image):
        self.right_frame = msg
        self.right_stamp = self.get_clock().now()

    def seconds_since(self, stamp):
        if stamp is None:
            return 999.0
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def frames_fresh(self):
        return (
            self.left_frame is not None
            and self.right_frame is not None
            and self.seconds_since(self.left_stamp) <= self.max_frame_age_sec
            and self.seconds_since(self.right_stamp) <= self.max_frame_age_sec
        )

    def make_rectified_info(self, frame_id: str, tx: float) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        msg.k = [self.fx_px, 0.0, self.cx_px, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [self.fx_px, 0.0, self.cx_px, tx, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def timer_callback(self):
        ready_now = self.frames_fresh()
        stamp = self.get_clock().now().to_msg()

        left_info = self.make_rectified_info(self.left_frame_id, 0.0)
        right_info = self.make_rectified_info(self.right_frame_id, -self.fx_px * self.baseline_m)
        left_info.header.stamp = stamp
        right_info.header.stamp = stamp
        self.left_rect_info_pub.publish(left_info)
        self.right_rect_info_pub.publish(right_info)

        if ready_now:
            self.left_rect_pub.publish(self.left_frame)
            self.right_rect_pub.publish(self.right_frame)

        ready = Bool()
        ready.data = ready_now
        self.ready_pub.publish(ready)

        status = String()
        status.data = (
            f'calibration_file={self.calibration_file}, '
            f'baseline_m={self.baseline_m:.3f}, fx_px={self.fx_px:.1f}, '
            f'left_age_sec={self.seconds_since(self.left_stamp):.2f}, '
            f'right_age_sec={self.seconds_since(self.right_stamp):.2f}, ready={ready_now}'
        )
        self.status_pub.publish(status)

        focal = Float32()
        focal.data = self.fx_px
        self.focal_pub.publish(focal)


def main(args=None):
    rclpy.init(args=args)
    node = StereoRectificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
