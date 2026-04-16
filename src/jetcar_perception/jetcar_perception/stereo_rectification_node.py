import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import CameraInfo


class StereoRectificationNode(Node):
    def __init__(self):
        super().__init__('stereo_rectification_node')

        self.declare_parameter('calibration_file', 'config/stereo_calibration.yaml')
        self.declare_parameter('fx_px', 700.0)
        self.declare_parameter('fy_px', 700.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)
        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.fx_px = float(self.get_parameter('fx_px').value)
        self.fy_px = float(self.get_parameter('fy_px').value)
        self.cx_px = float(self.get_parameter('cx_px').value)
        self.cy_px = float(self.get_parameter('cy_px').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)

        self.left_rect_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/left/camera_info_rect', 10)
        self.right_rect_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/right/camera_info_rect', 10)
        self.ready_pub = self.create_publisher(Bool, '/sensors/stereo/rectified/ready', 10)
        self.status_pub = self.create_publisher(String, '/sensors/stereo/calibration/status', 10)
        self.focal_pub = self.create_publisher(Float32, '/sensors/stereo/calibration/fx_px', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_rectification_node started | '
            f'calibration_file={self.calibration_file}, fx={self.fx_px:.1f}, baseline={self.baseline_m:.3f}'
        )

    def make_rectified_info(self, frame_id: str, tx: float) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.width = 1280
        msg.height = 720
        msg.distortion_model = 'plumb_bob'
        msg.k = [self.fx_px, 0.0, self.cx_px, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [self.fx_px, 0.0, self.cx_px, tx, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def timer_callback(self):
        self.left_rect_info_pub.publish(self.make_rectified_info('stereo_left_optical_frame', 0.0))
        self.right_rect_info_pub.publish(
            self.make_rectified_info('stereo_right_optical_frame', -self.fx_px * self.baseline_m)
        )

        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)

        status = String()
        status.data = (
            f'calibration_file={self.calibration_file}, '
            f'baseline_m={self.baseline_m:.3f}, fx_px={self.fx_px:.1f}'
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
