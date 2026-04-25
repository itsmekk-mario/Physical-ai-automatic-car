from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import CameraInfo, Image
import yaml


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
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.left_rect_info = None
        self.right_rect_info = None
        self.calibration_loaded = False
        self.calibration_error = ''

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
        self.load_calibration()

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

    def resolve_calibration_path(self) -> Path:
        candidate = Path(self.calibration_file).expanduser()
        if candidate.is_absolute():
            return candidate
        search_roots = [
            Path.cwd(),
            Path(__file__).resolve().parent.parent / 'config',
        ]
        try:
            search_roots.append(Path(get_package_share_directory('jetcar_perception')) / 'config')
        except PackageNotFoundError:
            pass
        for root in search_roots:
            resolved = (root / candidate).resolve()
            if resolved.exists():
                return resolved
            fallback = (root / candidate.name).resolve()
            if fallback.exists():
                return fallback
        return (search_roots[0] / candidate).resolve()

    def load_yaml_matrix(self, section, key, shape):
        values = section.get(key)
        if values is None:
            raise ValueError(f'missing calibration key: {key}')
        array = np.array(values, dtype=np.float64)
        return array.reshape(shape)

    def load_calibration(self):
        calibration_path = self.resolve_calibration_path()
        if not calibration_path.exists():
            self.calibration_loaded = False
            self.calibration_error = f'calibration file not found: {calibration_path}'
            self.get_logger().warning(self.calibration_error)
            return

        try:
            with calibration_path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}

            left = data['left']
            right = data['right']
            stereo = data['stereo']

            image_width = int(data.get('image_width', self.image_width))
            image_height = int(data.get('image_height', self.image_height))
            image_size = (image_width, image_height)

            left_k = self.load_yaml_matrix(left, 'camera_matrix', (3, 3))
            right_k = self.load_yaml_matrix(right, 'camera_matrix', (3, 3))
            left_d = np.array(left.get('dist_coeffs', [0.0] * 5), dtype=np.float64).reshape(-1, 1)
            right_d = np.array(right.get('dist_coeffs', [0.0] * 5), dtype=np.float64).reshape(-1, 1)
            left_r = self.load_yaml_matrix(stereo, 'R1', (3, 3))
            right_r = self.load_yaml_matrix(stereo, 'R2', (3, 3))
            left_p = self.load_yaml_matrix(stereo, 'P1', (3, 4))
            right_p = self.load_yaml_matrix(stereo, 'P2', (3, 4))

            self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
                left_k, left_d, left_r, left_p[:3, :3], image_size, cv2.CV_32FC1
            )
            self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
                right_k, right_d, right_r, right_p[:3, :3], image_size, cv2.CV_32FC1
            )

            self.image_width = image_width
            self.image_height = image_height
            self.fx_px = float(left_p[0, 0])
            self.fy_px = float(left_p[1, 1])
            self.cx_px = float(left_p[0, 2])
            self.cy_px = float(left_p[1, 2])
            if self.fx_px > 0.0 and right_p[0, 3] != 0.0:
                self.baseline_m = abs(float(right_p[0, 3])) / self.fx_px

            self.left_rect_info = self.make_rectified_info(self.left_frame_id, left_p)
            self.right_rect_info = self.make_rectified_info(self.right_frame_id, right_p)
            self.calibration_loaded = True
            self.calibration_error = ''
            self.get_logger().info(f'loaded stereo calibration: {calibration_path}')
        except Exception as exc:
            self.calibration_loaded = False
            self.calibration_error = f'failed to load calibration: {exc}'
            self.get_logger().warning(self.calibration_error)

    def make_rectified_info(self, frame_id: str, tx: float) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        if isinstance(tx, np.ndarray):
            projection = tx
            msg.k = [
                float(projection[0, 0]), 0.0, float(projection[0, 2]),
                0.0, float(projection[1, 1]), float(projection[1, 2]),
                0.0, 0.0, 1.0,
            ]
            msg.p = [float(value) for value in projection.reshape(-1)]
        else:
            msg.k = [self.fx_px, 0.0, self.cx_px, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 1.0]
            msg.p = [self.fx_px, 0.0, self.cx_px, tx, 0.0, self.fy_px, self.cy_px, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.d = [0.0] * 5
        return msg

    def image_to_array(self, msg: Image):
        if msg.encoding == 'mono8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        if msg.encoding in ('bgr8', 'rgb8'):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == 'rgb8':
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        raise ValueError(f'unsupported image encoding: {msg.encoding}')

    def array_to_image(self, frame, template: Image, frame_id: str, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.is_bigendian = False
        if frame.ndim == 2:
            msg.encoding = 'mono8'
            msg.step = int(frame.shape[1])
        else:
            msg.encoding = 'bgr8'
            msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def rectify_frame(self, msg: Image, map1, map2):
        frame = self.image_to_array(msg)
        if map1 is None or map2 is None:
            return frame
        interpolation = cv2.INTER_LINEAR
        return cv2.remap(frame, map1, map2, interpolation)

    def timer_callback(self):
        ready_now = self.frames_fresh()
        stamp = self.get_clock().now().to_msg()

        left_info = self.left_rect_info or self.make_rectified_info(self.left_frame_id, 0.0)
        right_info = self.right_rect_info or self.make_rectified_info(self.right_frame_id, -self.fx_px * self.baseline_m)
        left_info.header.stamp = stamp
        right_info.header.stamp = stamp
        self.left_rect_info_pub.publish(left_info)
        self.right_rect_info_pub.publish(right_info)

        if ready_now:
            left_rect = self.rectify_frame(self.left_frame, self.left_map1, self.left_map2)
            right_rect = self.rectify_frame(self.right_frame, self.right_map1, self.right_map2)
            self.left_rect_pub.publish(self.array_to_image(left_rect, self.left_frame, self.left_frame_id, stamp))
            self.right_rect_pub.publish(self.array_to_image(right_rect, self.right_frame, self.right_frame_id, stamp))

        ready = Bool()
        ready.data = ready_now
        self.ready_pub.publish(ready)

        status = String()
        status.data = (
            f'calibration_file={self.calibration_file}, '
            f'calibration_loaded={self.calibration_loaded}, '
            f'baseline_m={self.baseline_m:.3f}, fx_px={self.fx_px:.1f}, '
            f'left_age_sec={self.seconds_since(self.left_stamp):.2f}, '
            f'right_age_sec={self.seconds_since(self.right_stamp):.2f}, ready={ready_now}, '
            f'error={self.calibration_error or "none"}'
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
