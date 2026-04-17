import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CameraInfo, Image


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        self.declare_parameter('left_camera_name', 'left_imx219')
        self.declare_parameter('right_camera_name', 'right_imx219')
        self.declare_parameter('left_frame_id', 'stereo_left_optical_frame')
        self.declare_parameter('right_frame_id', 'stereo_right_optical_frame')
        self.declare_parameter('left_topic', '/sensors/stereo/left/image_raw')
        self.declare_parameter('right_topic', '/sensors/stereo/right/image_raw')
        self.declare_parameter('left_camera_source', 'csi:0')
        self.declare_parameter('right_camera_source', 'csi:1')
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('camera_flip_method', 0)
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
        self.left_camera_source = str(self.get_parameter('left_camera_source').value)
        self.right_camera_source = str(self.get_parameter('right_camera_source').value)
        self.camera_backend = str(self.get_parameter('camera_backend').value).lower().strip()
        self.camera_flip_method = int(self.get_parameter('camera_flip_method').value)
        self.flip_horizontal = bool(self.get_parameter('flip_horizontal').value)
        self.flip_vertical = bool(self.get_parameter('flip_vertical').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)

        self.left_image_pub = self.create_publisher(Image, self.left_topic, 10)
        self.right_image_pub = self.create_publisher(Image, self.right_topic, 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/right/camera_info', 10)
        self.status_pub = self.create_publisher(String, '/sensors/stereo/status', 10)
        self.ready_pub = self.create_publisher(Bool, '/sensors/stereo/ready', 10)

        self.left_capture, self.left_active_source = self.open_capture(self.left_camera_source)
        self.right_capture, self.right_active_source = self.open_capture(self.right_camera_source)
        self.left_frames = 0
        self.right_frames = 0
        self.last_frame_time = None

        period = 1.0 / max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_camera_node started | '
            f'left={self.left_camera_name}, right={self.right_camera_name}, '
            f'size={self.image_width}x{self.image_height}@{self.fps:.1f}fps, '
            f'baseline={self.baseline_m:.3f}m, '
            f'left_source={self.left_active_source}, right_source={self.right_active_source}'
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

    def make_csi_pipeline(self, sensor_id: int) -> str:
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width={self.image_width}, height={self.image_height}, '
            f'framerate={int(self.fps)}/1, format=NV12 ! '
            f'nvvidconv flip-method={self.camera_flip_method} ! '
            'video/x-raw, format=BGRx ! videoconvert ! '
            'video/x-raw, format=BGR ! appsink max-buffers=1 drop=true sync=false'
        )

    def open_capture(self, source: str):
        source = str(source).strip()
        active_source = source
        if source.lower().startswith('csi:'):
            sensor_id = int(source.split(':', 1)[1])
            active_source = self.make_csi_pipeline(sensor_id)
            capture = cv2.VideoCapture(active_source, cv2.CAP_GSTREAMER)
        elif source.lower() in ('csi', 'nvargus', 'nvarguscamerasrc'):
            active_source = self.make_csi_pipeline(0)
            capture = cv2.VideoCapture(active_source, cv2.CAP_GSTREAMER)
        elif '!' in source or self.camera_backend == 'gstreamer':
            capture = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)
        elif source.isdigit():
            capture = cv2.VideoCapture(int(source))
            active_source = f'/dev/video{source}'
            self.apply_capture_size(capture)
        else:
            capture = cv2.VideoCapture(source)
            self.apply_capture_size(capture)

        if not capture.isOpened():
            self.get_logger().error(f'camera open failed: {active_source}')
        return capture, active_source

    def apply_capture_size(self, capture):
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.image_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        if self.image_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        if self.fps > 0:
            capture.set(cv2.CAP_PROP_FPS, self.fps)

    def prepare_frame(self, frame):
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.flip_vertical:
            frame = cv2.flip(frame, 0)
        return frame

    def bgr_to_image_msg(self, frame, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        return msg

    def read_capture(self, capture):
        if not capture.isOpened():
            return False, None
        ok, frame = capture.read()
        if not ok or frame is None:
            return False, None
        return True, self.prepare_frame(frame)

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        left_ok, left_frame = self.read_capture(self.left_capture)
        right_ok, right_frame = self.read_capture(self.right_capture)

        left_info = self.make_camera_info(self.left_camera_name, self.left_frame_id)
        right_info = self.make_camera_info(self.right_camera_name, self.right_frame_id)
        left_info.header.stamp = stamp
        right_info.header.stamp = stamp
        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)

        if left_ok:
            self.left_image_pub.publish(self.bgr_to_image_msg(left_frame, self.left_frame_id, stamp))
            self.left_frames += 1
        if right_ok:
            self.right_image_pub.publish(self.bgr_to_image_msg(right_frame, self.right_frame_id, stamp))
            self.right_frames += 1

        ready = Bool()
        ready.data = left_ok and right_ok
        self.ready_pub.publish(ready)

        now = time.monotonic()
        frame_age = 0.0 if self.last_frame_time is None else now - self.last_frame_time
        if ready.data:
            self.last_frame_time = now

        status = String()
        status.data = (
            f'ready={ready.data}, left_ok={left_ok}, right_ok={right_ok}, '
            f'left_topic={self.left_topic}, right_topic={self.right_topic}, '
            f'left_source={self.left_active_source}, right_source={self.right_active_source}, '
            f'flip_h={self.flip_horizontal}, flip_v={self.flip_vertical}, '
            f'baseline_m={self.baseline_m:.3f}, left_frames={self.left_frames}, '
            f'right_frames={self.right_frames}, last_frame_age_sec={frame_age:.2f}'
        )
        self.status_pub.publish(status)

    def destroy_node(self):
        if hasattr(self, 'left_capture'):
            self.left_capture.release()
        if hasattr(self, 'right_capture'):
            self.right_capture.release()
        super().destroy_node()


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
