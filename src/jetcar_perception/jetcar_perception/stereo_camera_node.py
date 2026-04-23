import threading
import time

import cv2
import gi
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)


class GStreamerCapture:
    def __init__(self, pipeline: str, sink_name: str):
        self.pipeline = Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name(sink_name)
        if self.appsink is None:
            raise RuntimeError(f'appsink "{sink_name}" not found in pipeline')
        self.appsink.set_property('emit-signals', False)
        self.appsink.set_property('sync', False)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.opened = True

    def isOpened(self):
        return self.opened

    def read(self, timeout_ns: int = 1_000_000_000):
        if not self.opened:
            return False, None
        sample = self.appsink.emit('try-pull-sample', timeout_ns)
        if sample is None:
            return False, None

        buf = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = int(structure.get_value('width'))
        height = int(structure.get_value('height'))
        fmt = str(structure.get_value('format'))

        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return False, None
        try:
            if fmt == 'GRAY8':
                frame = np.ndarray((height, width), dtype=np.uint8, buffer=map_info.data).copy()
            else:
                channels = 4 if fmt == 'BGRx' else 3
                frame = np.ndarray((height, width, channels), dtype=np.uint8, buffer=map_info.data).copy()
        finally:
            buf.unmap(map_info)

        if fmt == 'BGRx':
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return True, frame

    def release(self):
        if not self.opened:
            return
        self.opened = False
        self.pipeline.set_state(Gst.State.NULL)


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
        self.declare_parameter('camera_flip_method', 0)
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('max_frame_age_sec', 0.5)
        self.declare_parameter('camera_read_failures_before_reopen', 5)
        self.declare_parameter('capture_warmup_sec', 6.0)
        self.declare_parameter('camera_retry_sec', 1.0)
        self.declare_parameter('right_camera_start_delay_sec', 0.75)
        self.declare_parameter('left_sensor_mode', 4)
        self.declare_parameter('right_sensor_mode', 4)
        self.declare_parameter('publish_encoding', 'mono8')

        self.left_camera_name = str(self.get_parameter('left_camera_name').value)
        self.right_camera_name = str(self.get_parameter('right_camera_name').value)
        self.left_frame_id = str(self.get_parameter('left_frame_id').value)
        self.right_frame_id = str(self.get_parameter('right_frame_id').value)
        self.left_topic = str(self.get_parameter('left_topic').value)
        self.right_topic = str(self.get_parameter('right_topic').value)
        self.left_camera_source = str(self.get_parameter('left_camera_source').value)
        self.right_camera_source = str(self.get_parameter('right_camera_source').value)
        self.camera_flip_method = int(self.get_parameter('camera_flip_method').value)
        self.flip_horizontal = bool(self.get_parameter('flip_horizontal').value)
        self.flip_vertical = bool(self.get_parameter('flip_vertical').value)
        self.capture_width = int(self.get_parameter('capture_width').value)
        self.capture_height = int(self.get_parameter('capture_height').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.max_frame_age_sec = max(0.05, float(self.get_parameter('max_frame_age_sec').value))
        self.camera_read_failures_before_reopen = max(
            1, int(self.get_parameter('camera_read_failures_before_reopen').value)
        )
        self.capture_warmup_sec = max(1.0, float(self.get_parameter('capture_warmup_sec').value))
        self.camera_retry_sec = max(0.2, float(self.get_parameter('camera_retry_sec').value))
        self.right_camera_start_delay_sec = max(0.0, float(self.get_parameter('right_camera_start_delay_sec').value))
        self.left_sensor_mode = int(self.get_parameter('left_sensor_mode').value)
        self.right_sensor_mode = int(self.get_parameter('right_sensor_mode').value)
        self.publish_encoding = str(self.get_parameter('publish_encoding').value).lower().strip()

        self.left_image_pub = self.create_publisher(Image, self.left_topic, 10)
        self.right_image_pub = self.create_publisher(Image, self.right_topic, 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/sensors/stereo/right/camera_info', 10)
        self.status_pub = self.create_publisher(String, '/sensors/stereo/status', 10)
        self.ready_pub = self.create_publisher(Bool, '/sensors/stereo/ready', 10)

        self.stop_event = threading.Event()
        self.state_lock = threading.Lock()
        self.side_state = {
            'left': self.make_side_state(self.left_camera_source),
            'right': self.make_side_state(self.right_camera_source),
        }

        self.workers = [
            threading.Thread(target=self.capture_loop, args=('left', 0.0), daemon=True),
            threading.Thread(target=self.capture_loop, args=('right', self.right_camera_start_delay_sec), daemon=True),
        ]
        for worker in self.workers:
            worker.start()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            'stereo_camera_node started | '
            f'size={self.image_width}x{self.image_height}@{self.fps:.1f}fps, '
            f'publish_rate_hz={self.publish_rate_hz:.1f}, '
            f'left_source={self.left_camera_source}, right_source={self.right_camera_source}'
        )

    def make_side_state(self, source: str):
        return {
            'source': source,
            'capture': None,
            'active_source': '',
            'latest_frame': None,
            'latest_time': None,
            'frames': 0,
            'read_failures': 0,
            'last_error': 'starting',
        }

    def make_camera_info(self, frame_id: str, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        msg.k = [1.0, 0.0, self.image_width / 2.0, 0.0, 1.0, self.image_height / 2.0, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, self.image_width / 2.0, 0.0, 0.0, 1.0, self.image_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def make_csi_pipeline(self, sensor_id: int, sink_name: str, sensor_mode: int) -> str:
        if self.publish_encoding == 'mono8':
            return (
                f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! '
                f'video/x-raw(memory:NVMM), width={self.capture_width}, height={self.capture_height}, '
                f'framerate={int(self.fps)}/1, format=NV12 ! '
                f'nvvidconv flip-method={self.camera_flip_method} ! '
                f'video/x-raw, width={self.image_width}, height={self.image_height}, format=GRAY8 ! '
                f'queue max-size-buffers=1 leaky=downstream ! appsink name={sink_name} '
                'max-buffers=1 drop=true sync=false'
            )
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! '
            f'video/x-raw(memory:NVMM), width={self.capture_width}, height={self.capture_height}, '
            f'framerate={int(self.fps)}/1, format=NV12 ! '
            f'nvvidconv flip-method={self.camera_flip_method} ! '
            f'video/x-raw, width={self.image_width}, height={self.image_height}, format=BGRx ! '
            'queue max-size-buffers=1 leaky=downstream ! videoconvert ! '
            f'video/x-raw, format=BGR ! appsink name={sink_name} max-buffers=1 drop=true sync=false'
        )

    def open_capture(self, source: str, side: str):
        if source.lower().startswith('csi:'):
            sensor_id = int(source.split(':', 1)[1])
            sink_name = f'{side}_capture_sink_{sensor_id}'
            sensor_mode = self.left_sensor_mode if side == 'left' else self.right_sensor_mode
            pipeline = self.make_csi_pipeline(sensor_id, sink_name, sensor_mode)
            return GStreamerCapture(pipeline, sink_name), pipeline

        if source.isdigit():
            capture = cv2.VideoCapture(int(source))
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            return capture, f'/dev/video{source}'

        raise RuntimeError(f'unsupported camera source: {source}')

    def try_open_capture(self, side: str):
        source = self.side_state[side]['source']
        capture = None
        try:
            capture, active_source = self.open_capture(source, side)
            if not capture.isOpened():
                if capture is not None:
                    capture.release()
                self.get_logger().error(f'{side} camera open failed: {active_source}: not opened')
                return None, active_source

            deadline = time.monotonic() + self.capture_warmup_sec
            while time.monotonic() < deadline and not self.stop_event.is_set():
                ok, frame = capture.read()
                if ok and frame is not None:
                    self.get_logger().info(f'{side} camera opened: {active_source}')
                    return capture, active_source
                time.sleep(0.05)

            capture.release()
            self.get_logger().error(f'{side} camera open failed: {active_source}: no frame')
            return None, active_source
        except Exception as exc:
            if capture is not None:
                capture.release()
            self.get_logger().error(f'{side} camera open failed: {source}: {exc}')
            return None, source

    def prepare_frame(self, frame):
        if frame.shape[1] != self.image_width or frame.shape[0] != self.image_height:
            frame = cv2.resize(frame, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.flip_vertical:
            frame = cv2.flip(frame, 0)
        if self.publish_encoding == 'mono8' and frame.ndim == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def capture_loop(self, side: str, start_delay_sec: float):
        if start_delay_sec > 0:
            self.stop_event.wait(start_delay_sec)

        while not self.stop_event.is_set():
            capture, active_source = self.try_open_capture(side)
            with self.state_lock:
                self.side_state[side]['capture'] = capture
                self.side_state[side]['active_source'] = active_source
                self.side_state[side]['read_failures'] = 0
                self.side_state[side]['last_error'] = 'ok' if capture is not None else 'open failed'

            if capture is None:
                self.stop_event.wait(self.camera_retry_sec)
                continue

            while not self.stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    with self.state_lock:
                        self.side_state[side]['read_failures'] += 1
                        failures = self.side_state[side]['read_failures']
                        self.side_state[side]['last_error'] = 'read failed'
                    if failures >= self.camera_read_failures_before_reopen:
                        self.get_logger().warning(f'{side} camera read failed repeatedly, reopening capture')
                        break
                    self.stop_event.wait(0.01)
                    continue

                prepared = self.prepare_frame(frame)
                now = self.get_clock().now()
                with self.state_lock:
                    self.side_state[side]['latest_frame'] = prepared
                    self.side_state[side]['latest_time'] = now
                    self.side_state[side]['frames'] += 1
                    self.side_state[side]['read_failures'] = 0
                    self.side_state[side]['last_error'] = 'ok'

            capture.release()
            with self.state_lock:
                self.side_state[side]['capture'] = None
            self.stop_event.wait(self.camera_retry_sec)

    def frame_is_fresh(self, stamp):
        if stamp is None:
            return False
        age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age_sec <= self.max_frame_age_sec

    def bgr_to_image_msg(self, frame, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        if frame.ndim == 2:
            msg.encoding = 'mono8'
            msg.step = int(frame.shape[1])
        else:
            msg.encoding = 'bgr8'
            msg.step = int(frame.shape[1] * 3)
        msg.is_bigendian = False
        msg.data = frame.tobytes()
        return msg

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        with self.state_lock:
            left_frame = None if self.side_state['left']['latest_frame'] is None else self.side_state['left']['latest_frame'].copy()
            right_frame = None if self.side_state['right']['latest_frame'] is None else self.side_state['right']['latest_frame'].copy()
            left_time = self.side_state['left']['latest_time']
            right_time = self.side_state['right']['latest_time']
            left_source = self.side_state['left']['active_source']
            right_source = self.side_state['right']['active_source']
            left_frames = self.side_state['left']['frames']
            right_frames = self.side_state['right']['frames']
            left_error = self.side_state['left']['last_error']
            right_error = self.side_state['right']['last_error']

        left_ok = left_frame is not None and self.frame_is_fresh(left_time)
        right_ok = right_frame is not None and self.frame_is_fresh(right_time)

        self.left_info_pub.publish(self.make_camera_info(self.left_frame_id, stamp))
        self.right_info_pub.publish(self.make_camera_info(self.right_frame_id, stamp))

        if left_ok:
            self.left_image_pub.publish(self.bgr_to_image_msg(left_frame, self.left_frame_id, stamp))
        if right_ok:
            self.right_image_pub.publish(self.bgr_to_image_msg(right_frame, self.right_frame_id, stamp))

        ready = Bool()
        ready.data = left_ok and right_ok
        self.ready_pub.publish(ready)

        left_age_sec = 999.0 if left_time is None else (self.get_clock().now() - left_time).nanoseconds / 1e9
        right_age_sec = 999.0 if right_time is None else (self.get_clock().now() - right_time).nanoseconds / 1e9

        status = String()
        status.data = (
            f'ready={ready.data}, left_ok={left_ok}, right_ok={right_ok}, '
            f'left_age_sec={left_age_sec:.2f}, right_age_sec={right_age_sec:.2f}, '
            f'left_frames={left_frames}, right_frames={right_frames}, '
            f'left_source={left_source}, right_source={right_source}, '
            f'left_error={left_error}, right_error={right_error}'
        )
        self.status_pub.publish(status)

    def destroy_node(self):
        self.stop_event.set()
        for worker in getattr(self, 'workers', []):
            worker.join(timeout=1.0)
        with self.state_lock:
            for side in ('left', 'right'):
                capture = self.side_state[side]['capture']
                if capture is not None:
                    capture.release()
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
