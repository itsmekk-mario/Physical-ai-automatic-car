import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lane_center_offset_m', 0.0)
        self.declare_parameter('heading_error_deg', 0.0)
        self.declare_parameter('lane_confidence', 0.9)
        self.declare_parameter('assist_gain', 0.7)
        self.declare_parameter('image_topic', '/sensors/stereo/left/image_raw')
        self.declare_parameter('meters_per_pixel', 0.0025)
        self.declare_parameter('roi_top_fraction', 0.55)
        self.declare_parameter('min_lane_pixels', 150)
        self.declare_parameter('max_frame_age_sec', 0.5)

        self.default_lane_center_offset_m = float(self.get_parameter('lane_center_offset_m').value)
        self.default_heading_error_deg = float(self.get_parameter('heading_error_deg').value)
        self.default_lane_confidence = float(self.get_parameter('lane_confidence').value)
        self.assist_gain = float(self.get_parameter('assist_gain').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.meters_per_pixel = float(self.get_parameter('meters_per_pixel').value)
        self.roi_top_fraction = float(self.get_parameter('roi_top_fraction').value)
        self.min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)

        self.latest_result = None
        self.latest_image_time = None

        self.ready_pub = self.create_publisher(Bool, '/perception/lane/ready', 10)
        self.offset_pub = self.create_publisher(Float32, '/perception/lane/offset_m', 10)
        self.heading_pub = self.create_publisher(Float32, '/perception/lane/heading_error_deg', 10)
        self.confidence_pub = self.create_publisher(Float32, '/perception/lane/confidence', 10)
        self.suggested_pub = self.create_publisher(Float32, '/perception/lane/suggested_steering', 10)
        self.status_pub = self.create_publisher(String, '/perception/lane/status', 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'lane_detection_node started | '
            f'image_topic={self.image_topic}, meters_per_pixel={self.meters_per_pixel:.4f}'
        )

    def image_cb(self, msg: Image):
        try:
            frame = self.image_to_bgr(msg)
            self.latest_result = self.detect_lane(frame)
            self.latest_image_time = self.get_clock().now()
        except Exception as exc:
            self.latest_result = None
            self.latest_image_time = self.get_clock().now()
            self.get_logger().warning(f'lane image processing failed: {exc}')

    def image_to_bgr(self, msg: Image):
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        channels = 1 if msg.encoding == 'mono8' else 3
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'mono8':
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def detect_lane(self, frame):
        height, width = frame.shape[:2]
        roi_y = int(max(0.0, min(0.95, self.roi_top_fraction)) * height)
        roi = frame[roi_y:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white = cv2.inRange(hsv, np.array([0, 0, 170]), np.array([180, 70, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([40, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        ys, xs = np.nonzero(mask)
        pixel_count = int(xs.size)
        if pixel_count < self.min_lane_pixels:
            return {
                'ready': False,
                'offset_m': self.default_lane_center_offset_m,
                'heading_error_deg': self.default_heading_error_deg,
                'confidence': 0.0,
                'pixel_count': pixel_count,
            }

        lane_center_px = float(np.median(xs))
        image_center_px = width / 2.0
        offset_m = (lane_center_px - image_center_px) * self.meters_per_pixel

        heading_error_deg = self.default_heading_error_deg
        if pixel_count >= max(self.min_lane_pixels, 20):
            sample = max(1, pixel_count // 2000)
            fit = np.polyfit(ys[::sample].astype(float), xs[::sample].astype(float), 1)
            heading_error_deg = float(np.degrees(np.arctan(fit[0])))

        confidence = min(1.0, pixel_count / max(float(self.min_lane_pixels) * 6.0, 1.0))
        return {
            'ready': True,
            'offset_m': float(offset_m),
            'heading_error_deg': heading_error_deg,
            'confidence': confidence,
            'pixel_count': pixel_count,
        }

    def seconds_since_image(self):
        if self.latest_image_time is None:
            return 999.0
        return (self.get_clock().now() - self.latest_image_time).nanoseconds / 1e9

    def timer_callback(self):
        frame_age = self.seconds_since_image()
        result = self.latest_result or {
            'ready': False,
            'offset_m': self.default_lane_center_offset_m,
            'heading_error_deg': self.default_heading_error_deg,
            'confidence': 0.0,
            'pixel_count': 0,
        }
        lane_ready = bool(result['ready']) and frame_age <= self.max_frame_age_sec
        offset_m = float(result['offset_m'])
        heading_error_deg = float(result['heading_error_deg'])
        confidence = float(result['confidence']) if lane_ready else 0.0

        ready = Bool()
        ready.data = lane_ready
        self.ready_pub.publish(ready)

        offset_msg = Float32()
        offset_msg.data = offset_m
        self.offset_pub.publish(offset_msg)

        heading_msg = Float32()
        heading_msg.data = heading_error_deg
        self.heading_pub.publish(heading_msg)

        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.confidence_pub.publish(confidence_msg)

        suggested = Float32()
        suggested.data = max(-1.0, min(1.0, -self.assist_gain * offset_m))
        self.suggested_pub.publish(suggested)

        status = String()
        status.data = (
            f'ready={lane_ready}, image_age_sec={frame_age:.2f}, offset_m={offset_m:.2f}, '
            f'heading_error_deg={heading_error_deg:.1f}, confidence={confidence:.2f}, '
            f'pixels={int(result["pixel_count"])}'
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
