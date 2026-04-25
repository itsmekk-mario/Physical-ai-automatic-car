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
        self.declare_parameter('expected_lane_width_px', 180.0)
        self.declare_parameter('heading_gain', 0.015)
        self.declare_parameter('max_heading_error_deg', 35.0)
        self.declare_parameter('detector_backend', 'cv')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('inference_device', 'cuda:0')
        self.declare_parameter('prefer_half', True)
        self.declare_parameter('yolo_class_name', 'lane')
        self.declare_parameter('yolo_min_mask_pixels', 80)

        self.default_lane_center_offset_m = float(self.get_parameter('lane_center_offset_m').value)
        self.default_heading_error_deg = float(self.get_parameter('heading_error_deg').value)
        self.default_lane_confidence = float(self.get_parameter('lane_confidence').value)
        self.assist_gain = float(self.get_parameter('assist_gain').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.meters_per_pixel = float(self.get_parameter('meters_per_pixel').value)
        self.roi_top_fraction = float(self.get_parameter('roi_top_fraction').value)
        self.min_lane_pixels = int(self.get_parameter('min_lane_pixels').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)
        self.expected_lane_width_px = max(20.0, float(self.get_parameter('expected_lane_width_px').value))
        self.heading_gain = max(0.0, float(self.get_parameter('heading_gain').value))
        self.max_heading_error_deg = max(1.0, float(self.get_parameter('max_heading_error_deg').value))
        self.detector_backend = str(self.get_parameter('detector_backend').value).lower().strip()
        self.model_path = str(self.get_parameter('model_path').value).strip()
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.inference_device = str(self.get_parameter('inference_device').value).strip()
        self.prefer_half = bool(self.get_parameter('prefer_half').value)
        self.yolo_class_name = str(self.get_parameter('yolo_class_name').value).lower().strip()
        self.yolo_min_mask_pixels = max(1, int(self.get_parameter('yolo_min_mask_pixels').value))

        self.latest_result = None
        self.latest_image_time = None
        self.lane_model = None
        self.active_model_path = ''
        self.model_error = ''
        self.load_lane_model()

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
            f'image_topic={self.image_topic}, backend={self.detector_backend}, '
            f'meters_per_pixel={self.meters_per_pixel:.4f}'
        )

    def load_lane_model(self):
        if self.detector_backend not in ('yolo', 'auto'):
            return
        if not self.model_path:
            self.model_error = 'model_path is empty'
            if self.detector_backend == 'yolo':
                self.detector_backend = 'cv'
            self.get_logger().warning(f'YOLO lane model unavailable: {self.model_error}; using cv backend')
            return

        try:
            from ultralytics import YOLO

            self.lane_model = YOLO(self.model_path)
            self.active_model_path = self.model_path
            self.detector_backend = 'yolo'
            self.get_logger().info(f'YOLO lane model loaded: {self.active_model_path}')
        except Exception as exc:
            self.lane_model = None
            self.model_error = str(exc)
            if self.detector_backend == 'yolo':
                self.detector_backend = 'cv'
            self.get_logger().warning(f'YOLO lane model unavailable: {self.model_error}; using cv backend')

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
        if self.detector_backend == 'yolo' and self.lane_model is not None:
            return self.detect_lane_yolo(frame)
        return self.detect_lane_cv(frame)

    def detect_lane_yolo(self, frame):
        height, width = frame.shape[:2]
        results = self.lane_model.predict(
            source=frame,
            conf=self.confidence_threshold,
            imgsz=self.image_size,
            device=self.inference_device or None,
            half=bool(self.prefer_half),
            verbose=False,
        )
        if not results:
            return self.empty_result(0)

        result = results[0]
        if result.masks is None or result.boxes is None:
            return self.empty_result(0)

        names = result.names or {}
        mask_image = np.zeros((height, width), dtype=np.uint8)
        polygons = result.masks.xy or []
        boxes = result.boxes
        for index, polygon in enumerate(polygons):
            if polygon is None or len(polygon) < 3:
                continue
            confidence = float(boxes.conf[index]) if boxes.conf is not None and index < len(boxes.conf) else 1.0
            if confidence < self.confidence_threshold:
                continue
            class_id = int(boxes.cls[index]) if boxes.cls is not None and index < len(boxes.cls) else 0
            class_name = str(names.get(class_id, class_id)).lower().strip()
            if self.yolo_class_name and class_name != self.yolo_class_name:
                continue
            points = np.asarray(polygon, dtype=np.int32)
            points[:, 0] = np.clip(points[:, 0], 0, width - 1)
            points[:, 1] = np.clip(points[:, 1], 0, height - 1)
            cv2.fillPoly(mask_image, [points], 255)

        roi_y = int(max(0.0, min(0.95, self.roi_top_fraction)) * height)
        if roi_y > 0:
            mask_image[:roi_y, :] = 0

        ys, xs = np.nonzero(mask_image)
        pixel_count = int(xs.size)
        if pixel_count < self.yolo_min_mask_pixels:
            return self.empty_result(pixel_count)

        return self.measure_lane_from_pixels(
            xs,
            ys - roi_y,
            width,
            max(1, height - roi_y),
            pixel_count,
        )

    def detect_lane_cv(self, frame):
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
            return self.empty_result(pixel_count)

        return self.measure_lane_from_pixels(xs, ys, width, roi.shape[0], pixel_count)

    def empty_result(self, pixel_count):
        return {
            'ready': False,
            'offset_m': self.default_lane_center_offset_m,
            'heading_error_deg': self.default_heading_error_deg,
            'confidence': 0.0,
            'pixel_count': pixel_count,
            'backend': self.detector_backend,
        }

    def measure_lane_from_pixels(self, xs, ys, width, roi_height, pixel_count):
        left_points = xs < (width / 2.0)
        right_points = ~left_points
        left_xs = xs[left_points]
        left_ys = ys[left_points]
        right_xs = xs[right_points]
        right_ys = ys[right_points]

        left_fit = self.fit_lane_line(left_xs, left_ys)
        right_fit = self.fit_lane_line(right_xs, right_ys)
        lane_center_px = self.estimate_lane_center(width, roi_height, left_fit, right_fit)
        if lane_center_px is None:
            lane_center_px = float(np.median(xs))
        image_center_px = width / 2.0
        offset_m = (lane_center_px - image_center_px) * self.meters_per_pixel

        heading_error_deg = self.estimate_heading_error_deg(left_fit, right_fit)

        lane_count = int(left_fit is not None) + int(right_fit is not None)
        confidence = min(1.0, pixel_count / max(float(self.min_lane_pixels) * 6.0, 1.0))
        if lane_count == 1:
            confidence *= 0.7
        elif lane_count == 0:
            confidence = 0.0
        return {
            'ready': lane_count > 0,
            'offset_m': float(offset_m),
            'heading_error_deg': heading_error_deg,
            'confidence': confidence,
            'pixel_count': pixel_count,
            'lane_count': lane_count,
            'backend': self.detector_backend,
        }

    def fit_lane_line(self, xs, ys):
        if xs.size < self.min_lane_pixels // 3:
            return None
        sample = max(1, xs.size // 1000)
        fit = np.polyfit(ys[::sample].astype(float), xs[::sample].astype(float), 1)
        return float(fit[0]), float(fit[1])

    def lane_x_at(self, fit, y_value: float):
        if fit is None:
            return None
        slope, intercept = fit
        return slope * y_value + intercept

    def estimate_lane_center(self, width: int, roi_height: int, left_fit, right_fit):
        sample_y = float(max(roi_height - 1, 0))
        left_x = self.lane_x_at(left_fit, sample_y)
        right_x = self.lane_x_at(right_fit, sample_y)
        if left_x is not None and right_x is not None:
            return 0.5 * (left_x + right_x)
        if left_x is not None:
            return left_x + (self.expected_lane_width_px / 2.0)
        if right_x is not None:
            return right_x - (self.expected_lane_width_px / 2.0)
        return None

    def estimate_heading_error_deg(self, left_fit, right_fit):
        slopes = []
        if left_fit is not None:
            slopes.append(left_fit[0])
        if right_fit is not None:
            slopes.append(right_fit[0])
        if not slopes:
            return self.default_heading_error_deg
        avg_slope = float(sum(slopes) / len(slopes))
        heading = float(np.degrees(np.arctan(avg_slope)))
        return max(-self.max_heading_error_deg, min(self.max_heading_error_deg, heading))

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
        heading_term = 0.0
        if self.max_heading_error_deg > 0.0:
            heading_term = heading_error_deg / self.max_heading_error_deg
        suggested_steering = -self.assist_gain * offset_m - self.heading_gain * heading_term
        if confidence < 0.2:
            suggested_steering = 0.0

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
        suggested.data = max(-1.0, min(1.0, suggested_steering))
        self.suggested_pub.publish(suggested)

        status = String()
        status.data = (
            f'ready={lane_ready}, image_age_sec={frame_age:.2f}, offset_m={offset_m:.2f}, '
            f'heading_error_deg={heading_error_deg:.1f}, confidence={confidence:.2f}, '
            f'pixels={int(result["pixel_count"])}, lanes={int(result.get("lane_count", 0))}, '
            f'backend={result.get("backend", self.detector_backend)}'
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
