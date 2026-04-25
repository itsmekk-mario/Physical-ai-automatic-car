from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('engine_path', 'models/yolov8n_int8.engine')
        self.declare_parameter('image_topic', '/sensors/stereo/left/image_rect')
        self.declare_parameter('target_classes', ['all'])
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('image_size', 320)
        self.declare_parameter('inference_device', 'cuda:0')
        self.declare_parameter('prefer_half', False)
        self.declare_parameter('max_frame_age_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.engine_path = str(self.get_parameter('engine_path').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.target_classes = list(self.get_parameter('target_classes').value)
        self.target_class_set = {
            str(name).strip().lower()
            for name in self.target_classes
            if str(name).strip() and str(name).strip().lower() not in ('all', '*', 'any')
        }
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.inference_device = str(self.get_parameter('inference_device').value).strip()
        self.prefer_half = bool(self.get_parameter('prefer_half').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)

        self.model_search_paths = []
        self.active_model_path = ''
        self.model_error = ''
        self.model_loaded = False
        self.last_status = 'starting'
        self.last_image_time = None
        self.latest_detection_count = 0
        self.latest_confidence = 0.0
        self.latest_hazard = False
        self.latest_detections = []

        self.ready_pub = self.create_publisher(Bool, '/perception/detections/ready', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/detections/hazard', 10)
        self.person_pub = self.create_publisher(Bool, '/perception/detections/person_detected', 10)
        self.closest_pub = self.create_publisher(Float32, '/perception/detections/closest_confidence', 10)
        self.offset_pub = self.create_publisher(Float32, '/perception/detections/closest_offset', 10)
        self.area_pub = self.create_publisher(Float32, '/perception/detections/closest_area_ratio', 10)
        self.status_pub = self.create_publisher(String, '/perception/detections/status', 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 1)

        self.model = self.load_model()

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'object_detection_node started | '
            f'image_topic={self.image_topic}, engine_path={self.engine_path}, '
            f'confidence_threshold={self.confidence_threshold:.2f}'
        )

    def model_search_roots(self):
        roots = []

        def add_root(path):
            resolved = Path(path).expanduser().resolve()
            if resolved not in roots:
                roots.append(resolved)

        add_root(Path.cwd())
        for parent in Path.cwd().parents:
            if parent.name.endswith('_ws'):
                add_root(parent)
                break

        source_path = Path(__file__).resolve()
        for parent in source_path.parents:
            if parent.name.endswith('_ws'):
                add_root(parent)
                break

        try:
            add_root(get_package_share_directory('jetcar_perception'))
        except PackageNotFoundError:
            pass

        return roots

    def resolve_model_path(self):
        if self.engine_path and self.engine_path.lower() != 'auto':
            return str(Path(self.engine_path).expanduser())

        self.model_search_paths = []
        for root in self.model_search_roots():
            for filename in ('yolov8n_int8.engine', 'yolov8n.engine', 'yolov8n_int8.pt', 'yolov8n.pt'):
                for candidate in (root / 'models' / filename, root / filename):
                    self.model_search_paths.append(str(candidate))
                    if candidate.exists():
                        return str(candidate)

        self.model_search_paths.append('yolov8n_int8.pt (ultralytics default/cache/download)')
        self.model_search_paths.append('yolov8n.pt (ultralytics default/cache/download)')
        return 'yolov8n.pt'

    def load_model(self):
        try:
            from ultralytics import YOLO

            model_path = self.resolve_model_path()
            model = YOLO(model_path)
            self.active_model_path = model_path
            self.model_loaded = True
            self.last_status = 'model loaded'
            self.get_logger().info(f'YOLO model loaded: {model_path}')
            return model
        except Exception as exc:
            self.model_loaded = False
            searched = ', '.join(self.model_search_paths)
            self.model_error = f'{exc}; searched: {searched}' if searched else str(exc)
            self.last_status = f'model unavailable: {self.model_error}'
            self.get_logger().warning(self.last_status)
            return None

    def seconds_since_image(self):
        if self.last_image_time is None:
            return 999.0
        return (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9

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

    def run_detection(self, frame):
        frame_height, frame_width = frame.shape[:2]
        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            imgsz=self.image_size,
            device=self.inference_device or None,
            half=bool(self.prefer_half and str(self.active_model_path).lower().endswith('.pt')),
            verbose=False,
        )
        detections = []
        if not results:
            return detections

        result = results[0]
        names = result.names
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = str(names.get(class_id, class_id))
            confidence = float(box.conf[0])
            if self.target_class_set and class_name.strip().lower() not in self.target_class_set:
                continue
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            box_width = max(1, x2 - x1)
            box_height = max(1, y2 - y1)
            center_x = 0.5 * (x1 + x2)
            detections.append({
                'class_name': class_name,
                'confidence': confidence,
                'box': (x1, y1, x2, y2),
                'center_offset': float((center_x - (frame_width / 2.0)) / max(frame_width / 2.0, 1.0)),
                'area_ratio': float((box_width * box_height) / max(frame_width * frame_height, 1)),
            })
        return detections

    def select_primary_detection(self):
        if not self.latest_detections:
            return None
        return max(
            self.latest_detections,
            key=lambda detection: detection.get('area_ratio', 0.0) * max(detection.get('confidence', 0.0), 0.01),
        )

    def image_cb(self, msg: Image):
        self.last_image_time = self.get_clock().now()
        if self.model is None:
            self.latest_detection_count = 0
            self.latest_confidence = 0.0
            self.latest_hazard = False
            return

        try:
            frame = self.image_to_bgr(msg)
            detections = self.run_detection(frame)
            self.latest_detections = detections
            self.latest_detection_count = len(detections)
            self.latest_confidence = max((d['confidence'] for d in detections), default=0.0)
            self.latest_hazard = bool(detections)
            self.last_status = 'ok'
        except Exception as exc:
            self.latest_detections = []
            self.latest_detection_count = 0
            self.latest_confidence = 0.0
            self.latest_hazard = False
            self.last_status = f'detection failed: {exc}'
            self.get_logger().warning(self.last_status)

    def timer_callback(self):
        ready = Bool()
        ready.data = self.model_loaded and self.seconds_since_image() <= self.max_frame_age_sec
        self.ready_pub.publish(ready)

        hazard = Bool()
        hazard.data = bool(self.latest_hazard) and ready.data
        self.hazard_pub.publish(hazard)

        person_detected = Bool()
        person_detected.data = ready.data and any(
            detection.get('class_name') == 'person' for detection in self.latest_detections
        )
        self.person_pub.publish(person_detected)

        confidence = Float32()
        confidence.data = float(self.latest_confidence if ready.data else 0.0)
        self.closest_pub.publish(confidence)

        primary_detection = self.select_primary_detection()

        offset = Float32()
        offset.data = float(primary_detection.get('center_offset', 0.0)) if ready.data and primary_detection else 0.0
        self.offset_pub.publish(offset)

        area = Float32()
        area.data = float(primary_detection.get('area_ratio', 0.0)) if ready.data and primary_detection else 0.0
        self.area_pub.publish(area)

        status = String()
        status.data = (
            f'ready={ready.data}, image_age_sec={self.seconds_since_image():.2f}, '
            f'engine_path={self.active_model_path or self.engine_path}, '
            f'target_classes={",".join(self.target_classes)}, '
            f'confidence_threshold={self.confidence_threshold:.2f}, '
            f'detections={self.latest_detection_count}, hazard={hazard.data}, person_detected={person_detected.data}, '
            f'closest_offset={offset.data:.2f}, closest_area_ratio={area.data:.4f}, '
            f'status={self.last_status}'
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
