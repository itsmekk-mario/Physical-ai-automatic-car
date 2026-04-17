import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from flask import Flask, Response, jsonify, render_template_string
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>JetCar YOLO View</title>
    <style>
        body {
            margin: 0;
            font-family: Arial, sans-serif;
            background: #101010;
            color: #f1f1f1;
        }
        header {
            padding: 14px 18px;
            background: #181818;
            border-bottom: 1px solid #303030;
        }
        h1 {
            margin: 0;
            font-size: 22px;
        }
        main {
            max-width: 1180px;
            margin: 0 auto;
            padding: 18px;
        }
        img {
            display: block;
            width: 100%;
            max-height: 78vh;
            object-fit: contain;
            background: #000;
        }
        .status {
            margin-top: 12px;
            color: #cfcfcf;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <header><h1>JetCar YOLO View</h1></header>
    <main>
        <img src="/stream" alt="YOLO stream">
        <div class="status" id="status">Loading...</div>
    </main>
    <script>
        async function refreshStatus() {
            const response = await fetch('/api/status');
            const data = await response.json();
            document.getElementById('status').textContent =
                `ready=${data.ready} model=${data.model_loaded} fps=${data.fps.toFixed(1)} ` +
                `detections=${data.detection_count} hazard=${data.hazard} ` +
                `model_path=${data.model_path} camera=${data.camera_source} status=${data.status}`;
        }
        setInterval(refreshStatus, 1000);
        refreshStatus();
    </script>
</body>
</html>
"""


class YoloWebNode(Node):
    def __init__(self):
        super().__init__('yolo_web_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('camera_source', 'auto')
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('camera_sensor_id', 0)
        self.declare_parameter('camera_flip_method', 0)
        self.declare_parameter('model_path', 'auto')
        self.declare_parameter('target_classes', ['person', 'car', 'stop sign'])
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 30)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.camera_source = str(self.get_parameter('camera_source').value)
        self.camera_backend = str(self.get_parameter('camera_backend').value).lower().strip()
        self.camera_sensor_id = int(self.get_parameter('camera_sensor_id').value)
        self.camera_flip_method = int(self.get_parameter('camera_flip_method').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.target_classes = [str(name) for name in self.get_parameter('target_classes').value]
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fps = int(self.get_parameter('camera_fps').value)

        self.ready_pub = self.create_publisher(Bool, '/perception/detections/ready', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/detections/hazard', 10)
        self.closest_pub = self.create_publisher(Float32, '/perception/detections/closest_confidence', 10)
        self.status_pub = self.create_publisher(String, '/perception/detections/status', 10)

        self.frame_lock = threading.Lock()
        self.latest_jpeg = None
        self.latest_status = 'starting'
        self.latest_detection_count = 0
        self.latest_confidence = 0.0
        self.latest_hazard = False
        self.latest_ready = False
        self.latest_fps = 0.0
        self.model_loaded = False
        self.model_error = ''
        self.active_model_path = ''
        self.active_camera_source = ''
        self.pending_frame = None
        self.stop_event = threading.Event()

        self.model = self.load_model()
        self.app = Flask(__name__)
        self.setup_routes()

        self.worker_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.worker_thread.start()
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'yolo_web_node started | '
            f'url=http://{self.host}:{self.port}, camera_source={self.camera_source}, model={self.model_path}'
        )

    def load_model(self):
        try:
            from ultralytics import YOLO

            model_path = self.resolve_model_path()
            model = YOLO(model_path)
            self.model_loaded = True
            self.active_model_path = model_path
            self.latest_status = 'model loaded'
            return model
        except Exception as exc:
            self.model_loaded = False
            self.model_error = str(exc)
            self.latest_status = f'model unavailable: {self.model_error}'
            self.get_logger().warning(self.latest_status)
            return None

    def resolve_model_path(self):
        if self.model_path and self.model_path.lower() != 'auto':
            return self.model_path

        candidates = [
            'models/yolov8n.engine',
            'yolov8n.engine',
            'models/yolov8n.pt',
            'yolov8n.pt',
        ]
        for candidate in candidates:
            if Path(candidate).exists():
                return candidate
        raise FileNotFoundError(
            'no YOLO model found; put yolov8n.engine or yolov8n.pt in /home/kown/jetcar_ws/models, '
            'or set model_path in yolo_web.yaml'
        )

    def make_csi_pipeline(self, sensor_id=None):
        if sensor_id is None:
            sensor_id = self.camera_sensor_id
        return (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width={self.camera_width}, height={self.camera_height}, '
            f'framerate={self.camera_fps}/1, format=NV12 ! '
            f'nvvidconv flip-method={self.camera_flip_method} ! '
            'video/x-raw, format=BGRx ! videoconvert ! '
            'video/x-raw, format=BGR ! appsink drop=1 sync=false'
        )

    def open_capture(self):
        errors = []
        for source in self.camera_candidates():
            capture, active_source = self.open_capture_candidate(source)
            if not capture.isOpened():
                errors.append(f'{active_source}: not opened')
                capture.release()
                continue

            ok, frame = capture.read()
            if not ok or frame is None:
                errors.append(f'{active_source}: no frame')
                capture.release()
                continue

            self.pending_frame = frame
            self.active_camera_source = active_source
            self.get_logger().info(f'camera opened: {active_source}')
            return capture

        self.active_camera_source = '; '.join(errors) if errors else self.camera_source
        return cv2.VideoCapture()

    def camera_candidates(self):
        source = self.camera_source.lower().strip()
        if source == 'auto':
            return [
                ('csi', self.camera_sensor_id),
                ('csi', 1 if self.camera_sensor_id == 0 else 0),
                '0',
                '1',
            ]
        return [self.camera_source]

    def open_capture_candidate(self, source):
        if isinstance(source, tuple) and source[0] == 'csi':
            pipeline = self.make_csi_pipeline(sensor_id=source[1])
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), pipeline

        source = str(source)
        if source.lower() in ('csi', 'nvargus', 'nvarguscamerasrc'):
            pipeline = self.make_csi_pipeline()
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), pipeline
        if '!' in source or self.camera_backend == 'gstreamer':
            return cv2.VideoCapture(source, cv2.CAP_GSTREAMER), source
        if source.isdigit():
            capture = cv2.VideoCapture(int(source))
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            if self.camera_width > 0:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            if self.camera_height > 0:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            if self.camera_fps > 0:
                capture.set(cv2.CAP_PROP_FPS, self.camera_fps)
            return capture, f'/dev/video{source}'

        capture = cv2.VideoCapture(source)
        self.apply_capture_size(capture)
        return capture, source

    def read_frame(self, capture):
        if self.pending_frame is not None:
            frame = self.pending_frame
            self.pending_frame = None
            return True, frame
        return capture.read()

    def make_capture_unavailable_text(self):
        if self.camera_source.lower().strip() == 'auto':
            return f'Camera unavailable after trying: {self.active_camera_source}'
        if self.active_camera_source:
            return f'Camera unavailable: {self.active_camera_source}'
        return f'Camera unavailable: {self.camera_source}'

    def apply_capture_size(self, capture):
        if self.camera_width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        if self.camera_height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        if self.camera_fps > 0:
            capture.set(cv2.CAP_PROP_FPS, self.camera_fps)

    def capture_loop(self):
        capture = self.open_capture()
        if not capture.isOpened():
            status = self.make_capture_unavailable_text()
            jpeg = self.encode_frame(self.make_status_frame(status))
            with self.frame_lock:
                self.latest_status = status
                self.latest_jpeg = jpeg
            self.get_logger().error(self.latest_status)
            return

        last_time = time.monotonic()
        while not self.stop_event.is_set():
            ok, frame = self.read_frame(capture)
            if not ok or frame is None:
                with self.frame_lock:
                    self.latest_ready = False
                    self.latest_status = 'camera frame read failed'
                time.sleep(0.1)
                continue

            detections = []
            detection_error = None
            if self.model is not None:
                try:
                    detections = self.run_detection(frame)
                except Exception as exc:
                    detection_error = f'detection failed: {exc}'
                    self.get_logger().warning(detection_error)
                self.draw_detections(frame, detections)
            else:
                cv2.putText(
                    frame,
                    'YOLO model unavailable',
                    (20, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            now = time.monotonic()
            elapsed = max(now - last_time, 0.001)
            last_time = now
            fps = 1.0 / elapsed
            jpeg = self.encode_frame(frame)
            if jpeg is None:
                continue

            hazard = bool(detections)
            confidence = max((detection['confidence'] for detection in detections), default=0.0)
            status = detection_error or ('ok' if self.model_loaded else self.latest_status)

            with self.frame_lock:
                self.latest_jpeg = jpeg
                self.latest_ready = True
                self.latest_detection_count = len(detections)
                self.latest_hazard = hazard
                self.latest_confidence = confidence
                self.latest_fps = fps
                self.latest_status = status

        capture.release()

    def make_status_frame(self, text):
        frame = np.full((360, 640, 3), (20, 20, 20), dtype=np.uint8)
        for index, line in enumerate(self.wrap_text(text, 48)[:5]):
            cv2.putText(
                frame,
                line,
                (24, 130 + index * 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (240, 240, 240),
                2,
                cv2.LINE_AA,
            )
        return frame

    def wrap_text(self, text, max_chars):
        words = str(text).split()
        lines = []
        current = ''
        for word in words:
            if len(current) + len(word) + 1 > max_chars:
                if current:
                    lines.append(current)
                current = word
            else:
                current = f'{current} {word}'.strip()
        if current:
            lines.append(current)
        return lines or ['']

    def run_detection(self, frame):
        results = self.model.predict(
            source=frame,
            conf=self.confidence_threshold,
            imgsz=self.image_size,
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
            if self.target_classes and class_name not in self.target_classes:
                continue
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            detections.append({
                'class_name': class_name,
                'confidence': confidence,
                'box': (x1, y1, x2, y2),
            })
        return detections

    def draw_detections(self, frame, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection['box']
            label = f"{detection['class_name']} {detection['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (20, 220, 20), 2)
            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 8, 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (20, 220, 20),
                2,
                cv2.LINE_AA,
            )

    def encode_frame(self, frame):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(100, self.jpeg_quality))]
        ok, encoded = cv2.imencode('.jpg', frame, params)
        if not ok:
            return None
        return encoded.tobytes()

    def setup_routes(self):
        @self.app.route('/', methods=['GET'])
        def index():
            return render_template_string(HTML_PAGE)

        @self.app.route('/api/status', methods=['GET'])
        def api_status():
            with self.frame_lock:
                return jsonify({
                    'ready': self.latest_ready,
                    'model_loaded': self.model_loaded,
                    'fps': self.latest_fps,
                    'detection_count': self.latest_detection_count,
                    'hazard': self.latest_hazard,
                    'confidence': self.latest_confidence,
                    'model_error': self.model_error,
                    'model_path': self.active_model_path or self.model_path,
                    'camera_source': self.active_camera_source or self.camera_source,
                    'status': self.latest_status,
                })

        @self.app.route('/stream', methods=['GET'])
        def stream():
            return Response(self.stream_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def stream_frames(self):
        while not self.stop_event.is_set():
            with self.frame_lock:
                jpeg = self.latest_jpeg
            if jpeg is None:
                time.sleep(0.05)
                continue
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
            time.sleep(0.03)

    def run_flask(self):
        self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False, threaded=True)

    def timer_callback(self):
        with self.frame_lock:
            ready = self.latest_ready
            hazard = self.latest_hazard
            confidence = self.latest_confidence
            status = (
                f'model_loaded={self.model_loaded}, camera_source={self.active_camera_source or self.camera_source}, '
                f'detections={self.latest_detection_count}, fps={self.latest_fps:.1f}, '
                f'status={self.latest_status}'
            )

        ready_msg = Bool()
        ready_msg.data = ready
        self.ready_pub.publish(ready_msg)

        hazard_msg = Bool()
        hazard_msg.data = hazard
        self.hazard_pub.publish(hazard_msg)

        confidence_msg = Float32()
        confidence_msg.data = confidence
        self.closest_pub.publish(confidence_msg)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def shutdown(self):
        self.stop_event.set()
        self.worker_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = YoloWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
