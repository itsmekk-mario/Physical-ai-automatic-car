import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        self.declare_parameter('baseline_m', 0.06)
        self.declare_parameter('fx_px', 700.0)
        self.declare_parameter('disparity_min_px', 2.0)
        self.declare_parameter('disparity_nominal_px', 42.0)
        self.declare_parameter('safe_stop_distance_m', 0.8)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('left_image_topic', '/sensors/stereo/left/image_raw')
        self.declare_parameter('right_image_topic', '/sensors/stereo/right/image_raw')
        self.declare_parameter('max_frame_age_sec', 0.5)
        self.declare_parameter('stereo_num_disparities', 96)
        self.declare_parameter('stereo_block_size', 7)
        self.declare_parameter('roi_top_fraction', 0.45)
        self.declare_parameter('roi_bottom_fraction', 0.95)
        self.declare_parameter('roi_left_fraction', 0.25)
        self.declare_parameter('roi_right_fraction', 0.75)

        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.fx_px = float(self.get_parameter('fx_px').value)
        self.disparity_min_px = float(self.get_parameter('disparity_min_px').value)
        self.disparity_nominal_px = float(self.get_parameter('disparity_nominal_px').value)
        self.safe_stop_distance_m = float(self.get_parameter('safe_stop_distance_m').value)
        self.left_image_topic = str(self.get_parameter('left_image_topic').value)
        self.right_image_topic = str(self.get_parameter('right_image_topic').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)
        self.roi_top_fraction = float(self.get_parameter('roi_top_fraction').value)
        self.roi_bottom_fraction = float(self.get_parameter('roi_bottom_fraction').value)
        self.roi_left_fraction = float(self.get_parameter('roi_left_fraction').value)
        self.roi_right_fraction = float(self.get_parameter('roi_right_fraction').value)

        self.rectified_ready = False
        self.camera_ready = False
        self.left_frame = None
        self.right_frame = None
        self.left_time = None
        self.right_time = None
        self.last_depth = None

        num_disparities = int(self.get_parameter('stereo_num_disparities').value)
        num_disparities = max(16, (num_disparities // 16) * 16)
        block_size = int(self.get_parameter('stereo_block_size').value)
        if block_size % 2 == 0:
            block_size += 1
        block_size = max(5, block_size)
        self.stereo = cv2.StereoBM_create(numDisparities=num_disparities, blockSize=block_size)

        self.ready_pub = self.create_publisher(Bool, '/perception/depth/ready', 10)
        self.distance_pub = self.create_publisher(Float32, '/perception/depth/min_distance_m', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/depth/hazard_close', 10)
        self.status_pub = self.create_publisher(String, '/perception/depth/status', 10)

        self.create_subscription(Bool, '/sensors/stereo/ready', self.camera_ready_cb, 10)
        self.create_subscription(Bool, '/sensors/stereo/rectified/ready', self.rectified_ready_cb, 10)
        self.create_subscription(Image, self.left_image_topic, self.left_image_cb, 10)
        self.create_subscription(Image, self.right_image_topic, self.right_image_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'stereo_depth_node started | '
            f'baseline={self.baseline_m:.3f}m, fx={self.fx_px:.1f}, stop_distance={self.safe_stop_distance_m:.2f}m, '
            f'left_topic={self.left_image_topic}, right_topic={self.right_image_topic}'
        )

    def camera_ready_cb(self, msg: Bool):
        self.camera_ready = bool(msg.data)

    def rectified_ready_cb(self, msg: Bool):
        self.rectified_ready = bool(msg.data)

    def left_image_cb(self, msg: Image):
        self.left_frame = self.image_to_gray(msg)
        self.left_time = self.get_clock().now()

    def right_image_cb(self, msg: Image):
        self.right_frame = self.image_to_gray(msg)
        self.right_time = self.get_clock().now()

    def image_to_gray(self, msg: Image):
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        channels = 1 if msg.encoding == 'mono8' else 3
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        if msg.encoding == 'mono8':
            return frame
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def compute_nominal_depth(self) -> float:
        disparity = max(self.disparity_min_px, self.disparity_nominal_px)
        return (self.fx_px * self.baseline_m) / disparity

    def seconds_since(self, stamp):
        if stamp is None:
            return 999.0
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def frames_fresh(self):
        return (
            self.seconds_since(self.left_time) <= self.max_frame_age_sec
            and self.seconds_since(self.right_time) <= self.max_frame_age_sec
        )

    def compute_depth(self):
        if self.left_frame is None or self.right_frame is None:
            return None
        if self.left_frame.shape != self.right_frame.shape:
            right = cv2.resize(self.right_frame, (self.left_frame.shape[1], self.left_frame.shape[0]))
        else:
            right = self.right_frame

        disparity = self.stereo.compute(self.left_frame, right).astype(np.float32) / 16.0
        height, width = disparity.shape[:2]
        y1 = int(max(0.0, min(0.95, self.roi_top_fraction)) * height)
        y2 = int(max(self.roi_top_fraction, min(1.0, self.roi_bottom_fraction)) * height)
        x1 = int(max(0.0, min(0.95, self.roi_left_fraction)) * width)
        x2 = int(max(self.roi_left_fraction, min(1.0, self.roi_right_fraction)) * width)
        roi = disparity[y1:y2, x1:x2]
        valid = roi[roi >= self.disparity_min_px]
        if valid.size < 50:
            return None

        near_disparity = float(np.percentile(valid, 95.0))
        if near_disparity < self.disparity_min_px:
            return None
        return (self.fx_px * self.baseline_m) / near_disparity

    def timer_callback(self):
        image_ready = self.frames_fresh()
        measured_depth = self.compute_depth() if image_ready else None
        depth_ready = self.camera_ready and self.rectified_ready and image_ready and measured_depth is not None
        min_depth_m = measured_depth if measured_depth is not None else self.compute_nominal_depth()
        hazard_close = depth_ready and min_depth_m <= self.safe_stop_distance_m

        ready = Bool()
        ready.data = depth_ready
        self.ready_pub.publish(ready)

        distance = Float32()
        distance.data = float(min_depth_m)
        self.distance_pub.publish(distance)

        hazard = Bool()
        hazard.data = hazard_close
        self.hazard_pub.publish(hazard)

        status = String()
        status.data = (
            f'ready={depth_ready}, camera_ready={self.camera_ready}, rectified_ready={self.rectified_ready}, '
            f'images_fresh={image_ready}, left_age_sec={self.seconds_since(self.left_time):.2f}, '
            f'right_age_sec={self.seconds_since(self.right_time):.2f}, min_distance_m={min_depth_m:.2f}, '
            f'safe_stop_distance_m={self.safe_stop_distance_m:.2f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
