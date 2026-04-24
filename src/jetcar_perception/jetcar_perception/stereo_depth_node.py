import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
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
        self.declare_parameter('left_image_topic', '/sensors/stereo/left/image_rect')
        self.declare_parameter('right_image_topic', '/sensors/stereo/right/image_rect')
        self.declare_parameter('left_camera_info_topic', '/sensors/stereo/left/camera_info_rect')
        self.declare_parameter('right_camera_info_topic', '/sensors/stereo/right/camera_info_rect')
        self.declare_parameter('max_frame_age_sec', 0.5)
        self.declare_parameter('stereo_num_disparities', 128)
        self.declare_parameter('stereo_block_size', 5)
        self.declare_parameter('roi_top_fraction', 0.45)
        self.declare_parameter('roi_bottom_fraction', 0.95)
        self.declare_parameter('roi_left_fraction', 0.25)
        self.declare_parameter('roi_right_fraction', 0.75)
        self.declare_parameter('min_valid_depth_samples', 80)
        self.declare_parameter('depth_percentile', 20.0)
        self.declare_parameter('speckle_window_size', 50)
        self.declare_parameter('speckle_range', 2)

        self.baseline_m = float(self.get_parameter('baseline_m').value)
        self.fx_px = float(self.get_parameter('fx_px').value)
        self.disparity_min_px = float(self.get_parameter('disparity_min_px').value)
        self.disparity_nominal_px = float(self.get_parameter('disparity_nominal_px').value)
        self.safe_stop_distance_m = float(self.get_parameter('safe_stop_distance_m').value)
        self.left_image_topic = str(self.get_parameter('left_image_topic').value)
        self.right_image_topic = str(self.get_parameter('right_image_topic').value)
        self.left_camera_info_topic = str(self.get_parameter('left_camera_info_topic').value)
        self.right_camera_info_topic = str(self.get_parameter('right_camera_info_topic').value)
        self.max_frame_age_sec = float(self.get_parameter('max_frame_age_sec').value)
        self.roi_top_fraction = float(self.get_parameter('roi_top_fraction').value)
        self.roi_bottom_fraction = float(self.get_parameter('roi_bottom_fraction').value)
        self.roi_left_fraction = float(self.get_parameter('roi_left_fraction').value)
        self.roi_right_fraction = float(self.get_parameter('roi_right_fraction').value)
        self.min_valid_depth_samples = max(20, int(self.get_parameter('min_valid_depth_samples').value))
        self.depth_percentile = max(1.0, min(99.0, float(self.get_parameter('depth_percentile').value)))
        self.speckle_window_size = max(0, int(self.get_parameter('speckle_window_size').value))
        self.speckle_range = max(0, int(self.get_parameter('speckle_range').value))

        self.rectified_ready = False
        self.camera_ready = False
        self.left_frame = None
        self.right_frame = None
        self.left_time = None
        self.right_time = None
        self.last_depth = None
        self.left_info_ready = False
        self.right_info_ready = False

        num_disparities = int(self.get_parameter('stereo_num_disparities').value)
        num_disparities = max(16, (num_disparities // 16) * 16)
        block_size = int(self.get_parameter('stereo_block_size').value)
        if block_size % 2 == 0:
            block_size += 1
        block_size = max(3, block_size)
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * block_size * block_size,
            P2=32 * block_size * block_size,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=self.speckle_window_size,
            speckleRange=self.speckle_range,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self.ready_pub = self.create_publisher(Bool, '/perception/depth/ready', 10)
        self.distance_pub = self.create_publisher(Float32, '/perception/depth/min_distance_m', 10)
        self.closest_offset_pub = self.create_publisher(Float32, '/perception/depth/closest_offset', 10)
        self.left_distance_pub = self.create_publisher(Float32, '/perception/depth/left_distance_m', 10)
        self.center_distance_pub = self.create_publisher(Float32, '/perception/depth/center_distance_m', 10)
        self.right_distance_pub = self.create_publisher(Float32, '/perception/depth/right_distance_m', 10)
        self.hazard_pub = self.create_publisher(Bool, '/perception/depth/hazard_close', 10)
        self.status_pub = self.create_publisher(String, '/perception/depth/status', 10)

        self.create_subscription(Bool, '/sensors/stereo/ready', self.camera_ready_cb, 10)
        self.create_subscription(Bool, '/sensors/stereo/rectified/ready', self.rectified_ready_cb, 10)
        self.create_subscription(Image, self.left_image_topic, self.left_image_cb, 10)
        self.create_subscription(Image, self.right_image_topic, self.right_image_cb, 10)
        self.create_subscription(CameraInfo, self.left_camera_info_topic, self.left_info_cb, 10)
        self.create_subscription(CameraInfo, self.right_camera_info_topic, self.right_info_cb, 10)

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

    def left_info_cb(self, msg: CameraInfo):
        if len(msg.k) >= 3 and msg.k[0] > 0.0:
            self.fx_px = float(msg.k[0])
            self.left_info_ready = True

    def right_info_cb(self, msg: CameraInfo):
        if len(msg.p) >= 4 and self.fx_px > 0.0 and msg.p[3] != 0.0:
            self.baseline_m = abs(float(msg.p[3])) / self.fx_px
        self.right_info_ready = True

    def image_to_gray(self, msg: Image):
        if msg.encoding not in ('bgr8', 'rgb8', 'mono8'):
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        if msg.encoding == 'mono8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
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

    def percentile_depth(self, disparities):
        valid = disparities[disparities >= self.disparity_min_px]
        if valid.size < max(5, self.min_valid_depth_samples // 6):
            return None
        depths = (self.fx_px * self.baseline_m) / valid
        depths = depths[np.isfinite(depths)]
        if depths.size < max(5, self.min_valid_depth_samples // 6):
            return None
        near_depth = float(np.percentile(depths, self.depth_percentile))
        return near_depth if near_depth > 0.0 else None

    def compute_depth_metrics(self):
        if self.left_frame is None or self.right_frame is None:
            return None
        if self.left_frame.shape != self.right_frame.shape:
            right = cv2.resize(self.right_frame, (self.left_frame.shape[1], self.left_frame.shape[0]))
        else:
            right = self.right_frame

        left = cv2.equalizeHist(self.left_frame)
        right = cv2.equalizeHist(right)
        disparity = self.stereo.compute(left, right).astype(np.float32) / 16.0
        height, width = disparity.shape[:2]
        y1 = int(max(0.0, min(0.95, self.roi_top_fraction)) * height)
        y2 = int(max(self.roi_top_fraction, min(1.0, self.roi_bottom_fraction)) * height)
        x1 = int(max(0.0, min(0.95, self.roi_left_fraction)) * width)
        x2 = int(max(self.roi_left_fraction, min(1.0, self.roi_right_fraction)) * width)
        roi = disparity[y1:y2, x1:x2]
        valid_mask = roi >= self.disparity_min_px
        valid = roi[valid_mask]
        if valid.size < self.min_valid_depth_samples:
            return None

        depths = (self.fx_px * self.baseline_m) / valid
        finite_mask = np.isfinite(depths)
        depths = depths[finite_mask]
        if depths.size < self.min_valid_depth_samples:
            return None
        near_depth = float(np.percentile(depths, self.depth_percentile))
        if near_depth <= 0.0:
            return None

        valid_ys, valid_xs = np.nonzero(valid_mask)
        valid_xs = valid_xs[finite_mask]
        close_threshold = float(np.percentile(depths, min(35.0, max(self.depth_percentile, 5.0))))
        close_xs = valid_xs[depths <= close_threshold]
        if close_xs.size > 0:
            closest_x_px = float(x1 + np.median(close_xs))
            closest_offset = (closest_x_px - (width / 2.0)) / max(width / 2.0, 1.0)
        else:
            closest_offset = 0.0

        roi_width = max(1, roi.shape[1])
        left_roi = roi[:, :roi_width // 3]
        center_roi = roi[:, roi_width // 3:(2 * roi_width) // 3]
        right_roi = roi[:, (2 * roi_width) // 3:]
        fallback_depth = float(near_depth)

        return {
            'min_distance_m': float(near_depth),
            'closest_offset': float(max(-1.0, min(1.0, closest_offset))),
            'left_distance_m': self.percentile_depth(left_roi) or fallback_depth,
            'center_distance_m': self.percentile_depth(center_roi) or fallback_depth,
            'right_distance_m': self.percentile_depth(right_roi) or fallback_depth,
        }

    def timer_callback(self):
        image_ready = self.frames_fresh()
        metrics = self.compute_depth_metrics() if image_ready else None
        info_ready = self.left_info_ready and self.right_info_ready
        depth_ready = (
            self.camera_ready and self.rectified_ready and info_ready and image_ready and metrics is not None
        )
        nominal_depth = self.compute_nominal_depth()
        min_depth_m = metrics['min_distance_m'] if metrics is not None else nominal_depth
        closest_offset = metrics['closest_offset'] if metrics is not None else 0.0
        left_distance_m = metrics['left_distance_m'] if metrics is not None else nominal_depth
        center_distance_m = metrics['center_distance_m'] if metrics is not None else nominal_depth
        right_distance_m = metrics['right_distance_m'] if metrics is not None else nominal_depth
        hazard_close = depth_ready and min_depth_m <= self.safe_stop_distance_m

        ready = Bool()
        ready.data = depth_ready
        self.ready_pub.publish(ready)

        distance = Float32()
        distance.data = float(min_depth_m)
        self.distance_pub.publish(distance)

        offset = Float32()
        offset.data = float(closest_offset)
        self.closest_offset_pub.publish(offset)

        left_distance = Float32()
        left_distance.data = float(left_distance_m)
        self.left_distance_pub.publish(left_distance)

        center_distance = Float32()
        center_distance.data = float(center_distance_m)
        self.center_distance_pub.publish(center_distance)

        right_distance = Float32()
        right_distance.data = float(right_distance_m)
        self.right_distance_pub.publish(right_distance)

        hazard = Bool()
        hazard.data = hazard_close
        self.hazard_pub.publish(hazard)

        status = String()
        status.data = (
            f'ready={depth_ready}, camera_ready={self.camera_ready}, rectified_ready={self.rectified_ready}, '
            f'info_ready={info_ready}, images_fresh={image_ready}, left_age_sec={self.seconds_since(self.left_time):.2f}, '
            f'right_age_sec={self.seconds_since(self.right_time):.2f}, min_distance_m={min_depth_m:.2f}, '
            f'offset={closest_offset:.2f}, sectors=({left_distance_m:.2f},{center_distance_m:.2f},{right_distance_m:.2f}), '
            f'safe_stop_distance_m={self.safe_stop_distance_m:.2f}, fx_px={self.fx_px:.1f}, baseline_m={self.baseline_m:.3f}'
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
