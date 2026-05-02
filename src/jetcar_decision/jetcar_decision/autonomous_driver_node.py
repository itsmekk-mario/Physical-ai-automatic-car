import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class AutonomousDriverNode(Node):
    """Conservative autonomy policy for the Jetcar platform.

    This node does not claim production vehicle safety. It applies production-style
    design habits that are appropriate for this ROS2 stack: explicit states,
    stale-input shutdown, distance envelopes, command rate limits, and clear status
    reporting.
    """

    def __init__(self):
        super().__init__('autonomous_driver_node')

        self.declare_parameter('cruise_throttle', 0.08)
        self.declare_parameter('caution_throttle', 0.035)
        self.declare_parameter('lane_lost_throttle', 0.025)
        self.declare_parameter('hazard_stop_distance_m', 0.8)
        self.declare_parameter('slow_distance_m', 1.5)
        self.declare_parameter('clear_distance_m', 2.2)
        self.declare_parameter('center_blocked_distance_m', 1.0)
        self.declare_parameter('closing_speed_stop_mps', 0.45)
        self.declare_parameter('closing_speed_slow_mps', 0.18)
        self.declare_parameter('closing_speed_alpha', 0.35)
        self.declare_parameter('lane_follow_gain', 1.0)
        self.declare_parameter('lane_confidence_min', 0.25)
        self.declare_parameter('lane_confidence_full', 0.65)
        self.declare_parameter('curve_slow_steering', 0.55)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('enabled_on_start', False)
        self.declare_parameter('startup_hold_sec', 2.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('max_throttle_step', 0.01)
        self.declare_parameter('max_brake_step', 0.04)
        self.declare_parameter('max_steering_step', 0.08)
        self.declare_parameter('max_steering_cmd', 0.90)
        self.declare_parameter('autonomy_level', 2)
        self.declare_parameter('require_depth', True)
        self.declare_parameter('require_lane', False)
        self.declare_parameter('lane_follow_min_level', 2)
        self.declare_parameter('use_detection_hazard', True)
        self.declare_parameter('person_stop_enabled', True)

        self.cruise_throttle = self.clamp_positive(self.get_parameter('cruise_throttle').value)
        self.caution_throttle = self.clamp_positive(self.get_parameter('caution_throttle').value)
        self.lane_lost_throttle = self.clamp_positive(self.get_parameter('lane_lost_throttle').value)
        self.hazard_stop_distance_m = max(0.05, float(self.get_parameter('hazard_stop_distance_m').value))
        self.slow_distance_m = max(
            self.hazard_stop_distance_m + 0.05,
            float(self.get_parameter('slow_distance_m').value),
        )
        self.clear_distance_m = max(
            self.slow_distance_m + 0.05,
            float(self.get_parameter('clear_distance_m').value),
        )
        self.center_blocked_distance_m = max(0.05, float(self.get_parameter('center_blocked_distance_m').value))
        self.closing_speed_stop_mps = max(0.0, float(self.get_parameter('closing_speed_stop_mps').value))
        self.closing_speed_slow_mps = max(0.0, float(self.get_parameter('closing_speed_slow_mps').value))
        self.closing_speed_alpha = max(0.0, min(1.0, float(self.get_parameter('closing_speed_alpha').value)))
        self.lane_follow_gain = float(self.get_parameter('lane_follow_gain').value)
        self.lane_confidence_min = max(0.0, min(1.0, float(self.get_parameter('lane_confidence_min').value)))
        self.lane_confidence_full = max(
            self.lane_confidence_min + 0.01,
            min(1.0, float(self.get_parameter('lane_confidence_full').value)),
        )
        self.curve_slow_steering = max(0.0, min(1.0, float(self.get_parameter('curve_slow_steering').value)))
        self.enabled = bool(self.get_parameter('enabled_on_start').value)
        self.startup_hold_sec = max(0.0, float(self.get_parameter('startup_hold_sec').value))
        self.input_timeout_sec = max(0.05, float(self.get_parameter('input_timeout_sec').value))
        self.max_throttle_step = abs(float(self.get_parameter('max_throttle_step').value))
        self.max_brake_step = abs(float(self.get_parameter('max_brake_step').value))
        self.max_steering_step = abs(float(self.get_parameter('max_steering_step').value))
        self.max_steering_cmd = max(0.0, min(1.0, float(self.get_parameter('max_steering_cmd').value)))
        self.autonomy_level = max(2, int(self.get_parameter('autonomy_level').value))
        self.require_depth = bool(self.get_parameter('require_depth').value)
        self.require_lane = bool(self.get_parameter('require_lane').value)
        self.lane_follow_min_level = max(2, int(self.get_parameter('lane_follow_min_level').value))
        self.use_detection_hazard = bool(self.get_parameter('use_detection_hazard').value)
        self.person_stop_enabled = bool(self.get_parameter('person_stop_enabled').value)

        self.depth_ready = False
        self.lane_ready = False
        self.detection_ready = False
        self.min_distance_m = 99.0
        self.left_distance_m = 99.0
        self.center_distance_m = 99.0
        self.right_distance_m = 99.0
        self.previous_distance_m = None
        self.previous_distance_time = None
        self.relative_closing_speed_mps = 0.0
        self.lane_confidence = 0.0
        self.lane_suggested_steering = 0.0
        self.detection_hazard = False
        self.person_detected = False
        self.current_throttle_cmd = 0.0
        self.current_steering_cmd = 0.0
        self.start_time = self.get_clock().now()
        self.last_depth_time = None
        self.last_lane_time = None
        self.last_detection_time = None

        self.pub_throttle = self.create_publisher(Float32, '/input/autonomy/throttle', 10)
        self.pub_steering = self.create_publisher(Float32, '/input/autonomy/steering', 10)
        self.pub_status = self.create_publisher(String, '/autonomy/status', 10)

        self.create_subscription(Bool, '/perception/depth/ready', self.depth_ready_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/left_distance_m', self.left_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/center_distance_m', self.center_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/right_distance_m', self.right_distance_cb, 10)
        self.create_subscription(Bool, '/perception/lane/ready', self.lane_ready_cb, 10)
        self.create_subscription(Float32, '/perception/lane/confidence', self.lane_confidence_cb, 10)
        self.create_subscription(Float32, '/perception/lane/suggested_steering', self.suggested_steering_cb, 10)
        self.create_subscription(Bool, '/perception/detections/ready', self.detection_ready_cb, 10)
        self.create_subscription(Bool, '/perception/detections/hazard', self.detection_hazard_cb, 10)
        self.create_subscription(Bool, '/perception/detections/person_detected', self.person_detected_cb, 10)
        self.create_subscription(Bool, '/system/autonomy_enable', self.enable_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'autonomous_driver_node started | '
            f'level={self.autonomy_level}, cruise_throttle={self.cruise_throttle:.2f}, '
            f'stop_distance={self.hazard_stop_distance_m:.2f}, require_depth={self.require_depth}, '
            f'require_lane={self.require_lane}, enabled={self.enabled}'
        )

    def depth_ready_cb(self, msg: Bool):
        self.depth_ready = bool(msg.data)
        self.last_depth_time = self.get_clock().now()

    def mark_depth_time(self):
        self.last_depth_time = self.get_clock().now()

    def min_distance_cb(self, msg: Float32):
        distance = max(0.0, float(msg.data))
        now = self.get_clock().now()
        if self.previous_distance_m is not None and self.previous_distance_time is not None:
            dt = (now - self.previous_distance_time).nanoseconds / 1e9
            if dt > 0.02:
                closing_speed = max(0.0, (self.previous_distance_m - distance) / dt)
                self.relative_closing_speed_mps = (
                    self.closing_speed_alpha * closing_speed
                    + (1.0 - self.closing_speed_alpha) * self.relative_closing_speed_mps
                )
        self.previous_distance_m = distance
        self.previous_distance_time = now
        self.min_distance_m = distance
        self.last_depth_time = now

    def left_distance_cb(self, msg: Float32):
        self.left_distance_m = max(0.0, float(msg.data))
        self.mark_depth_time()

    def center_distance_cb(self, msg: Float32):
        self.center_distance_m = max(0.0, float(msg.data))
        self.mark_depth_time()

    def right_distance_cb(self, msg: Float32):
        self.right_distance_m = max(0.0, float(msg.data))
        self.mark_depth_time()

    def lane_ready_cb(self, msg: Bool):
        self.lane_ready = bool(msg.data)
        self.last_lane_time = self.get_clock().now()

    def lane_confidence_cb(self, msg: Float32):
        self.lane_confidence = max(0.0, min(1.0, float(msg.data)))
        self.last_lane_time = self.get_clock().now()

    def suggested_steering_cb(self, msg: Float32):
        self.lane_suggested_steering = self.clamp(float(msg.data))
        self.last_lane_time = self.get_clock().now()

    def detection_ready_cb(self, msg: Bool):
        self.detection_ready = bool(msg.data)
        self.last_detection_time = self.get_clock().now()

    def detection_hazard_cb(self, msg: Bool):
        self.detection_hazard = bool(msg.data)
        self.last_detection_time = self.get_clock().now()

    def person_detected_cb(self, msg: Bool):
        self.person_detected = bool(msg.data)
        self.last_detection_time = self.get_clock().now()

    def enable_cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.current_throttle_cmd = 0.0
            self.current_steering_cmd = 0.0
        self.get_logger().warn(f'autonomy_enable={self.enabled}')

    @staticmethod
    def clamp(value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def clamp_positive(value: float, limit: float = 1.0) -> float:
        return max(0.0, min(limit, float(value)))

    def seconds_since(self, stamp):
        if stamp is None:
            return 999.0
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def fresh(self, stamp) -> bool:
        return self.seconds_since(stamp) <= self.input_timeout_sec

    def ramp_value(self, current: float, target: float, up_step: float, down_step: float) -> float:
        delta = float(target) - float(current)
        if delta >= 0.0:
            step = max(0.0, up_step)
        else:
            step = max(0.0, down_step)
        if abs(delta) <= step:
            return float(target)
        return current + step if delta > 0.0 else current - step

    def distance_throttle_scale(self, distance_m: float) -> float:
        if distance_m <= self.hazard_stop_distance_m:
            return 0.0
        if distance_m >= self.clear_distance_m:
            return 1.0
        usable_range = self.clear_distance_m - self.hazard_stop_distance_m
        return max(0.0, min(1.0, (distance_m - self.hazard_stop_distance_m) / usable_range))

    def lane_confidence_scale(self) -> float:
        if self.lane_confidence <= self.lane_confidence_min:
            return 0.0
        usable_range = self.lane_confidence_full - self.lane_confidence_min
        return max(0.0, min(1.0, (self.lane_confidence - self.lane_confidence_min) / usable_range))

    def curve_throttle_scale(self, steering_cmd: float) -> float:
        magnitude = abs(steering_cmd)
        if magnitude <= self.curve_slow_steering:
            return 1.0
        usable_range = max(1.0 - self.curve_slow_steering, 0.01)
        excess = min(1.0, (magnitude - self.curve_slow_steering) / usable_range)
        return max(0.45, 1.0 - 0.55 * excess)

    def compute_steering_target(self, lane_available: bool) -> float:
        if not lane_available:
            return 0.0
        steering = self.lane_follow_gain * self.lane_suggested_steering
        return self.clamp(steering, self.max_steering_cmd)

    def compute_policy(self):
        uptime = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        depth_fresh = self.fresh(self.last_depth_time)
        lane_fresh = self.fresh(self.last_lane_time)
        detection_fresh = self.fresh(self.last_detection_time)
        depth_available = depth_fresh and self.depth_ready
        lane_follow_enabled = self.autonomy_level >= self.lane_follow_min_level
        lane_available = (
            lane_follow_enabled
            and lane_fresh
            and self.lane_ready
            and self.lane_confidence >= self.lane_confidence_min
        )
        detection_available = detection_fresh and self.detection_ready
        person_stop = self.person_stop_enabled and detection_fresh and self.person_detected
        detection_stop = self.use_detection_hazard and detection_fresh and self.detection_hazard

        if not self.enabled:
            return 'DISABLED', 'autonomy disabled', 0.0, 0.0, depth_fresh, lane_available, detection_available
        if uptime < self.startup_hold_sec:
            return 'STARTUP_HOLD', f'startup hold {uptime:.1f}s', 0.0, 0.0, depth_fresh, lane_available, detection_available
        if self.require_depth and not depth_available:
            return 'FAULT_STOP', 'depth input stale or not ready', 0.0, 0.0, depth_fresh, lane_available, detection_available
        if person_stop:
            return 'OBJECT_STOP', 'person detected', 0.0, 0.0, depth_fresh, lane_available, detection_available
        if detection_stop:
            return 'OBJECT_STOP', 'detection hazard', 0.0, 0.0, depth_fresh, lane_available, detection_available

        stop_for_distance = depth_available and self.min_distance_m <= self.hazard_stop_distance_m
        stop_for_closing = (
            depth_available
            and self.min_distance_m <= self.slow_distance_m
            and self.relative_closing_speed_mps >= self.closing_speed_stop_mps
        )
        if stop_for_distance or stop_for_closing:
            reason = (
                f'depth stop distance={self.min_distance_m:.2f}m '
                f'closing={self.relative_closing_speed_mps:.2f}mps'
            )
            return 'HAZARD_STOP', reason, 0.0, 0.0, depth_fresh, lane_available, detection_available
        if self.require_lane and lane_follow_enabled and not lane_available:
            return 'LANE_LOSS_STOP', 'required lane input unavailable', 0.0, 0.0, depth_fresh, lane_available, detection_available

        steering_target = self.compute_steering_target(lane_available)
        throttle_target = self.cruise_throttle
        state = 'CRUISE'
        reason = 'clear'

        if depth_available:
            distance_scale = self.distance_throttle_scale(self.min_distance_m)
            if distance_scale < 1.0:
                state = 'CAUTION'
                reason = f'distance envelope distance={self.min_distance_m:.2f}m'
                throttle_target *= distance_scale
            if self.center_distance_m <= self.center_blocked_distance_m:
                state = 'CAUTION'
                reason = f'center sector blocked distance={self.center_distance_m:.2f}m'
                throttle_target = min(throttle_target, self.caution_throttle)
            if self.relative_closing_speed_mps >= self.closing_speed_slow_mps:
                state = 'CAUTION'
                reason = f'closing speed={self.relative_closing_speed_mps:.2f}mps'
                throttle_target = min(throttle_target, self.caution_throttle)

        if lane_follow_enabled and not lane_available:
            state = 'DEGRADED'
            reason = 'lane unavailable, straight crawl only'
            throttle_target = min(throttle_target, self.lane_lost_throttle)
        elif lane_follow_enabled:
            lane_scale = self.lane_confidence_scale()
            throttle_target *= max(0.35, lane_scale)
            if lane_scale < 1.0 and state == 'CRUISE':
                state = 'CAUTION'
                reason = f'low lane confidence={self.lane_confidence:.2f}'

        throttle_target *= self.curve_throttle_scale(steering_target)

        return (
            state,
            reason,
            self.clamp_positive(throttle_target, self.cruise_throttle),
            steering_target,
            depth_fresh,
            lane_available,
            detection_available,
        )

    def timer_callback(self):
        (
            state,
            reason,
            throttle_target,
            steering_target,
            depth_fresh,
            lane_available,
            detection_available,
        ) = self.compute_policy()

        self.current_throttle_cmd = self.ramp_value(
            self.current_throttle_cmd,
            throttle_target,
            self.max_throttle_step,
            self.max_brake_step,
        )
        if throttle_target <= 0.0:
            self.current_throttle_cmd = 0.0
        self.current_steering_cmd = self.ramp_value(
            self.current_steering_cmd,
            steering_target,
            self.max_steering_step,
            self.max_steering_step,
        )
        self.current_steering_cmd = self.clamp(self.current_steering_cmd, self.max_steering_cmd)

        throttle = Float32()
        throttle.data = float(self.current_throttle_cmd)
        self.pub_throttle.publish(throttle)

        steering = Float32()
        steering.data = float(self.current_steering_cmd)
        self.pub_steering.publish(steering)

        status = String()
        status.data = (
            f'state={state}, reason={reason}, level={self.autonomy_level}, enabled={self.enabled}, '
            f'depth_ready={self.depth_ready}, depth_fresh={depth_fresh}, '
            f'lane_ready={self.lane_ready}, lane_available={lane_available}, lane_confidence={self.lane_confidence:.2f}, '
            f'detection_available={detection_available}, detection_hazard={self.detection_hazard}, '
            f'person_detected={self.person_detected}, throttle={self.current_throttle_cmd:.2f}, '
            f'target_throttle={throttle_target:.2f}, steering={self.current_steering_cmd:.2f}, '
            f'target_steering={steering_target:.2f}, min_distance_m={self.min_distance_m:.2f}, '
            f'sectors=({self.left_distance_m:.2f},{self.center_distance_m:.2f},{self.right_distance_m:.2f}), '
            f'closing_speed_mps={self.relative_closing_speed_mps:.2f}'
        )
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
