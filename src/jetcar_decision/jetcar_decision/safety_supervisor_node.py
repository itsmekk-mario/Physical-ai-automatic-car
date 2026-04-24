import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor_node')

        self.declare_parameter('emergency_distance_m', 0.8)
        self.declare_parameter('lane_assist_gain', 0.8)
        self.declare_parameter('ai_intervention_person_stop', True)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('autonomy_level', 1)
        self.declare_parameter('avoidance_enabled', True)
        self.declare_parameter('obstacle_reaction_distance_m', 1.4)
        self.declare_parameter('obstacle_stop_distance_m', 0.55)
        self.declare_parameter('moving_obstacle_speed_threshold_mps', 0.15)
        self.declare_parameter('avoidance_gain', 1.1)
        self.declare_parameter('lane_blend_gain', 0.35)
        self.declare_parameter('max_avoidance_steering', 0.85)
        self.declare_parameter('moving_obstacle_throttle', 0.03)
        self.declare_parameter('static_obstacle_throttle', 0.04)
        self.declare_parameter('avoidance_offset_deadband', 0.08)
        self.declare_parameter('depth_speed_alpha', 0.4)
        self.declare_parameter('dynamic_reaction_time_sec', 1.2)
        self.declare_parameter('max_dynamic_reaction_distance_m', 2.2)

        self.emergency_distance_m = float(self.get_parameter('emergency_distance_m').value)
        self.lane_assist_gain = float(self.get_parameter('lane_assist_gain').value)
        self.ai_intervention_person_stop = bool(self.get_parameter('ai_intervention_person_stop').value)
        self.autonomy_level = max(1, int(self.get_parameter('autonomy_level').value))
        self.avoidance_enabled = bool(self.get_parameter('avoidance_enabled').value)
        self.obstacle_reaction_distance_m = float(self.get_parameter('obstacle_reaction_distance_m').value)
        self.obstacle_stop_distance_m = float(self.get_parameter('obstacle_stop_distance_m').value)
        self.moving_obstacle_speed_threshold_mps = float(
            self.get_parameter('moving_obstacle_speed_threshold_mps').value
        )
        self.avoidance_gain = float(self.get_parameter('avoidance_gain').value)
        self.lane_blend_gain = float(self.get_parameter('lane_blend_gain').value)
        self.max_avoidance_steering = float(self.get_parameter('max_avoidance_steering').value)
        self.moving_obstacle_throttle = float(self.get_parameter('moving_obstacle_throttle').value)
        self.static_obstacle_throttle = float(self.get_parameter('static_obstacle_throttle').value)
        self.avoidance_offset_deadband = float(self.get_parameter('avoidance_offset_deadband').value)
        self.depth_speed_alpha = float(self.get_parameter('depth_speed_alpha').value)
        self.dynamic_reaction_time_sec = float(self.get_parameter('dynamic_reaction_time_sec').value)
        self.max_dynamic_reaction_distance_m = float(self.get_parameter('max_dynamic_reaction_distance_m').value)

        self.depth_ready = False
        self.depth_hazard = False
        self.detection_hazard = False
        self.person_detected = False
        self.min_distance_m = 99.0
        self.depth_closest_offset = 0.0
        self.left_distance_m = 99.0
        self.center_distance_m = 99.0
        self.right_distance_m = 99.0
        self.lane_suggested_steering = 0.0
        self.closest_detection_offset = 0.0
        self.closest_detection_area = 0.0
        self.relative_speed_mps = 0.0
        self.previous_distance_m = None
        self.previous_distance_time = None
        self.drive_mode = 'MANUAL'

        self.override_pub = self.create_publisher(Bool, '/system/safety_override_active', 10)
        self.ai_throttle_pub = self.create_publisher(Float32, '/input/ai/throttle', 10)
        self.ai_steering_pub = self.create_publisher(Float32, '/input/ai/steering', 10)
        self.reason_pub = self.create_publisher(String, '/system/safety_override_reason', 10)
        self.estop_pub = self.create_publisher(Bool, '/system/estop_cmd', 10)

        self.create_subscription(Bool, '/perception/depth/ready', self.depth_ready_cb, 10)
        self.create_subscription(Bool, '/perception/depth/hazard_close', self.depth_hazard_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/closest_offset', self.depth_closest_offset_cb, 10)
        self.create_subscription(Float32, '/perception/depth/left_distance_m', self.left_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/center_distance_m', self.center_distance_cb, 10)
        self.create_subscription(Float32, '/perception/depth/right_distance_m', self.right_distance_cb, 10)
        self.create_subscription(Bool, '/perception/detections/hazard', self.detection_hazard_cb, 10)
        self.create_subscription(Bool, '/perception/detections/person_detected', self.person_detected_cb, 10)
        self.create_subscription(Float32, '/perception/detections/closest_offset', self.closest_offset_cb, 10)
        self.create_subscription(Float32, '/perception/detections/closest_area_ratio', self.closest_area_cb, 10)
        self.create_subscription(Float32, '/perception/lane/suggested_steering', self.lane_suggested_cb, 10)
        self.create_subscription(String, '/system/drive_mode', self.drive_mode_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            'safety_supervisor_node started | '
            f'level={self.autonomy_level}, emergency_distance_m={self.emergency_distance_m:.2f}, '
            f'reaction_distance_m={self.obstacle_reaction_distance_m:.2f}'
        )

    def depth_ready_cb(self, msg: Bool):
        self.depth_ready = bool(msg.data)

    def depth_hazard_cb(self, msg: Bool):
        self.depth_hazard = bool(msg.data)

    def min_distance_cb(self, msg: Float32):
        distance = float(msg.data)
        now = self.get_clock().now()
        if self.previous_distance_m is not None and self.previous_distance_time is not None:
            dt = (now - self.previous_distance_time).nanoseconds / 1e9
            if dt > 0.02:
                closing_speed = max(0.0, (self.previous_distance_m - distance) / dt)
                self.relative_speed_mps = (
                    self.depth_speed_alpha * closing_speed
                    + (1.0 - self.depth_speed_alpha) * self.relative_speed_mps
                )
        self.previous_distance_m = distance
        self.previous_distance_time = now
        self.min_distance_m = distance

    def depth_closest_offset_cb(self, msg: Float32):
        self.depth_closest_offset = self.clamp(msg.data)

    def left_distance_cb(self, msg: Float32):
        self.left_distance_m = float(msg.data)

    def center_distance_cb(self, msg: Float32):
        self.center_distance_m = float(msg.data)

    def right_distance_cb(self, msg: Float32):
        self.right_distance_m = float(msg.data)

    def detection_hazard_cb(self, msg: Bool):
        self.detection_hazard = bool(msg.data)

    def person_detected_cb(self, msg: Bool):
        self.person_detected = bool(msg.data)

    def closest_offset_cb(self, msg: Float32):
        self.closest_detection_offset = float(msg.data)

    def closest_area_cb(self, msg: Float32):
        self.closest_detection_area = float(msg.data)

    def lane_suggested_cb(self, msg: Float32):
        self.lane_suggested_steering = float(msg.data)

    def drive_mode_cb(self, msg: String):
        self.drive_mode = str(msg.data).upper().strip()

    def clamp(self, value: float, limit: float = 1.0) -> float:
        return max(-limit, min(limit, float(value)))

    def detection_avoidance_available(self) -> bool:
        return self.closest_detection_area > 0.0 and abs(self.closest_detection_offset) >= self.avoidance_offset_deadband

    def depth_avoidance_available(self) -> bool:
        return self.depth_ready and self.min_distance_m <= self.effective_reaction_distance()

    def effective_reaction_distance(self) -> float:
        if self.autonomy_level >= 4:
            dynamic_distance = self.obstacle_reaction_distance_m + (
                self.relative_speed_mps * max(0.0, self.dynamic_reaction_time_sec)
            )
            return min(self.max_dynamic_reaction_distance_m, max(self.obstacle_reaction_distance_m, dynamic_distance))
        return self.obstacle_reaction_distance_m

    def primary_obstacle_offset(self) -> float:
        if self.closest_detection_area > 0.0:
            return self.closest_detection_offset
        return self.depth_closest_offset

    def clearance_steering(self) -> float:
        clearance_delta = self.right_distance_m - self.left_distance_m
        if abs(clearance_delta) < 0.05:
            return 0.0
        return self.max_avoidance_steering if clearance_delta > 0.0 else -self.max_avoidance_steering

    def compute_avoidance_steering(self) -> float:
        obstacle_term = 0.0
        obstacle_offset = self.primary_obstacle_offset()
        if abs(obstacle_offset) >= self.avoidance_offset_deadband:
            obstacle_term = -self.avoidance_gain * obstacle_offset
        elif self.depth_avoidance_available():
            obstacle_term = self.clearance_steering()
        lane_term = self.lane_blend_gain * self.lane_suggested_steering
        return self.clamp(obstacle_term + lane_term, self.max_avoidance_steering)

    def timer_callback(self):
        reaction_distance = self.effective_reaction_distance()
        depth_stop_active = (
            self.autonomy_level >= 2
            and self.depth_ready
            and (self.depth_hazard or self.min_distance_m <= self.emergency_distance_m)
        )
        avoidance_enabled = self.autonomy_level >= 3 and self.avoidance_enabled
        reaction_active = (
            avoidance_enabled
            and self.depth_ready
            and self.min_distance_m <= reaction_distance
        )
        override_active = depth_stop_active or reaction_active
        reason = 'clear'
        throttle_cmd = 0.0
        steering_cmd = 0.0
        moving_obstacle = self.relative_speed_mps >= self.moving_obstacle_speed_threshold_mps
        person_stop_active = self.ai_intervention_person_stop and self.person_detected

        if override_active:
            can_avoid = avoidance_enabled and self.min_distance_m > self.obstacle_stop_distance_m
            if depth_stop_active:
                reason = f'depth_hazard distance={self.min_distance_m:.2f}m'
            elif reaction_active:
                reason = f'depth_reaction distance={self.min_distance_m:.2f}m'
            if can_avoid and self.min_distance_m > self.obstacle_stop_distance_m:
                steering_cmd = self.compute_avoidance_steering()
                if abs(steering_cmd) < 0.05:
                    throttle_cmd = 0.0
                    reason = f'hold_no_clearance distance={self.min_distance_m:.2f}m'
                else:
                    throttle_cmd = self.moving_obstacle_throttle if moving_obstacle else self.static_obstacle_throttle
                obstacle_type = 'moving' if moving_obstacle else 'static'
                if throttle_cmd > 0.0:
                    reason = (
                        f'avoid_{obstacle_type} distance={self.min_distance_m:.2f}m '
                        f'offset={self.primary_obstacle_offset():.2f} rel_speed={self.relative_speed_mps:.2f} '
                        f'sectors=({self.left_distance_m:.2f},{self.center_distance_m:.2f},{self.right_distance_m:.2f})'
                    )
            else:
                throttle_cmd = 0.0
                steering_cmd = self.clamp(self.lane_assist_gain * self.lane_suggested_steering)
        if person_stop_active:
            reason = 'person_detected_ai_intervention_stop'
            throttle_cmd = 0.0
            steering_cmd = 0.0

        override = Bool()
        override.data = override_active
        self.override_pub.publish(override)

        throttle = Float32()
        throttle.data = throttle_cmd
        self.ai_throttle_pub.publish(throttle)

        steering = Float32()
        steering.data = steering_cmd
        self.ai_steering_pub.publish(steering)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

        estop_msg = Bool()
        estop_msg.data = person_stop_active
        self.estop_pub.publish(estop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
