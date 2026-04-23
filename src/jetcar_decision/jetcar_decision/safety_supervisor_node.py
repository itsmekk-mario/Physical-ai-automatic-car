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

        self.emergency_distance_m = float(self.get_parameter('emergency_distance_m').value)
        self.lane_assist_gain = float(self.get_parameter('lane_assist_gain').value)
        self.ai_intervention_person_stop = bool(self.get_parameter('ai_intervention_person_stop').value)

        self.depth_hazard = False
        self.detection_hazard = False
        self.person_detected = False
        self.min_distance_m = 99.0
        self.lane_suggested_steering = 0.0
        self.drive_mode = 'MANUAL'

        self.override_pub = self.create_publisher(Bool, '/system/safety_override_active', 10)
        self.ai_throttle_pub = self.create_publisher(Float32, '/input/ai/throttle', 10)
        self.ai_steering_pub = self.create_publisher(Float32, '/input/ai/steering', 10)
        self.reason_pub = self.create_publisher(String, '/system/safety_override_reason', 10)
        self.estop_pub = self.create_publisher(Bool, '/system/estop_cmd', 10)

        self.create_subscription(Bool, '/perception/depth/hazard_close', self.depth_hazard_cb, 10)
        self.create_subscription(Float32, '/perception/depth/min_distance_m', self.min_distance_cb, 10)
        self.create_subscription(Bool, '/perception/detections/hazard', self.detection_hazard_cb, 10)
        self.create_subscription(Bool, '/perception/detections/person_detected', self.person_detected_cb, 10)
        self.create_subscription(Float32, '/perception/lane/suggested_steering', self.lane_suggested_cb, 10)
        self.create_subscription(String, '/system/drive_mode', self.drive_mode_cb, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'safety_supervisor_node started | emergency_distance_m={self.emergency_distance_m:.2f}'
        )

    def depth_hazard_cb(self, msg: Bool):
        self.depth_hazard = bool(msg.data)

    def min_distance_cb(self, msg: Float32):
        self.min_distance_m = float(msg.data)

    def detection_hazard_cb(self, msg: Bool):
        self.detection_hazard = bool(msg.data)

    def person_detected_cb(self, msg: Bool):
        self.person_detected = bool(msg.data)

    def lane_suggested_cb(self, msg: Float32):
        self.lane_suggested_steering = float(msg.data)

    def drive_mode_cb(self, msg: String):
        self.drive_mode = str(msg.data).upper().strip()

    def timer_callback(self):
        override_active = self.depth_hazard or self.detection_hazard
        reason = 'clear'
        throttle_cmd = 0.0
        steering_cmd = 0.0
        person_stop_active = (
            self.ai_intervention_person_stop
            and self.drive_mode == 'AI_INTERVENTION'
            and self.person_detected
        )

        if override_active:
            if self.depth_hazard:
                reason = f'depth_hazard distance={self.min_distance_m:.2f}m'
            elif self.detection_hazard:
                reason = 'object_detection_hazard'
            throttle_cmd = 0.0
            steering_cmd = max(-1.0, min(1.0, self.lane_assist_gain * self.lane_suggested_steering))
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
