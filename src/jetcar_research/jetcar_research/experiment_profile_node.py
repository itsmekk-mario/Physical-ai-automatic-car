import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class ExperimentProfileNode(Node):
    def __init__(self):
        super().__init__('experiment_profile_node')

        self.declare_parameter('profile_name', 'camera_only')
        self.declare_parameter('v2x_enabled', False)
        self.declare_parameter('lidar_enabled', False)
        self.declare_parameter('publish_rate_hz', 2.0)

        self.profile_name = str(self.get_parameter('profile_name').value)
        self.v2x_enabled = bool(self.get_parameter('v2x_enabled').value)
        self.lidar_enabled = bool(self.get_parameter('lidar_enabled').value)

        self.profile_pub = self.create_publisher(String, '/research/profile', 10)
        self.v2x_pub = self.create_publisher(Bool, '/research/v2x_enabled', 10)
        self.lidar_pub = self.create_publisher(Bool, '/research/lidar_enabled', 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'experiment_profile_node started | profile={self.profile_name}, '
            f'v2x_enabled={self.v2x_enabled}, lidar_enabled={self.lidar_enabled}'
        )

    def timer_callback(self):
        profile = String()
        profile.data = self.profile_name
        self.profile_pub.publish(profile)

        v2x = Bool()
        v2x.data = self.v2x_enabled
        self.v2x_pub.publish(v2x)

        lidar = Bool()
        lidar.data = self.lidar_enabled
        self.lidar_pub.publish(lidar)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
