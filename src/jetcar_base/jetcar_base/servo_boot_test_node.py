import time
import rclpy
from rclpy.node import Node
from jetcar_base.pca9685_driver import PCA9685


class ServoBootTestNode(Node):
    def __init__(self):
        super().__init__('servo_boot_test_node')

        self.pca = PCA9685(bus_num=7, address=0x40, pwm_freq=50.0)

        self.center = 323
        self.left_test = 260
        self.right_test = 380

        self.get_logger().info('SERVO_BOOT_TEST_START')

        self.move(self.center)
        time.sleep(0.3)

        self.move(self.left_test)
        time.sleep(0.5)

        self.move(self.right_test)
        time.sleep(0.5)

        self.move(self.center)
        time.sleep(0.3)

        self.get_logger().info('SERVO_BOOT_TEST_DONE')

    def move(self, tick):
        self.pca.set_pwm(0, 0, tick)
        self.get_logger().info(f'MOVE tick={tick}')

    def destroy_node(self):
        try:
            self.pca.set_pwm(0, 0, self.center)
            time.sleep(0.2)
            self.pca.close()
        except Exception as e:
            self.get_logger().error(f'cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoBootTestNode()
    except Exception as e:
        print(f'servo_boot_test_node failed to start: {e}')
    finally:
        if node is not None:
            node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
