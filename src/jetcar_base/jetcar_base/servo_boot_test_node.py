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
        self.sequence = [
            (self.center, 0.3),
            (self.left_test, 0.5),
            (self.right_test, 0.5),
            (self.center, 0.3),
        ]
        self.sequence_index = 0
        self.step_timer = None

        self.get_logger().info('SERVO_BOOT_TEST_START')
        self.run_next_step()

    def move(self, tick):
        self.pca.set_pwm(0, 0, tick)
        self.get_logger().info(f'MOVE tick={tick}')

    def run_next_step(self):
        if self.sequence_index >= len(self.sequence):
            self.get_logger().info('SERVO_BOOT_TEST_DONE')
            return

        tick, delay_sec = self.sequence[self.sequence_index]
        self.sequence_index += 1
        self.move(tick)
        if self.sequence_index < len(self.sequence):
            self.step_timer = super().create_timer(delay_sec, self._timer_callback_once)
        else:
            self.get_logger().info('SERVO_BOOT_TEST_DONE')

    def _timer_callback_once(self):
        if self.step_timer is not None:
            self.step_timer.cancel()
            self.destroy_timer(self.step_timer)
            self.step_timer = None
        self.run_next_step()

    def destroy_node(self):
        try:
            if self.step_timer is not None:
                self.step_timer.cancel()
                self.destroy_timer(self.step_timer)
                self.step_timer = None
            self.pca.set_pwm(0, 0, self.center)
            self.pca.close()
        except Exception as e:
            self.get_logger().error(f'cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoBootTestNode()
        while rclpy.ok() and node.sequence_index < len(node.sequence):
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print(f'servo_boot_test_node failed to start: {e}')
    finally:
        if node is not None:
            node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
