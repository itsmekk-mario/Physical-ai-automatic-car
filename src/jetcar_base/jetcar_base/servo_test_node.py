import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from jetcar_base.pca9685_driver import PCA9685


class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')

        self.pca = PCA9685(bus_num=7, address=0x40, pwm_freq=50.0)

        self.servo_channel = 0

        # 캘리브레이션 값
        self.servo_center_tick = 323
        self.servo_min_tick = 236
        self.servo_max_tick = 410

        # 입력 1번당 목표 변화량
        self.command_step_tick = 5

        # 실제 이동은 천천히
        self.slew_tick_per_cycle = 1

        self.current_tick = self.servo_center_tick
        self.target_tick = self.servo_center_tick

        self.sub = self.create_subscription(
            Int32,
            '/vehicle/steering_step',
            self.step_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.apply_tick(self.current_tick)
        self.get_logger().info(
            f'servo_test_node started | center={self.servo_center_tick}, '
            f'min={self.servo_min_tick}, max={self.servo_max_tick}'
        )

    def clamp_tick(self, tick: int) -> int:
        return max(self.servo_min_tick, min(self.servo_max_tick, int(tick)))

    def apply_tick(self, tick: int):
        tick = self.clamp_tick(tick)
        self.current_tick = tick
        self.pca.set_pwm(self.servo_channel, 0, self.current_tick)

    def step_callback(self, msg: Int32):
        delta = int(msg.data) * self.command_step_tick
        self.target_tick = self.clamp_tick(self.target_tick + delta)
        self.get_logger().info(
            f'command={msg.data}, target_tick={self.target_tick}, current_tick={self.current_tick}'
        )

    def timer_callback(self):
        if self.current_tick == self.target_tick:
            return

        if self.current_tick < self.target_tick:
            next_tick = min(self.current_tick + self.slew_tick_per_cycle, self.target_tick)
        else:
            next_tick = max(self.current_tick - self.slew_tick_per_cycle, self.target_tick)

        self.apply_tick(next_tick)

    def destroy_node(self):
        try:
            self.target_tick = self.servo_center_tick
            while self.current_tick != self.target_tick:
                if self.current_tick < self.target_tick:
                    self.current_tick += 1
                else:
                    self.current_tick -= 1
                self.apply_tick(self.current_tick)
            self.pca.close()
        except Exception as e:
            self.get_logger().error(f'cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
