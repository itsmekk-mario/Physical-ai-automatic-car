import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

from jetcar_base.pca9685_driver import PCA9685


class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)

        self.declare_parameter('left_ena_channel', 1)
        self.declare_parameter('left_in1_channel', 2)
        self.declare_parameter('left_in2_channel', 3)

        self.declare_parameter('right_in3_channel', 4)
        self.declare_parameter('right_in4_channel', 5)
        self.declare_parameter('right_enb_channel', 6)

        self.declare_parameter('motor_max_duty', 0.8)
        self.declare_parameter('pca_pwm_freq', 1000.0)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value

        self.left_ena = self.get_parameter('left_ena_channel').value
        self.left_in1 = self.get_parameter('left_in1_channel').value
        self.left_in2 = self.get_parameter('left_in2_channel').value

        self.right_enb = self.get_parameter('right_enb_channel').value
        self.right_in3 = self.get_parameter('right_in3_channel').value
        self.right_in4 = self.get_parameter('right_in4_channel').value

        self.motor_max_duty = float(self.get_parameter('motor_max_duty').value)
        self.pca_pwm_freq = float(self.get_parameter('pca_pwm_freq').value)

        self.pca = PCA9685(
            bus_num=self.i2c_bus,
            address=self.i2c_address,
            pwm_freq=self.pca_pwm_freq
        )

        self.estop = False

        self.sub_throttle = self.create_subscription(
            Float32,
            '/vehicle/throttle',
            self.throttle_callback,
            10
        )

        self.sub_estop = self.create_subscription(
            Bool,
            '/vehicle/emergency_stop',
            self.estop_callback,
            10
        )

        self.stop_all()
        self.get_logger().info(
            f'motor_test_node started | pwm_freq={self.pca_pwm_freq}Hz '
            '(do not run with other PCA9685 hardware nodes)'
        )

    def set_duty_cycle(self, channel, duty):
        duty = max(0.0, min(1.0, duty))
        off = int(duty * 4095)
        self.pca.set_pwm(channel, 0, off)

    def set_motor(self, en_channel, in1_channel, in2_channel, throttle):
        throttle = max(-1.0, min(1.0, throttle))

        if abs(throttle) < 1e-4:
            self.set_duty_cycle(en_channel, 0.0)
            self.set_duty_cycle(in1_channel, 0.0)
            self.set_duty_cycle(in2_channel, 0.0)
            return

        duty = min(abs(throttle) * self.motor_max_duty, self.motor_max_duty)

        if throttle > 0.0:
            self.set_duty_cycle(en_channel, duty)
            self.set_duty_cycle(in1_channel, 1.0)
            self.set_duty_cycle(in2_channel, 0.0)
        else:
            self.set_duty_cycle(en_channel, duty)
            self.set_duty_cycle(in1_channel, 0.0)
            self.set_duty_cycle(in2_channel, 1.0)

    def apply_throttle(self, throttle):
        self.set_motor(self.left_ena, self.left_in1, self.left_in2, throttle)
        self.set_motor(self.right_enb, self.right_in3, self.right_in4, throttle)
        self.get_logger().info(f'throttle = {throttle:.2f}')

    def stop_all(self):
        self.set_motor(self.left_ena, self.left_in1, self.left_in2, 0.0)
        self.set_motor(self.right_enb, self.right_in3, self.right_in4, 0.0)
        self.get_logger().info('motors stopped')

    def throttle_callback(self, msg):
        if self.estop:
            self.stop_all()
            return
        throttle = max(-1.0, min(1.0, float(msg.data)))
        self.apply_throttle(throttle)

    def estop_callback(self, msg):
        self.estop = bool(msg.data)
        if self.estop:
            self.get_logger().warn('EMERGENCY STOP ON')
            self.stop_all()
        else:
            self.get_logger().info('EMERGENCY STOP OFF')

    def destroy_node(self):
        try:
            self.stop_all()
            self.pca.close()
        except Exception as e:
            self.get_logger().error(f'cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorTestNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'motor_test_node failed to start: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
