import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, String

from jetcar_base.pca9685_driver import PCA9685


class VehicleHardwareNode(Node):
    def __init__(self):
        super().__init__('vehicle_hw_node')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x40)

        # steering servo (raw tick)
        self.declare_parameter('steering_channel', 0)
        self.declare_parameter('servo_center_tick', 323)
        self.declare_parameter('servo_min_tick', 236)
        self.declare_parameter('servo_max_tick', 410)
        self.declare_parameter('servo_step_tick', 3)

        # left motor
        self.declare_parameter('left_ena_channel', 1)
        self.declare_parameter('left_in1_channel', 2)
        self.declare_parameter('left_in2_channel', 3)

        # right motor
        self.declare_parameter('right_in3_channel', 4)
        self.declare_parameter('right_in4_channel', 5)
        self.declare_parameter('right_enb_channel', 6)

        self.declare_parameter('motor_max_duty', 1.0)
        self.declare_parameter('pca_pwm_freq', 50.0)
        self.declare_parameter('command_timeout_sec', 0.7)

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value

        self.steering_channel = self.get_parameter('steering_channel').value
        self.servo_center_tick = int(self.get_parameter('servo_center_tick').value)
        self.servo_min_tick = int(self.get_parameter('servo_min_tick').value)
        self.servo_max_tick = int(self.get_parameter('servo_max_tick').value)
        self.servo_step_tick = int(self.get_parameter('servo_step_tick').value)

        self.left_ena = self.get_parameter('left_ena_channel').value
        self.left_in1 = self.get_parameter('left_in1_channel').value
        self.left_in2 = self.get_parameter('left_in2_channel').value

        self.right_in3 = self.get_parameter('right_in3_channel').value
        self.right_in4 = self.get_parameter('right_in4_channel').value
        self.right_enb = self.get_parameter('right_enb_channel').value

        self.motor_max_duty = float(self.get_parameter('motor_max_duty').value)
        self.pca_pwm_freq = float(self.get_parameter('pca_pwm_freq').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)

        self.pca = PCA9685(
            bus_num=self.i2c_bus,
            address=self.i2c_address,
            pwm_freq=self.pca_pwm_freq
        )

        self.current_servo_tick = self.servo_center_tick
        self.current_throttle = 0.0
        self.estop = False
        self.last_cmd_time = self.get_clock().now()

        self.sub_steering = self.create_subscription(
            Int32, '/vehicle/steering_step', self.steering_callback, 10
        )
        self.sub_throttle = self.create_subscription(
            Float32, '/vehicle/throttle', self.throttle_callback, 10
        )
        self.sub_estop = self.create_subscription(
            Bool, '/vehicle/emergency_stop', self.estop_callback, 10
        )

        self.pub_state = self.create_publisher(String, '/vehicle/state', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.set_steering_tick(self.servo_center_tick)
        self.stop_all()

        self.get_logger().info(
            f'vehicle_hw_node started | '
            f'servo center={self.servo_center_tick}, '
            f'min={self.servo_min_tick}, max={self.servo_max_tick}, '
            f'step={self.servo_step_tick}'
        )

    def publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.pub_state.publish(msg)

    def set_pwm_raw(self, channel: int, duty: float):
        duty = max(0.0, min(1.0, duty))
        off = int(duty * 4095)
        self.pca.set_pwm(channel, 0, off)

    def set_steering_tick(self, tick: int):
        tick = int(tick)
        tick = max(self.servo_min_tick, min(self.servo_max_tick, tick))
        self.current_servo_tick = tick
        self.pca.set_pwm(self.steering_channel, 0, self.current_servo_tick)
        self.get_logger().info(f'steering_tick={self.current_servo_tick}')

    def set_left_motor(self, throttle: float):
        throttle = max(-1.0, min(1.0, throttle))

        if abs(throttle) < 1e-4:
            self.set_pwm_raw(self.left_ena, 0.0)
            self.set_pwm_raw(self.left_in1, 0.0)
            self.set_pwm_raw(self.left_in2, 0.0)
            return

        duty = min(abs(throttle) * self.motor_max_duty, self.motor_max_duty)

        if throttle > 0.0:
            self.set_pwm_raw(self.left_ena, duty)
            self.set_pwm_raw(self.left_in1, 1.0)
            self.set_pwm_raw(self.left_in2, 0.0)
        else:
            self.set_pwm_raw(self.left_ena, duty)
            self.set_pwm_raw(self.left_in1, 0.0)
            self.set_pwm_raw(self.left_in2, 1.0)

    def set_right_motor(self, throttle: float):
        throttle = max(-1.0, min(1.0, throttle))

        if abs(throttle) < 1e-4:
            self.set_pwm_raw(self.right_enb, 0.0)
            self.set_pwm_raw(self.right_in3, 0.0)
            self.set_pwm_raw(self.right_in4, 0.0)
            return

        duty = min(abs(throttle) * self.motor_max_duty, self.motor_max_duty)

        if throttle > 0.0:
            self.set_pwm_raw(self.right_enb, duty)
            self.set_pwm_raw(self.right_in3, 1.0)
            self.set_pwm_raw(self.right_in4, 0.0)
        else:
            self.set_pwm_raw(self.right_enb, duty)
            self.set_pwm_raw(self.right_in3, 0.0)
            self.set_pwm_raw(self.right_in4, 1.0)

    def apply_throttle(self, throttle: float):
        self.current_throttle = max(-1.0, min(1.0, throttle))
        self.set_left_motor(self.current_throttle)
        self.set_right_motor(self.current_throttle)
        self.get_logger().info(f'throttle={self.current_throttle:.2f}')

    def stop_all(self):
        self.set_left_motor(0.0)
        self.set_right_motor(0.0)
        self.current_throttle = 0.0
        self.get_logger().info('motors stopped')

    def steering_callback(self, msg: Int32):
        if self.estop:
            return

        self.last_cmd_time = self.get_clock().now()
        delta = int(msg.data) * self.servo_step_tick
        self.set_steering_tick(self.current_servo_tick + delta)

    def throttle_callback(self, msg: Float32):
        if self.estop:
            self.stop_all()
            return

        self.last_cmd_time = self.get_clock().now()
        self.apply_throttle(float(msg.data))

    def estop_callback(self, msg: Bool):
        self.estop = bool(msg.data)

        if self.estop:
            self.get_logger().warn('EMERGENCY STOP ON')
            self.stop_all()
            self.publish_state('EMERGENCY_STOP=ON')
        else:
            self.get_logger().info('EMERGENCY STOP OFF')
            self.publish_state('EMERGENCY_STOP=OFF')

    def timer_callback(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.command_timeout_sec and abs(self.current_throttle) > 1e-4:
            self.stop_all()
            self.publish_state('timeout_stop')

    def destroy_node(self):
        try:
            self.stop_all()
            self.set_steering_tick(self.servo_center_tick)
            self.pca.close()
        except Exception as e:
            self.get_logger().error(f'cleanup error: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleHardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'vehicle_hw_node failed to start: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
