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
        self.declare_parameter('servo_step_tick', 2)
        self.declare_parameter('servo_slew_tick_per_update', 8)
        self.declare_parameter('servo_update_period_sec', 0.02)
        self.declare_parameter('servo_write_deadband_tick', 1)
        self.declare_parameter('recenter_on_estop', True)
        self.declare_parameter('recenter_on_timeout', True)

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
        self.servo_slew_tick_per_update = int(self.get_parameter('servo_slew_tick_per_update').value)
        self.servo_update_period_sec = float(self.get_parameter('servo_update_period_sec').value)
        self.servo_write_deadband_tick = int(self.get_parameter('servo_write_deadband_tick').value)
        self.recenter_on_estop = bool(self.get_parameter('recenter_on_estop').value)
        self.recenter_on_timeout = bool(self.get_parameter('recenter_on_timeout').value)

        self.left_ena = self.get_parameter('left_ena_channel').value
        self.left_in1 = self.get_parameter('left_in1_channel').value
        self.left_in2 = self.get_parameter('left_in2_channel').value

        self.right_in3 = self.get_parameter('right_in3_channel').value
        self.right_in4 = self.get_parameter('right_in4_channel').value
        self.right_enb = self.get_parameter('right_enb_channel').value

        self.motor_max_duty = float(self.get_parameter('motor_max_duty').value)
        self.pca_pwm_freq = float(self.get_parameter('pca_pwm_freq').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.validate_parameters()

        self.pca = PCA9685(
            bus_num=self.i2c_bus,
            address=self.i2c_address,
            pwm_freq=self.pca_pwm_freq
        )

        self.current_servo_tick = self.servo_center_tick
        self.target_servo_tick = self.servo_center_tick
        self.last_written_servo_tick = None
        self.current_throttle = 0.0
        self.estop = False
        self.last_cmd_time = self.get_clock().now()

        self.sub_steering = self.create_subscription(
            Int32, '/vehicle/steering_step', self.steering_callback, 10
        )
        self.sub_steering_abs = self.create_subscription(
            Float32, '/vehicle/steering', self.steering_abs_callback, 10
        )
        self.sub_throttle = self.create_subscription(
            Float32, '/vehicle/throttle', self.throttle_callback, 10
        )
        self.sub_estop = self.create_subscription(
            Bool, '/vehicle/emergency_stop', self.estop_callback, 10
        )

        self.pub_state = self.create_publisher(String, '/vehicle/state', 10)
        self.pub_estop_state = self.create_publisher(Bool, '/vehicle/emergency_stop_state', 10)
        self.timer = self.create_timer(self.servo_update_period_sec, self.timer_callback)

        self.write_steering_tick(self.servo_center_tick, force=True)
        self.stop_all()

        self.get_logger().info(
            f'vehicle_hw_node started | '
            f'servo center={self.servo_center_tick}, '
            f'min={self.servo_min_tick}, max={self.servo_max_tick}, '
            f'step={self.servo_step_tick}, slew={self.servo_slew_tick_per_update}, '
            f'period={self.servo_update_period_sec:.3f}s'
        )
        self.log_configuration()
        self.publish_estop_state()

    def validate_parameters(self):
        if self.i2c_bus < 0:
            raise ValueError(f'i2c_bus must be >= 0, got {self.i2c_bus}')
        if not 0x03 <= int(self.i2c_address) <= 0x77:
            raise ValueError(f'i2c_address must be a valid 7-bit I2C address, got 0x{int(self.i2c_address):02x}')

        channels = {
            'steering_channel': self.steering_channel,
            'left_ena_channel': self.left_ena,
            'left_in1_channel': self.left_in1,
            'left_in2_channel': self.left_in2,
            'right_in3_channel': self.right_in3,
            'right_in4_channel': self.right_in4,
            'right_enb_channel': self.right_enb,
        }
        invalid_channels = [name for name, channel in channels.items() if not 0 <= int(channel) <= 15]
        if invalid_channels:
            raise ValueError(f'PCA9685 channels must be in [0, 15]: {", ".join(invalid_channels)}')

        duplicate_channels = []
        seen = {}
        for name, channel in channels.items():
            channel = int(channel)
            if channel in seen:
                duplicate_channels.append(f'{seen[channel]}={channel}, {name}={channel}')
            else:
                seen[channel] = name
        if duplicate_channels:
            raise ValueError(f'Duplicate PCA9685 channel assignments: {"; ".join(duplicate_channels)}')

        if not self.servo_min_tick < self.servo_center_tick < self.servo_max_tick:
            raise ValueError(
                'servo ticks must satisfy servo_min_tick < servo_center_tick < servo_max_tick'
            )
        if self.servo_step_tick <= 0:
            raise ValueError(f'servo_step_tick must be > 0, got {self.servo_step_tick}')
        if self.servo_slew_tick_per_update <= 0:
            raise ValueError(
                f'servo_slew_tick_per_update must be > 0, got {self.servo_slew_tick_per_update}'
            )
        if self.servo_update_period_sec <= 0.0:
            raise ValueError(
                f'servo_update_period_sec must be > 0, got {self.servo_update_period_sec}'
            )
        if self.servo_write_deadband_tick < 0:
            raise ValueError(
                f'servo_write_deadband_tick must be >= 0, got {self.servo_write_deadband_tick}'
            )
        if not 0.0 < self.motor_max_duty <= 1.0:
            raise ValueError(f'motor_max_duty must be in (0.0, 1.0], got {self.motor_max_duty}')
        if self.pca_pwm_freq <= 0.0:
            raise ValueError(f'pca_pwm_freq must be > 0, got {self.pca_pwm_freq}')
        if self.command_timeout_sec <= 0.0:
            raise ValueError(
                f'command_timeout_sec must be > 0, got {self.command_timeout_sec}'
            )

        if abs(self.pca_pwm_freq - 50.0) > 1e-6:
            self.get_logger().warn(
                'Using a shared servo/motor PCA9685 at a non-50Hz frequency can destabilize steering.'
            )

    def log_configuration(self):
        self.get_logger().info(
            'hardware config | '
            f'i2c_bus={self.i2c_bus}, address=0x{int(self.i2c_address):02x}, '
            f'pwm_freq={self.pca_pwm_freq}, motor_max_duty={self.motor_max_duty:.2f}, '
            f'timeout={self.command_timeout_sec:.2f}'
        )
        self.get_logger().info(
            'channel map | '
            f'steering={self.steering_channel}, '
            f'left=({self.left_ena},{self.left_in1},{self.left_in2}), '
            f'right=({self.right_in3},{self.right_in4},{self.right_enb})'
        )

    def publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.pub_state.publish(msg)

    def publish_estop_state(self):
        msg = Bool()
        msg.data = self.estop
        self.pub_estop_state.publish(msg)

    def set_pwm_raw(self, channel: int, duty: float):
        duty = max(0.0, min(1.0, duty))
        off = int(duty * 4095)
        self.pca.set_pwm(channel, 0, off)

    def clamp_steering_tick(self, tick: int) -> int:
        tick = int(tick)
        return max(self.servo_min_tick, min(self.servo_max_tick, tick))

    def write_steering_tick(self, tick: int, force: bool = False):
        tick = self.clamp_steering_tick(tick)
        if (
            not force
            and self.last_written_servo_tick is not None
            and abs(tick - self.last_written_servo_tick) < self.servo_write_deadband_tick
        ):
            return
        self.current_servo_tick = tick
        self.last_written_servo_tick = tick
        self.pca.set_pwm(self.steering_channel, 0, self.current_servo_tick)
        self.get_logger().debug(f'steering_tick={self.current_servo_tick}')

    def set_steering_tick(self, tick: int):
        target = self.clamp_steering_tick(tick)
        if target != self.target_servo_tick:
            self.target_servo_tick = target
            self.get_logger().info(f'steering_target_tick={self.target_servo_tick}')

    def set_steering_normalized(self, value: float):
        value = max(-1.0, min(1.0, float(value)))
        if value >= 0.0:
            tick = self.servo_center_tick + value * (self.servo_max_tick - self.servo_center_tick)
        else:
            tick = self.servo_center_tick + value * (self.servo_center_tick - self.servo_min_tick)
        self.set_steering_tick(int(round(tick)))

    def update_steering(self):
        if self.current_servo_tick == self.target_servo_tick:
            return

        delta = self.target_servo_tick - self.current_servo_tick
        step = min(abs(delta), self.servo_slew_tick_per_update)
        if delta < 0:
            step = -step
        self.write_steering_tick(self.current_servo_tick + step)

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

    def steering_abs_callback(self, msg: Float32):
        if self.estop:
            return

        self.last_cmd_time = self.get_clock().now()
        self.set_steering_normalized(float(msg.data))

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
            if self.recenter_on_estop:
                self.set_steering_tick(self.servo_center_tick)
            self.publish_state('EMERGENCY_STOP=ON')
        else:
            self.get_logger().info('EMERGENCY STOP OFF')
            self.publish_state('EMERGENCY_STOP=OFF')
        self.publish_estop_state()

    def timer_callback(self):
        self.update_steering()

        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.command_timeout_sec:
            timed_out = False
            if abs(self.current_throttle) > 1e-4:
                self.stop_all()
                timed_out = True
            if self.recenter_on_timeout and self.target_servo_tick != self.servo_center_tick:
                self.set_steering_tick(self.servo_center_tick)
                timed_out = True
            if timed_out:
                self.publish_state('timeout_stop')

    def destroy_node(self):
        try:
            self.stop_all()
            self.target_servo_tick = self.servo_center_tick
            self.write_steering_tick(self.servo_center_tick, force=True)
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
