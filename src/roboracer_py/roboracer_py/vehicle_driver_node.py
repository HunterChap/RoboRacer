import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32, String


class VehicleDriverNode(Node):
    """
    Converts /drive_target into conservative PWM/RPM debug outputs.

    Brake input:
      /brake/request  Float32, clamped to [0.0, 1.0]

    This version still does not write real VESC/ESC hardware. A non-zero
    brake request forces throttle PWM to neutral and publishes an estimated
    brake-current debug value for later VESC integration.
    """

    def __init__(self):
        super().__init__('vehicle_driver_node')

        # Vehicle parameters.
        self.declare_parameter('wheel_diameter_m', 0.102)
        self.declare_parameter('total_gear_ratio', 8.0)

        # Speed and steering limits.
        self.declare_parameter('max_speed_mps', 1.0)
        self.declare_parameter('max_reverse_speed_mps', 0.5)
        self.declare_parameter('max_steering_angle_rad', 0.50)

        # ESC throttle PWM limits in microseconds.
        self.declare_parameter('throttle_neutral_pwm', 1500)
        self.declare_parameter('throttle_forward_max_pwm', 1600)
        self.declare_parameter('throttle_reverse_max_pwm', 1400)

        # Steering-servo PWM limits in microseconds.
        self.declare_parameter('steering_center_pwm', 1500)
        self.declare_parameter('steering_left_max_pwm', 2000)
        self.declare_parameter('steering_right_max_pwm', 1000)

        # Brake diagnostics and future VESC integration.
        self.declare_parameter('brake_command_timeout_sec', 0.30)
        self.declare_parameter('brake_request_threshold', 0.001)
        # Diagnostic conversion only; this is not a calibrated hardware limit.
        self.declare_parameter('debug_max_brake_current_a', 5.0)

        # Safety and diagnostic settings.
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('hardware_output_enable', False)

        self.wheel_diameter_m = float(self.get_parameter('wheel_diameter_m').value)
        self.total_gear_ratio = float(self.get_parameter('total_gear_ratio').value)

        self.max_speed_mps = float(self.get_parameter('max_speed_mps').value)
        self.max_reverse_speed_mps = float(
            self.get_parameter('max_reverse_speed_mps').value
        )
        self.max_steering_angle_rad = float(
            self.get_parameter('max_steering_angle_rad').value
        )

        self.throttle_neutral_pwm = int(
            self.get_parameter('throttle_neutral_pwm').value
        )
        self.throttle_forward_max_pwm = int(
            self.get_parameter('throttle_forward_max_pwm').value
        )
        self.throttle_reverse_max_pwm = int(
            self.get_parameter('throttle_reverse_max_pwm').value
        )

        self.steering_center_pwm = int(
            self.get_parameter('steering_center_pwm').value
        )
        self.steering_left_max_pwm = int(
            self.get_parameter('steering_left_max_pwm').value
        )
        self.steering_right_max_pwm = int(
            self.get_parameter('steering_right_max_pwm').value
        )

        self.brake_command_timeout_sec = float(
            self.get_parameter('brake_command_timeout_sec').value
        )
        self.brake_request_threshold = float(
            self.get_parameter('brake_request_threshold').value
        )
        self.debug_max_brake_current_a = max(
            0.0, float(self.get_parameter('debug_max_brake_current_a').value)
        )

        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.hardware_output_enable = bool(
            self.get_parameter('hardware_output_enable').value
        )

        self.last_drive_msg = None
        self.last_drive_time = None
        self.last_brake_request = 0.0
        self.last_brake_time = None

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive_target',
            self.drive_target_callback,
            10
        )

        self.brake_sub = self.create_subscription(
            Float32,
            '/brake/request',
            self.brake_request_callback,
            10
        )

        self.throttle_pwm_pub = self.create_publisher(
            Int32, '/vehicle_debug/throttle_pwm', 10
        )
        self.steering_pwm_pub = self.create_publisher(
            Int32, '/vehicle_debug/steering_pwm', 10
        )
        self.wheel_rpm_pub = self.create_publisher(
            Float32, '/vehicle_debug/wheel_rpm', 10
        )
        self.motor_rpm_pub = self.create_publisher(
            Float32, '/vehicle_debug/motor_rpm', 10
        )
        self.brake_request_pub = self.create_publisher(
            Float32, '/vehicle_debug/brake_request', 10
        )
        self.brake_current_pub = self.create_publisher(
            Float32, '/vehicle_debug/brake_current_a', 10
        )
        self.brake_state_pub = self.create_publisher(
            String, '/vehicle_debug/brake_state', 10
        )

        self.timer = self.create_timer(self.publish_period, self.control_loop)

        self.get_logger().info('vehicle_driver_node started.')
        self.get_logger().info('Subscribing: /drive_target, /brake/request')
        self.get_logger().info(
            'Brake priority rule: brake request > throttle command.'
        )

        if self.hardware_output_enable:
            self.get_logger().warn(
                'hardware_output_enable=True, but write_hardware_outputs() '
                'is still a TODO. No real hardware output is written.'
            )

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def drive_target_callback(self, msg):
        self.last_drive_msg = msg
        self.last_drive_time = self.get_clock().now()

    def brake_request_callback(self, msg):
        self.last_brake_request = self.clamp(float(msg.data), 0.0, 1.0)
        self.last_brake_time = self.get_clock().now()

    def time_is_fresh(self, timestamp, timeout):
        if timestamp is None:
            return False
        age = (self.get_clock().now() - timestamp).nanoseconds / 1e9
        return age <= timeout

    def command_is_fresh(self):
        return self.time_is_fresh(
            self.last_drive_time, self.command_timeout_sec
        )

    def brake_command_is_fresh(self):
        return self.time_is_fresh(
            self.last_brake_time, self.brake_command_timeout_sec
        )

    def speed_to_throttle_pwm(self, speed_mps):
        speed_mps = self.clamp(
            speed_mps,
            -self.max_reverse_speed_mps,
            self.max_speed_mps
        )

        if abs(speed_mps) < 1e-6:
            return self.throttle_neutral_pwm

        if speed_mps > 0.0:
            ratio = speed_mps / self.max_speed_mps
            pwm = self.throttle_neutral_pwm + ratio * (
                self.throttle_forward_max_pwm - self.throttle_neutral_pwm
            )
        else:
            ratio = abs(speed_mps) / self.max_reverse_speed_mps
            pwm = self.throttle_neutral_pwm + ratio * (
                self.throttle_reverse_max_pwm - self.throttle_neutral_pwm
            )

        return int(round(pwm))

    def steering_angle_to_pwm(self, steering_angle_rad):
        steering_angle_rad = self.clamp(
            steering_angle_rad,
            -self.max_steering_angle_rad,
            self.max_steering_angle_rad
        )

        if abs(steering_angle_rad) < 1e-6:
            return self.steering_center_pwm

        if steering_angle_rad > 0.0:
            ratio = steering_angle_rad / self.max_steering_angle_rad
            pwm = self.steering_center_pwm + ratio * (
                self.steering_left_max_pwm - self.steering_center_pwm
            )
        else:
            ratio = abs(steering_angle_rad) / self.max_steering_angle_rad
            pwm = self.steering_center_pwm + ratio * (
                self.steering_right_max_pwm - self.steering_center_pwm
            )

        return int(round(pwm))

    def speed_to_wheel_rpm(self, speed_mps):
        if self.wheel_diameter_m <= 0.0:
            return 0.0
        wheel_circumference_m = math.pi * self.wheel_diameter_m
        return speed_mps / wheel_circumference_m * 60.0

    def wheel_rpm_to_motor_rpm(self, wheel_rpm):
        return wheel_rpm * self.total_gear_ratio

    @staticmethod
    def publish_int32(publisher, value):
        msg = Int32()
        msg.data = int(value)
        publisher.publish(msg)

    @staticmethod
    def publish_float32(publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    @staticmethod
    def publish_string(publisher, value):
        msg = String()
        msg.data = str(value)
        publisher.publish(msg)

    def make_stop_values(self):
        return (
            0.0,
            0.0,
            self.throttle_neutral_pwm,
            self.steering_center_pwm,
            0.0,
            0.0,
        )

    def write_hardware_outputs(
        self,
        throttle_pwm,
        steering_pwm,
        brake_current_a
    ):
        # Hardware integration placeholder. Implement after VESC and servo communication are confirmed:
        #
        # Priority:
        #   if brake_current_a > 0:
        #       send neutral/zero throttle
        #       send VESC brake-current command
        #   else:
        #       send normal motor/throttle command
        #
        # Steering output is independent.
        #
        # This method currently performs no physical hardware writes.
        return

    def control_loop(self):
        if self.last_drive_msg is None or not self.command_is_fresh():
            (
                speed_mps,
                steering_angle_rad,
                throttle_pwm,
                steering_pwm,
                wheel_rpm,
                motor_rpm,
            ) = self.make_stop_values()

            if self.last_drive_msg is not None:
                self.get_logger().warn(
                    'Drive target timeout. Using neutral throttle.',
                    throttle_duration_sec=1.0
                )
        else:
            speed_mps = self.clamp(
                self.last_drive_msg.drive.speed,
                -self.max_reverse_speed_mps,
                self.max_speed_mps
            )
            steering_angle_rad = self.clamp(
                self.last_drive_msg.drive.steering_angle,
                -self.max_steering_angle_rad,
                self.max_steering_angle_rad
            )
            throttle_pwm = self.speed_to_throttle_pwm(speed_mps)
            steering_pwm = self.steering_angle_to_pwm(steering_angle_rad)
            wheel_rpm = self.speed_to_wheel_rpm(speed_mps)
            motor_rpm = self.wheel_rpm_to_motor_rpm(wheel_rpm)

        brake_request = (
            self.last_brake_request
            if self.brake_command_is_fresh()
            else 0.0
        )
        brake_current_a = brake_request * self.debug_max_brake_current_a

        if brake_request > self.brake_request_threshold:
            # Do not request drive torque and braking simultaneously.
            speed_mps = 0.0
            throttle_pwm = self.throttle_neutral_pwm
            wheel_rpm = 0.0
            motor_rpm = 0.0
            brake_state = 'active'
        else:
            brake_request = 0.0
            brake_current_a = 0.0
            brake_state = 'idle'

        if self.publish_debug:
            self.publish_int32(self.throttle_pwm_pub, throttle_pwm)
            self.publish_int32(self.steering_pwm_pub, steering_pwm)
            self.publish_float32(self.wheel_rpm_pub, wheel_rpm)
            self.publish_float32(self.motor_rpm_pub, motor_rpm)
            self.publish_float32(self.brake_request_pub, brake_request)
            self.publish_float32(self.brake_current_pub, brake_current_a)
            self.publish_string(self.brake_state_pub, brake_state)

        if self.hardware_output_enable:
            self.write_hardware_outputs(
                throttle_pwm,
                steering_pwm,
                brake_current_a
            )

        self.get_logger().info(
            f'speed={speed_mps:.2f} m/s, '
            f'steering={steering_angle_rad:.2f} rad, '
            f'throttle_pwm={throttle_pwm}, steering_pwm={steering_pwm}, '
            f'brake_request={brake_request:.2f}, '
            f'brake_current_debug={brake_current_a:.2f} A, '
            f'brake_state={brake_state}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
