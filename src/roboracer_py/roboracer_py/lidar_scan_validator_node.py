import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32


class LidarScanValidatorNode(Node):
    def __init__(self):
        super().__init__('lidar_scan_validator_node')

        # Validator parameters.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('expected_frame_id', '')
        self.declare_parameter('min_valid_ratio', 0.20)
        self.declare_parameter('min_reasonable_range_m', 0.02)
        self.declare_parameter('warn_timeout_sec', 1.0)
        self.declare_parameter('publish_period', 0.5)

        # Angular sectors used for validation diagnostics.
        self.declare_parameter('front_min_deg', -10.0)
        self.declare_parameter('front_max_deg', 10.0)
        self.declare_parameter('left_min_deg', 30.0)
        self.declare_parameter('left_max_deg', 90.0)
        self.declare_parameter('right_min_deg', -90.0)
        self.declare_parameter('right_max_deg', -30.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.expected_frame_id = self.get_parameter('expected_frame_id').value

        self.min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self.min_reasonable_range_m = float(self.get_parameter('min_reasonable_range_m').value)
        self.warn_timeout_sec = float(self.get_parameter('warn_timeout_sec').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        self.front_min_deg = float(self.get_parameter('front_min_deg').value)
        self.front_max_deg = float(self.get_parameter('front_max_deg').value)
        self.left_min_deg = float(self.get_parameter('left_min_deg').value)
        self.left_max_deg = float(self.get_parameter('left_max_deg').value)
        self.right_min_deg = float(self.get_parameter('right_min_deg').value)
        self.right_max_deg = float(self.get_parameter('right_max_deg').value)

        # Best-effort sensor QoS is compatible with common LiDAR drivers.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos
        )

        self.status_pub = self.create_publisher(String, '/lidar_status', 10)
        self.valid_ratio_pub = self.create_publisher(Float32, '/lidar_debug/valid_ratio', 10)
        self.scan_hz_pub = self.create_publisher(Float32, '/lidar_debug/scan_hz', 10)
        self.front_min_pub = self.create_publisher(Float32, '/lidar_debug/front_min', 10)
        self.left_min_pub = self.create_publisher(Float32, '/lidar_debug/left_min', 10)
        self.right_min_pub = self.create_publisher(Float32, '/lidar_debug/right_min', 10)

        self.last_scan_time = None
        self.last_msg = None
        self.last_valid_ratio = 0.0
        self.last_front_min = math.inf
        self.last_left_min = math.inf
        self.last_right_min = math.inf

        self.scan_times = deque(maxlen=20)

        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info('LiDAR scan validator started.')
        self.get_logger().info(f'Subscribing scan topic: {self.scan_topic}')
        self.get_logger().info('Publishing: /lidar_status and /lidar_debug/*')

    def publish_string(self, publisher, value):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def publish_float(self, publisher, value):
        msg = Float32()

        if math.isfinite(value):
            msg.data = float(value)
        else:
            msg.data = -1.0

        publisher.publish(msg)

    def get_scan_hz(self):
        if len(self.scan_times) < 2:
            return 0.0

        duration = (self.scan_times[-1] - self.scan_times[0]).nanoseconds / 1e9

        if duration <= 0.0:
            return 0.0

        return (len(self.scan_times) - 1) / duration

    def get_valid_ranges(self, msg):
        valid_ranges = []

        for distance in msg.ranges:
            if (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
                and distance >= self.min_reasonable_range_m
            ):
                valid_ranges.append(distance)

        return valid_ranges

    def get_min_distance_in_angle_range(self, msg, angle_min_deg, angle_max_deg):
        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)

        values = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if angle_min_rad <= angle <= angle_max_rad:
                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                    and distance >= self.min_reasonable_range_m
                ):
                    values.append(distance)

        if not values:
            return math.inf

        return min(values)

    def scan_callback(self, msg):
        now = self.get_clock().now()

        self.last_scan_time = now
        self.last_msg = msg
        self.scan_times.append(now)

        total_count = len(msg.ranges)
        valid_ranges = self.get_valid_ranges(msg)

        if total_count > 0:
            self.last_valid_ratio = len(valid_ranges) / total_count
        else:
            self.last_valid_ratio = 0.0

        self.last_front_min = self.get_min_distance_in_angle_range(
            msg,
            self.front_min_deg,
            self.front_max_deg
        )

        self.last_left_min = self.get_min_distance_in_angle_range(
            msg,
            self.left_min_deg,
            self.left_max_deg
        )

        self.last_right_min = self.get_min_distance_in_angle_range(
            msg,
            self.right_min_deg,
            self.right_max_deg
        )

    def build_status(self):
        if self.last_scan_time is None or self.last_msg is None:
            return 'WARN: no LaserScan received yet.'

        now = self.get_clock().now()
        age = (now - self.last_scan_time).nanoseconds / 1e9
        scan_hz = self.get_scan_hz()

        warnings = []

        if age > self.warn_timeout_sec:
            warnings.append(f'scan timeout age={age:.2f}s')

        if len(self.last_msg.ranges) == 0:
            warnings.append('empty ranges array')

        if self.last_valid_ratio < self.min_valid_ratio:
            warnings.append(f'low valid ratio={self.last_valid_ratio:.2f}')

        if self.expected_frame_id:
            if self.last_msg.header.frame_id != self.expected_frame_id:
                warnings.append(
                    f'frame_id mismatch actual={self.last_msg.header.frame_id}, '
                    f'expected={self.expected_frame_id}'
                )

        if not math.isfinite(self.last_front_min):
            warnings.append('no valid front scan points')

        if not math.isfinite(self.last_left_min):
            warnings.append('no valid left scan points')

        if not math.isfinite(self.last_right_min):
            warnings.append('no valid right scan points')

        base_info = (
            f'topic={self.scan_topic}, '
            f'frame_id={self.last_msg.header.frame_id}, '
            f'hz={scan_hz:.1f}, '
            f'valid_ratio={self.last_valid_ratio:.2f}, '
            f'range_min={self.last_msg.range_min:.2f}, '
            f'range_max={self.last_msg.range_max:.2f}, '
            f'angle_min={math.degrees(self.last_msg.angle_min):.1f}deg, '
            f'angle_max={math.degrees(self.last_msg.angle_max):.1f}deg, '
            f'front={self.format_distance(self.last_front_min)}, '
            f'left={self.format_distance(self.last_left_min)}, '
            f'right={self.format_distance(self.last_right_min)}'
        )

        if warnings:
            return 'WARN: ' + '; '.join(warnings) + ' | ' + base_info

        return 'OK: ' + base_info

    def format_distance(self, value):
        if math.isfinite(value):
            return f'{value:.2f}m'

        return 'invalid'

    def timer_callback(self):
        status = self.build_status()
        scan_hz = self.get_scan_hz()

        self.publish_string(self.status_pub, status)
        self.publish_float(self.valid_ratio_pub, self.last_valid_ratio)
        self.publish_float(self.scan_hz_pub, scan_hz)
        self.publish_float(self.front_min_pub, self.last_front_min)
        self.publish_float(self.left_min_pub, self.last_left_min)
        self.publish_float(self.right_min_pub, self.last_right_min)

        if status.startswith('OK'):
            self.get_logger().info(status, throttle_duration_sec=1.0)
        else:
            self.get_logger().warn(status, throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanValidatorNode()

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
