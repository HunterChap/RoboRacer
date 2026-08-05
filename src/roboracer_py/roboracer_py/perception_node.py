import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

scan_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # QoS profile used by the /scan subscription.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
)

        # Subscribe to LaserScan data from the active simulator or LiDAR source.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
)

        # Publish the minimum distance in the front sector.
        self.front_pub = self.create_publisher(Float32, '/front_distance', 10)

        # Publish the minimum distance in the left sector.
        self.left_pub = self.create_publisher(Float32, '/left_distance', 10)

        # Publish the minimum distance in the right sector.
        self.right_pub = self.create_publisher(Float32, '/right_distance', 10)

        self.get_logger().info('Perception node started.')

    def get_min_distance_in_angle_range(self, msg, angle_min_deg, angle_max_deg):
        """
        Return the minimum valid LaserScan range inside an angular sector.

        Sector limits are specified in degrees. Positive angles are to the
        left of the vehicle centerline and negative angles are to the right.
        """

        angle_min_rad = math.radians(angle_min_deg)
        angle_max_rad = math.radians(angle_max_deg)

        valid_ranges = []

        for i, distance in enumerate(msg.ranges):
            # Compute the angle associated with this range sample.
            angle = msg.angle_min + i * msg.angle_increment

            # Process only samples inside the requested sector.
            if angle_min_rad <= angle <= angle_max_rad:
                # Reject non-finite values and ranges outside the sensor limits.
                if (
                    math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max
                ):
                    valid_ranges.append(distance)

        # Return the sensor maximum when the sector has no valid sample.
        # Downstream control logic interprets this sector as clear.
        if not valid_ranges:
            return msg.range_max

        return min(valid_ranges)

    def publish_float(self, publisher, value):
        msg = Float32()
        msg.data = float(value)
        publisher.publish(msg)

    def scan_callback(self, msg):
        if not msg.ranges:
            return

        # Front sector.
        front_distance = self.get_min_distance_in_angle_range(
            msg,
            -20,
            20
        )

        # Left sector.
        left_distance = self.get_min_distance_in_angle_range(
            msg,
            20,
            90
        )

        # Right sector.
        right_distance = self.get_min_distance_in_angle_range(
            msg,
            -90,
            -20
        )

        self.publish_float(self.front_pub, front_distance)
        self.publish_float(self.left_pub, left_distance)
        self.publish_float(self.right_pub, right_distance)

        self.get_logger().info(
            f'front={front_distance:.2f} m, '
            f'left={left_distance:.2f} m, '
            f'right={right_distance:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
