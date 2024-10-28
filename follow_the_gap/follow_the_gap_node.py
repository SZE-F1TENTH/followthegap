import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        # Parameters
        self.safety_radius = 2   # Minimum safe distance from obstacles
        self.max_throttle = 0.1   # Maximum throttle value
        self.steering_sensitivity = -0.9  # Adjust sensitivity as needed
        self.max_steering_angle = 0.52   # Steering angle limit in radians

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)

    def scan_callback(self, scan_data):
        ranges = np.array(scan_data.ranges)

        # Replace 'inf' values with max range
        max_range = scan_data.range_max
        ranges[np.isinf(ranges)] = max_range

        # Only keep ranges beyond the safety radius
        safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)

        # Find the largest safe gap and its midpoint angle
        best_direction = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)

        # Publish throttle and steering commands
        self.publish_drive_command(best_direction)

    def find_best_gap(self, ranges, angle_min, angle_increment):
        # Identify indices of safe ranges
        safe_indices = np.where(ranges > 0)[0]

        # Find gaps and select the largest
        gaps = []
        gap_start = safe_indices[0]
        for i in range(1, len(safe_indices)):
            if safe_indices[i] - safe_indices[i - 1] > 1:
                gaps.append((gap_start, safe_indices[i - 1]))
                gap_start = safe_indices[i]
        gaps.append((gap_start, safe_indices[-1]))

        # Select the largest gap by its width
        largest_gap = max(gaps, key=lambda gap: gap[1] - gap[0])

        # Calculate the middle angle of the largest gap
        mid_index = (largest_gap[0] + largest_gap[1]) // 2
        best_angle = angle_min + mid_index * angle_increment

        return best_angle

    def publish_drive_command(self, best_angle):
        # Throttle command (constant for simplicity)
        throttle_msg = Float32()
        throttle_msg.data = self.max_throttle
        self.throttle_pub.publish(throttle_msg)
        
        # Steering command, [-0.52, 0.52] range
        steering_msg = Float32()
        steering_value = -best_angle * self.steering_sensitivity
        steering_msg.data = max(-self.max_steering_angle, min(steering_value, self.max_steering_angle))
        self.steering_pub.publish(steering_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
