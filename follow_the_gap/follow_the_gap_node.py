import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap')  

        # Parameters
        self.safety_radius = 1.55  # Minimum safe distance from wall
        self.max_throttle = 2.1  # Maximum throttle
        self.steering_sensitivity = 0.37  # Adjust sensitivity as needed
        self.max_steering_angle = 0.52  # Steering angle limit in radians
        self.wheelbase = 0.25  # Approximate wheelbase length in meters

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def scan_callback(self, scan_data):
            # Preprocess scan data
            ranges = np.array(scan_data.ranges)
            ranges[np.isinf(ranges)] = scan_data.range_max

            # Define the angle range (20 degrees in front of the LIDAR)
            angle_range = 10 * (np.pi / 180)  # Convert degrees to radians
            center_index = len(ranges) // 2
            angle_increment = scan_data.angle_increment
            range_indices = int(angle_range / angle_increment)
            front_indices = ranges[center_index - range_indices // 2 : center_index + range_indices // 2]

            # Check for obstacles within the safety radius in the specified range
            if np.any(front_indices < self.safety_radius):
                self.publish_stop_command()
                self.get_logger().info("Obstacle detected in front! Stopping the car.")
            else:
                safe_ranges = np.where(ranges > self.safety_radius, ranges, 0)
                best_angle = self.find_best_gap(safe_ranges, scan_data.angle_min, scan_data.angle_increment)
                self.get_logger().info(f"best_angle = {best_angle} \n  safe_ranges={safe_ranges} \n ranges={ranges}")
                self.publish_drive_command(best_angle)
                self.publish_steer_marker(best_angle)
    def find_best_gap(self, ranges, angle_min, angle_increment):
        safe_indices = np.where(ranges > 0)[0]
        if len(safe_indices) == 0:
            return 0.0  # Default to 0 if no safe gap is found

        largest_gap = max(np.split(safe_indices, np.where(np.diff(safe_indices) > 1)[0] + 1), key=len)
        mid_index = (largest_gap[0] + largest_gap[-1]) // 2
        return angle_min + mid_index * angle_increment

    def publish_drive_command(self, best_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.max_throttle
        drive_msg.drive.steering_angle = max(-self.max_steering_angle, min(best_angle * self.steering_sensitivity, self.max_steering_angle))
        self.drive_pub.publish(drive_msg)

    def publish_stop_command(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)

    def publish_steer_marker(self, steering_angle):
        steer_marker = Marker()
        steer_marker.header.frame_id = "base_link"
        steer_marker.header.stamp = self.get_clock().now().to_msg()
        steer_marker.ns = "steering_path"
        steer_marker.id = 0
        steer_marker.type = Marker.LINE_STRIP
        steer_marker.action = Marker.ADD
        steer_marker.pose.orientation.w = 1.0
        steer_marker.scale.x = 0.2  # Line thickness
        steer_marker.color.r = 0.96
        steer_marker.color.g = 0.22
        steer_marker.color.b = 0.06
        steer_marker.color.a = 1.0

        # Generate steering path based on steering angle
        marker_pos_x, marker_pos_y, theta = 0.0, 0.0, 0.0
        num_points = 50  # Number of points in the marker path
        for i in range(num_points):
            marker_pos_x += 0.1 * np.cos(theta)
            marker_pos_y += 0.1 * np.sin(theta)
            theta += 0.1 / self.wheelbase * np.tan(steering_angle)
            point = Point(x=marker_pos_x, y=marker_pos_y, z=0.0)
            steer_marker.points.append(point)

        # Publish the marker
        self.marker_pub.publish(steer_marker)

    def publish_wall_markers(self, scan_data, ranges):
        # Create a marker for wall points visualization
        wall_marker = Marker()
        wall_marker.header.frame_id = "base_link"
        wall_marker.header.stamp = self.get_clock().now().to_msg()
        wall_marker.ns = "wall_points"
        wall_marker.id = 1
        wall_marker.type = Marker.POINTS
        wall_marker.action = Marker.ADD

        # Set size and color for the wall marker points
        wall_marker.scale.x = 0.05  # Point size in x direction
        wall_marker.scale.y = 0.05  # Point size in y direction
        wall_marker.color.r = 0.0  # Red channel (0.0)
        wall_marker.color.g = 0.8  # Green channel (high intensity for visibility)
        wall_marker.color.b = 0.0  # Blue channel (0.0)
        wall_marker.color.a = 1.0  # Alpha channel (opacity)

        # Convert polar coordinates (range, angle) to cartesian coordinates (x, y)
        angle = scan_data.angle_min
        for r in ranges:
            if r > 0:  # Only process valid range points
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                point = Point(x=x, y=y, z=0.0)
                wall_marker.points.append(point)
            angle += scan_data.angle_increment

        # Publish the wall marker
        self.marker_pub.publish(wall_marker)


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
