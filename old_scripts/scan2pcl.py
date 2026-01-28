import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf_transformations import *


class LaserScanToPointCloud(Node):

    def __init__(self):
        super().__init__('laser_scan_to_point_cloud')

        # Initialize Subscribers
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10  # QoS
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS
        )

        # Initialize the point cloud publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/laser_point_cloud', 10)

        # Set up TF listener for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def laser_scan_callback(self, laser_scan: LaserScan):
        # Get latest Odometry (pose) data
        try:
            odom_msg = self.get_odom()
        except Exception as e:
            self.get_logger().warn(f"Failed to get Odometry data: {e}")
            return

        # Convert LaserScan and Odometry to PointCloud2
        point_cloud = self.laser_scan_to_point_cloud2(laser_scan, odom_msg)

        # Publish the PointCloud2 message
        self.pc_pub.publish(point_cloud)

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

    def get_odom(self):
        if hasattr(self, 'odom_msg'):
            return self.odom_msg
        else:
            raise Exception("Odometry message not available")

    def laser_scan_to_point_cloud2(self, laser_scan, odom_msg):
        # Robot's position and orientation from Odometry
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        # Transform quaternion to Euler angles
        roll, pitch, yaw = self.euler_from_quaternion(orientation)

        points = []

        # Process each laser scan range value
        for i in range(len(laser_scan.ranges)):
            angle = laser_scan.angle_min + i * laser_scan.angle_increment
            r = laser_scan.ranges[i]

            # Skip invalid ranges (e.g., NaN or out of range)
            if r < laser_scan.range_min or r > laser_scan.range_max or np.isnan(r):
                continue

            # Convert polar to Cartesian coordinates (in robot frame)
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0  # Assuming laser scan is at the robot's height

            # Transform to global coordinates using odometry
            x_global = position.x + x * np.cos(yaw) - y * np.sin(yaw)
            y_global = position.y + x * np.sin(yaw) + y * np.cos(yaw)
            z_global = z  # Assuming no vertical movement

            points.append([x_global, y_global, z_global])

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Or use the appropriate frame

        pc_data = pc2.create_cloud_xyz32(header, points)
        return pc_data

    def euler_from_quaternion(self, quat):
        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
