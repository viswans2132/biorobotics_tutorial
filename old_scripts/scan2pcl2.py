import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import tf_transformations as tf

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        
        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg: LaserScan):
        if self.current_pose is None:
            return

        # Convert LaserScan to PointCloud2
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y, 0.0))
            angle += msg.angle_increment

        # Transform points to the global frame using odometry
        quaternion = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ]
        euler = tf.euler_from_quaternion(quaternion)
        yaw = euler[2]

        transformed_points = []
        for x, y, z in points:
            x_t = x * np.cos(yaw) - y * np.sin(yaw) + self.current_pose.position.x
            y_t = x * np.sin(yaw) + y * np.cos(yaw) + self.current_pose.position.y
            transformed_points.append((x_t, y_t, z))

        # Create PointCloud2 message
        cloud_msg = pc2.create_cloud_xyz32(msg.header, transformed_points)
        cloud_msg.header.frame_id = "odom"  # Assuming a global "map" frame
        self.pc_pub.publish(cloud_msg)
        
        self.get_logger().info("Published PointCloud2 message")


def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
