#!/usr/bin/python3
# Import necessary modules
import os
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

# Define a class for the JointCommander node
class JointCommander(Node):
    def __init__(self, namespace=''):
        # Initialize the node with the name 'four_joint_commander'
        super().__init__('four_joint_commander')
        
        # Define the QoS profile for the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create a publisher for the position controller commands
        self._joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', qos_profile)

    def pub_goal(self, ang):
        """
        Publish the goal angles to the position controller.
        """
        goal_msg = Float64MultiArray()
        try:
            goal_msg.data = [ang[0], ang[1], ang[2], ang[3]]
            # Publish the goal message
            self._joint_pub.publish(goal_msg)
            self.get_logger().info(f"Sent goal: {ang}")
        except IndexError:
            self.get_logger().info(f"Need four angle inputs")



def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the controller.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init()

    try:
        # Get the goal angle from the command line arguments
        angle = [float(sys.argv[1]),float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]

        # Create an instance of the JointCommander
        joint_commander = JointCommander()
        joint_commander.pub_goal(angle)

        try:
            # Spin the node to keep it active
            rclpy.spin(joint_commander)
        except SystemExit:
            # Log a message when shutting down
            rclpy.logging.get_logger('four_joint_commander').info('Shutting down the four_joint_commander')
        
        # Destroy the node explicitly
        joint_commander.destroy_node()

    except IndexError:
        rclpy.logging.get_logger('Invalid number of inputs. Use python3 four_joint_commander.py <ang1> <ang2> <ang3> <ang4>')

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()