#!/usr/bin/python3
# Import necessary modules
import os
import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock
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

        # Lengths of links in the four_links xacro file
        self._link_len = np.array([0.225, 0.275])

        # Time period
        self._dt = 0.02  # seconds

        # Create a timer to call a function repeatedly over the time period
        self._timer = self.create_timer(self._dt, self.cmdloop_callback)
        self._start_time = Clock().now().nanoseconds/1E9

    def cmdloop_callback(self):
        """
        Timed callback function to publish the joint angles for the end-effectors to track circles
        """
        # Measure the time from start
        t = Clock().now().nanoseconds/1E9 - self._start_time

        # Create position setpoints for the end-effectors of the arms
        setpoint1 = np.array([-0.1 + 0.1*np.cos(1.0*t), 0.25 + 0.1*np.sin(1.0*t)])
        setpoint2 = np.array([-0.1 + 0.1*np.cos(1.0*t), 0.25 + 0.1*np.sin(1.0*t)])

        # Find the joint angles to realize the position setpoints
        ik1 = self.inverseKinematics(setpoint1)
        ik2 = self.inverseKinematics(setpoint2)


        # Create a goal message
        goal_msg = Float64MultiArray()

        # Add the joint angles to the goal message
        goal_msg.data = ik1.tolist() + ik2.tolist() 

        # Publish the goal message
        self._joint_pub.publish(goal_msg)

    def inverseKinematics(self, pos):
        """
        Inverse Kinematics of two-link manipulator taken from MATLAB example:
        https://se.mathworks.com/help/fuzzy/modeling-inverse-kinematics-in-a-robotic-arm.html
        """
        """
        Please enter your code here.
        """
        return np.array([theta1, theta2])

def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the controller.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Create an instance of the JointCommander
    joint_commander = JointCommander()

    try:
        # Spin the node to keep it active
        rclpy.spin(joint_commander)
    except SystemExit:
        # Log a message when shutting down
        rclpy.logging.get_logger('four_joint_commander').info('Shutting down the four_joint_commander')
    
    # Destroy the node explicitly
    joint_commander.destroy_node()

    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()