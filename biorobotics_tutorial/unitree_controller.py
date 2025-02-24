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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import numpy.linalg as la


# Define a class for the JointCommander node
class JointCommander(Node):
    def __init__(self, namespace=''):
        # Initialize the node with the name 'four_joint_commander'
        super().__init__('four_joint_commander')
        
        # Define the QoS profile for the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create a publisher for the position controller commands
        self._joint_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self._odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        # Lengths of links in the four_links xacro file
        self._link_len = np.array([0.225, 0.275])

        # Time period
        self._dt = 0.1  # seconds

        # Create a timer to call a function repeatedly over the time period
        self._timer = self.create_timer(self._dt, self.cmdloop_callback)
        self._start_time = Clock().now().nanoseconds/1E9

        self._cur_pos = np.array([0.0, 0.0])
        self._yaw = 0.0
        self._des_pos = np.array([0.0, 0.0])

        self._horizon = 50

        self._obs_pos = np.array([2.0, 0.0])

    def cmdloop_callback(self):
        """
        Timed callback function to publish the joint angles for the end-effectors to track circles
        """

        # Create a goal message
        goal_msg = Twist()

        posErr  = self._des_pos - self._cur_pos
        # print(f'Command: {posErr[0]:.2f}: {posErr[1]:.2f}')
        if la.norm(posErr) < 0.1:
            goal_msg.linear.x = 0.0
            goal_msg.linear.y = 0.0

        else:
            if la.norm(posErr) > 0.5:
                yaw_sp = np.arctan2(posErr[1], posErr[0])
                yawErr = yaw_sp-self._yaw
                yawErr = (yawErr + np.pi) % (2 * np.pi) - np.pi

                # goal_msg.angular.z = 0.3*yawErr



            R = np.array([[np.cos(self._yaw), np.sin(self._yaw)], [-np.sin(self._yaw), np.cos(self._yaw)]])
            u_body = R@posErr

            print(f'Body vel: {u_body[0]:.2f}: {u_body[1]:.2f}')



            # goal_msg.linear.x = u_body[0]
            # goal_msg.linear.y = u_body[1]

            
            # # print(posErr)
            # print(f'Command: {posErr[0]:.2f}: {posErr[1]:.2f}')
            goal_msg.linear.x = np.minimum(np.maximum(2*u_body[0], -0.5), 0.5)
            goal_msg.linear.y = np.minimum(np.maximum(2*u_body[1], -0.5), 0.5)


        # Publish the goal message
        # goal_msg.linear.x = -0.0
        # goal_msg.linear.y = -0.0
        self._joint_pub.publish(goal_msg)
        print(f'Command: {goal_msg.linear.x:.2f}: {goal_msg.linear.y:.2f}')

    def inverseKinematics(self, pos):
        """
        Inverse Kinematics of two-link manipulator taken from MATLAB example:
        https://se.mathworks.com/help/fuzzy/modeling-inverse-kinematics-in-a-robotic-arm.html
        """
        """
        Please enter your code here.
        """
        return np.array([theta1, theta2])

    def odom_callback(self, msg):
        self._cur_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._yaw = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))
        # print(self._yaw*180/np.pi)

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