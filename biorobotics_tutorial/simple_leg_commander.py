#!/usr/bin/python3
import os
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np


class LegJointCommanderClient(Node):
    def __init__(self, namespace=''):
        super().__init__('leg_joint_action_client')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, qos_profile)
        self._namespace = namespace
        self._cur_position = np.array([0.0, 0.0])

    def send_goal(self, angle):
        goal_msg = FollowJointTrajectory.Goal()

        joint_name = ["lf_hip_abd",
                        "lf_hip_pitch",
                        "lf_knee_pitch"]
        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point.positions = [-angle/2, angle, -2*angle]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_name
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goals(self, angles):
        goal_msg = FollowJointTrajectory.Goal()

        joint_name = ["lf_hip_abd",
                        "lf_hip_pitch",
                        "lf_knee_pitch"]
        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point.positions = [angles[0], angles[1], angles[2]]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_name
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def inverse_kinematics(self, position):
        x = position[0]
        y = position[1]
        z = position[2]
        joint_angles = [0.5, -0.5, 0.5]
        """
        Write your inverse kinematics code here.
        """
        return joint_angles
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        raise SystemExit

    def feedback_callback(self, feedback_msg):
        angle_fb = feedback_msg.feedback.actual.positions
        angle_des = feedback_msg.feedback.desired.positions
        self.get_logger().info(f'Desired angles: {angle_des[0]:.2f}, {angle_des[1]:.2f}, {angle_des[2]:.2f}')
        self.get_logger().info(f'Current angles: {angle_fb[0]:.2f}, {angle_fb[1]:.2f}, {angle_fb[2]:.2f}')


    def state_callback(self, state_msg):
        self._cur_position = state_msg.position

    

def main(args=None):
    
    rclpy.init()

    namespace = "robot"
    action_client = LegJointCommanderClient(namespace)
    if len(sys.argv) == 2:
        angle = float(sys.argv[1])
        future = action_client.send_goal(angle)
    elif len(sys.argv) == 4:
        angles = action_client.inverse_kinematics([sys.argv[1], sys.argv[2], sys.argv[3]])
        future = action_client.send_goals(angles)
    try:
        rclpy.spin(action_client)
    except SystemExit:
        rclpy.logging.get_logger('leg_joint_controller_client').info('Shutting down the leg_joint_controller_client')
    action_client.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()