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


class TwoLinkControllerClient(Node):
    def __init__(self, namespace=''):
        super().__init__('quadruped_action_client')
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

        joint_name = ["lf_joint_1",
                        "rf_joint_1",
                        "lh_joint_1", 
                        "rh_joint_1",
                        "lf_joint_2",
                        "rf_joint_2",
                        "lh_joint_2", 
                        "rh_joint_2",
                        "lf_joint_3",
                        "rf_joint_3",
                        "lh_joint_3", 
                        "rh_joint_3"]
        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        # point.positions = [-angle/4, angle/4, -angle/4, angle/4, angle, angle, -angle, -angle, -2*angle, -2*angle, 2*angle, 2*angle]
        point.positions = [angle, angle, angle, angle, -angle, -angle, -angle, -angle, -angle, -angle, -angle, -angle]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_name
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
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
        feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback:'+str(feedback))

    def state_callback(self, state_msg):
        self._cur_position = state_msg.position
        # print(state_msg.position)

    

def main(args=None):
    
    rclpy.init()

    namespace = "robot"
    action_client = TwoLinkControllerClient(namespace)
    angle = float(sys.argv[1])
    future = action_client.send_goal(angle)
    try:
        rclpy.spin(action_client)
    except SystemExit:
        rclpy.logging.get_logger('two_link_controller_client').info('Shutting down the two_link_controller_client')
    action_client.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()