#!/usr/bin/env python3
'''Quadruped stand pose commander for
ros2_control joint_trajectory_controller.

This is adapted from the user’s two_link_controller.py
(FollowJointTrajectory action client) fileciteturn0file0 and extended
to command 12 joints (3 per leg) to a predefined “stand” pose.

Usage: ros2 run quadruped_stand_commander.py [namespace] [duration_sec]

Examples: # If your joint names are exactly like: dog/lf_hip_abd,
dog/lf_hip_pitch, … python3 quadruped_stand_commander.py dog 2.0

# If you used no prefix in joint names, pass empty string: python3
quadruped_stand_commander.py “” 2.0 '''

import sys 
from typing import List

import rclpy 
from rclpy.action import ActionClient 
from rclpy.duration
import Duration 
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory 
from
trajectory_msgs.msg import JointTrajectoryPoint

def _jn(ns: str, name: str) -> str: 
    """Build joint name with optional namespace prefix."""
    ns = (ns or ““).strip() 
    if ns ==”“: 
    return name 
    # Support both ‘dog’ and ‘/dog’ inputs 
    if ns.startswith(”/“): 
        ns = ns[1:]
    return f”{ns}/{name}”

class QuadrupedStandClient(Node): 
    def init(self, namespace: str =“dog”): 
        super().__init__(“quadruped_action_client”)
    self._namespace = namespace self._action_client = ActionClient(self.FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")

        # Joint order must match the controller's joint list for clarity.
        self.joint_names: List[str] = [
            _jn(self._namespace, "lf_hip_abd"),
            _jn(self._namespace, "lf_hip_pitch"),
            _jn(self._namespace, "lf_knee"),
            _jn(self._namespace, "rf_hip_abd"),
            _jn(self._namespace, "rf_hip_pitch"),
            _jn(self._namespace, "rf_knee"),
            _jn(self._namespace, "lh_hip_abd"),
            _jn(self._namespace, "lh_hip_pitch"),
            _jn(self._namespace, "lh_knee"),
            _jn(self._namespace, "rh_hip_abd"),
            _jn(self._namespace, "rh_hip_pitch"),
            _jn(self._namespace, "rh_knee"),
        ]

    def send_stand_goal(self, duration_sec: float = 2.0):
        """
        Sends a single-point trajectory to move to a stand pose.

        NOTE on signs:
          Different URDFs use different joint axis conventions. If the robot bends the wrong way,
          swap the signs for hip_pitch/knee in STAND_POSE below.
        """
        # A reasonable generic stand pose (radians) for many 3-DoF legs:
        # - hip_abd: 0.0 (legs straight under body)
        # - hip_pitch: -0.7 (hip flexion)
        # - knee: +1.4 (knee flexion)
        # If your knee bends the wrong direction, try knee = -1.4.
        STAND_POSE = [
            0.0, -0.7,  1.4,   # LF
            0.0, -0.7,  1.4,   # RF
            0.0, -0.7,  1.4,   # LH
            0.0, -0.7,  1.4,   # RH
        ]

        if len(STAND_POSE) != len(self.joint_names):
            raise RuntimeError("Stand pose length does not match joint list length.")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.time_from_start = Duration(seconds=float(duration_sec)).to_msg()
        pt.positions = STAND_POSE

        goal_msg.trajectory.points = [pt]
        goal_msg.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        self.get_logger().info("Waiting for /joint_trajectory_controller/follow_joint_trajectory ...")
        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending stand goal over {duration_sec:.2f}s to {len(self.joint_names)} joints")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # You can uncomment to see continuous feedback.
        # fb = feedback_msg.feedback
        # self.get_logger().info(str(fb))
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        raise SystemExit

def main(args=None): rclpy.init(args=args)

    # Parse CLI args
    # argv[1] = namespace/prefix, argv[2] = duration_sec (optional)
    namespace = "dog"
    duration_sec = 2.0

    if len(sys.argv) >= 2:
        namespace = str(sys.argv[1])
    if len(sys.argv) >= 3:
        duration_sec = float(sys.argv[2])

    node = QuadrupedStandClient(namespace=namespace)
    node.send_stand_goal(duration_sec=duration_sec)

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("quadruped_action_client").info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if name == “main”: main()
