#!/usr/bin/python3
import sys
import math
import argparse
from typing import Dict, List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

body_x, body_y, body_z = 0.40, 0.16, 0.10
hip_len = 0.06
thigh_len = 0.20
shank_len = 0.20

hip_rad = 0.018
hip_clear_y = hip_rad + 0.002
hip_drop_z  = hip_rad + 0.010

knee_clear_y = 0.05
knee_fwd_x   = 0.02

JOINT_LIMITS = {
    "abd":  (-0.8, 0.8),
    "hip":  (-1.5, 1.5),
    "knee": (-2.7, 0.1),
}

LEG_SIGNS = {
    "lf": (+1, +1),
    "rf": (+1, -1),
    "lh": (-1, +1),
    "rh": (-1, -1),
}

JOINT_ORDER = [
    "lf_hip_abd", "rf_hip_abd", "lh_hip_abd", "rh_hip_abd",
    "lf_hip_pitch","rf_hip_pitch","lh_hip_pitch","rh_hip_pitch",
    "lf_knee_pitch","rf_knee_pitch","lh_knee_pitch","rh_knee_pitch",
]


def Rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c,  -s],
        [0.0, s,   c],
    ], dtype=float)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def in_limits(q: float, lim: Tuple[float, float]) -> bool:
    return (q >= lim[0] - 1e-9) and (q <= lim[1] + 1e-9)


def leg_mount_in_trunk(xsign: int, ysign: int) -> np.ndarray:
    x_mount = xsign * (body_x / 2.0)
    y_mount = ysign * (body_y / 2.0 + hip_clear_y)
    z_mount = -(body_z / 2.0 + hip_drop_z)
    return np.array([x_mount, y_mount, z_mount], dtype=float)


def ik_leg_3dof(p_trunk: np.ndarray, xsign: int, ysign: int, return_all: bool=False):
    p = np.asarray(p_trunk, dtype=float).reshape(3)
    r0 = leg_mount_in_trunk(xsign, ysign)
    v = p - r0

    xoff = xsign * knee_fwd_x
    yoff = ysign * knee_clear_y
    L1, L2 = thigh_len, shank_len

    vy, vz = float(v[1]), float(v[2])
    r = math.hypot(vy, vz)
    if r < abs(yoff):
        return [] if return_all else None

    phi = math.atan2(vz, vy)
    delta = math.acos(clamp(yoff / r, -1.0, 1.0))
    abd_candidates = [phi - delta, phi + delta]

    sols = []
    for q_abd in abd_candidates:
        if not in_limits(q_abd, JOINT_LIMITS["abd"]):
            continue

        s = Rx(-q_abd).dot(v)
        p2 = s - np.array([hip_len / 2.0, 0.0, 0.0], dtype=float)
        tx, tz = float(p2[0]), float(p2[2])
        rho2 = tx*tx + tz*tz

        R = math.hypot(L1, xoff)
        D = (rho2 - (xoff*xoff + L1*L1 + L2*L2)) / (2.0 * L2)
        if abs(D) > R:
            continue

        alpha = math.atan2(xoff, L1)
        gamma = math.acos(clamp(D / R, -1.0, 1.0))
        knee_candidates = [gamma - alpha, -gamma - alpha]

        for q_knee in knee_candidates:
            if not in_limits(q_knee, JOINT_LIMITS["knee"]):
                continue

            dx = xoff - L2 * math.sin(q_knee)
            dz = -L1 - L2 * math.cos(q_knee)
            norm2 = dx*dx + dz*dz
            if norm2 < 1e-12:
                continue

            c = (dx*tx + dz*tz) / norm2
            s_ = (dz*tx - dx*tz) / norm2
            q_hip = math.atan2(s_, c)

            if not in_limits(q_hip, JOINT_LIMITS["hip"]):
                continue

            sols.append((q_abd, q_hip, q_knee))

    return sols if return_all else (sols[0] if sols else None)


def default_stance() -> Dict[str, np.ndarray]:
    return {
        "lf": np.array([+0.28, +0.12, -0.30], dtype=float),
        "rf": np.array([+0.28, -0.12, -0.30], dtype=float),
        "lh": np.array([-0.28, +0.12, -0.30], dtype=float),
        "rh": np.array([-0.28, -0.12, -0.30], dtype=float),
    }


class TwoLinkControllerClient(Node):
    def __init__(self, namespace: str = ""):
        super().__init__("quadruped_ik_action_client")

        ns = namespace.strip("/")
        self._ns_prefix = f"/{ns}" if ns else ""
        action_topic = f"{self._ns_prefix}/joint_trajectory_controller/follow_joint_trajectory"
        joint_state_topic = f"{self._ns_prefix}/joint_states"

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self._state_sub = self.create_subscription(JointState, joint_state_topic, self.state_callback, qos_profile)

        self._joint_state_pos: Dict[str, float] = {}
        self._received_joint_state = False

    def state_callback(self, msg: JointState):
        if msg.name and msg.position and len(msg.name) == len(msg.position):
            self._joint_state_pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
            self._received_joint_state = True

    def _current_leg_q(self, leg: str) -> np.ndarray:
        return np.array([
            self._joint_state_pos.get(f"{leg}_hip_abd", 0.0),
            self._joint_state_pos.get(f"{leg}_hip_pitch", 0.0),
            self._joint_state_pos.get(f"{leg}_knee_pitch", 0.0),
        ], dtype=float)

    def _pick_best_candidate(self, leg: str, candidates: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        q_cur = self._current_leg_q(leg)
        best = candidates[0]
        best_cost = float("inf")
        for q in candidates:
            dq = np.array(q, dtype=float) - q_cur
            cost = float(dq @ dq)
            if cost < best_cost:
                best_cost = cost
                best = q
        return best

    def foot_targets_to_joint_positions(self, feet_trunk: Dict[str, np.ndarray]) -> Optional[List[float]]:
        leg_q: Dict[str, Tuple[float, float, float]] = {}

        for leg, p in feet_trunk.items():
            xsign, ysign = LEG_SIGNS[leg]
            cands = ik_leg_3dof(p, xsign, ysign, return_all=True)
            if not cands:
                self.get_logger().error(f"IK failed for leg '{leg}' at p={p.tolist()}")
                return None
            q = self._pick_best_candidate(leg, cands) if self._received_joint_state else cands[0]
            leg_q[leg] = q

        out = []
        for name in JOINT_ORDER:
            leg = name.split("_")[0]
            if name.endswith("hip_abd"):
                out.append(leg_q[leg][0])
            elif name.endswith("hip_pitch"):
                out.append(leg_q[leg][1])
            elif name.endswith("knee_pitch"):
                out.append(leg_q[leg][2])
            else:
                raise RuntimeError(f"Unexpected joint name: {name}")
        return out

    def send_goal_from_feet(self, feet_trunk: Dict[str, np.ndarray], duration_s: float = 1.0):
        joint_positions = self.foot_targets_to_joint_positions(feet_trunk)
        if joint_positions is None:
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=2).to_msg()
        goal_msg.trajectory.joint_names = JOINT_ORDER

        point = JointTrajectoryPoint()
        secs = int(duration_s)
        nsecs = int((duration_s - secs) * 1e9)
        point.time_from_start = Duration(seconds=secs, nanoseconds=nsecs).to_msg()
        point.positions = joint_positions
        goal_msg.trajectory.points = [point]

        self.get_logger().info(
            f"Sending IK trajectory to {self._ns_prefix or '/'}joint_trajectory_controller "
            f"feet={ {k: v.tolist() for k,v in feet_trunk.items()} }"
        )

        self._action_client.wait_for_server()
        fut = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        fut.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        raise SystemExit


def parse_args(argv: List[str]):
    p = argparse.ArgumentParser(
        description="Moves the quadruped w.r.t truck frame or each leg independently"
    )
    p.add_argument("--namespace", default="", help="Robot namespace.")
    p.add_argument("--duration", type=float, default=1.0, help="Movement duration seconds.")

    p.add_argument("--dx", type=float, default=0.0, help="Shift all feet in x (m) from default stance.")
    p.add_argument("--dy", type=float, default=0.0, help="Shift all feet in y (m) from default stance.")
    p.add_argument("--dz", type=float, default=0.0, help="Shift all feet in z (m) from default stance.")

    p.add_argument("--lf", nargs=3, type=float, metavar=("X","Y","Z"), help="Override LF foot target.")
    p.add_argument("--rf", nargs=3, type=float, metavar=("X","Y","Z"), help="Override RF foot target.")
    p.add_argument("--lh", nargs=3, type=float, metavar=("X","Y","Z"), help="Override LH foot target.")
    p.add_argument("--rh", nargs=3, type=float, metavar=("X","Y","Z"), help="Override RH foot target.")
    return p.parse_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)

    feet = default_stance()
    shift = np.array([args.dx, args.dy, args.dz], dtype=float)
    for k in feet:
        feet[k] = feet[k] + shift

    if args.lf: feet["lf"] = np.array(args.lf, dtype=float)
    if args.rf: feet["rf"] = np.array(args.rf, dtype=float)
    if args.lh: feet["lh"] = np.array(args.lh, dtype=float)
    if args.rh: feet["rh"] = np.array(args.rh, dtype=float)

    rclpy.init()
    node = TwoLinkControllerClient(args.namespace)

    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
        if node._received_joint_state:
            break

    node.send_goal_from_feet(feet, duration_s=args.duration)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
