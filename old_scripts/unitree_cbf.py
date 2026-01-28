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
import cvxpy as cp
from visualization_msgs.msg import Marker, MarkerArray
from mocap4r2_msgs.msg import RigidBodies

# from tf_transformations import euler_from_quaternion, quaternion_matrix


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
        self._odom_sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.odom_rb_callback, qos_profile)
        # self._odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self._marker_sub = self.create_subscription(MarkerArray, '/marker/ellipses', self.marker_callback, qos_profile)


        # Lengths of links in the four_links xacro file
        self._link_len = np.array([0.225, 0.275])

        # Time period
        self._dt = 0.2  # seconds

        self._cur_pos = np.array([0.0, 0.0])
        self._yaw = 0.0
        self._des_pos = np.array([0.0, 0.0])

        self._obs_pos = np.array([2.5, 1.0])

        self.ellipses = np.array([])



        self.A = np.zeros((2,))
        self.b = 0.0
        self.P = np.eye(2)
        self.u = cp.Variable(2)

        self._odom_flag = False
        self._cons_flag = False



        # Create a timer to call a function repeatedly over the time period
        self._timer = self.create_timer(self._dt, self.cmdloop_callback)
        self._start_time = Clock().now().nanoseconds/1E9

    def cmdloop_callback(self):
        """
        Timed callback function to publish the joint angles for the end-effectors to track circles
        """

        # Create a goal message
        goal_msg = Twist()

        posErr  = self._des_pos - self._cur_pos
        # print(f'Poserr: {posErr[0]:.2f}: {posErr[1]:.2f}')
        if la.norm(posErr) < 0.0001 or self._odom_flag==False:
            goal_msg.linear.x = 0.0
            goal_msg.linear.y = 0.0

        else:
            if la.norm(posErr) > 0.5:
                yaw_sp = np.arctan2(posErr[1], posErr[0])
                yawErr = yaw_sp-self._yaw
                yawErr = (yawErr + np.pi) % (2 * np.pi) - np.pi

                goal_msg.angular.z = np.minimum(np.maximum(0.5*yawErr, -0.5), 0.5)



            R = np.array([[np.cos(self._yaw), np.sin(self._yaw)], [-np.sin(self._yaw), np.cos(self._yaw)]])
            u_nom = 2 * posErr
            u_nom = np.array([0.5*np.cos(self._yaw), 0.5*np.sin(self._yaw)])

            # print(f'Pre filter: {u_nom[0]:.2f} {u_nom[1]:.2f}')
            self.genConsMatrix()


            if self._cons_flag:
                u_f = self.filter(u_nom)
            else:
                u_f = u_nom

            # print(f'Post filter: {u_f[0]:.2f} {u_f[1]:.2f}')
            u_f = R@(u_f)
            goal_msg.linear.x = np.minimum(np.maximum(u_f[0], -0.3), 0.3)
            goal_msg.linear.y = np.minimum(np.maximum(u_f[1], -0.3), 0.3)
            goal_msg.angular.z = np.minimum(np.maximum(np.arctan2(u_f[1],u_f[0]), -0.5), 0.5)


        self._joint_pub.publish(goal_msg)
        print(f'Command: {goal_msg.linear.x:.2f}: {goal_msg.linear.y:.2f}')

    def filter(self, u_nom):
        """
        Control Barrier Function implementation
        """
        # errCyl = self._cur_pos - self._obs_pos
        # h1 = errCyl[0]*errCyl[0] + 0.25*errCyl[1]*errCyl[1] - 1.0
        # dh1dx = 2*errCyl[0]
        # dh1dy = 2*errCyl[1]

        # self.A = np.array([dh1dx, dh1dy])
        # self.b = np.array([-3.0*h1])
        try:
            constraints = [self.A@self.u >= self.b]
            prob = cp.Problem(cp.Minimize(cp.quad_form(self.u-u_nom, self.P)), constraints)
            try:
                result = prob.solve()
                u = self.u.value
                # print("Problem status:", prob.status)
            except cp.error.SolverError:
                print("Solver Error: Holding the position")
                u = np.array([0.0, 0.0])


        except ValueError:
            print(f"[{self.name}_lib]: Constraint matrices have incompatible dimensions {self.A.shape}:{self.B.shape}")
            self.landFlag = True
            u = np.array([0.0, 0.0])


        try:
            return np.array([u[0], u[1]])
        except TypeError:
            print('Optimizer error')
            return np.array([0.0, 0.0])


    def odom_callback(self, msg):
        self._cur_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._yaw = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))
        if not self._odom_flag:
            # self._des_pos = np.array([self._cur_pos[0], self._cur_pos[1]])
            self._odom_flag = True
            print(self._yaw*180/np.pi)

    def odom_rb_callback(self, msg):
        rbs = msg.rigidbodies
        for rb in rbs:
            if rb.rigid_body_name == "unitree1.unitree1":
                self._cur_pos = np.array([rb.pose.position.x, rb.pose.position.y])
                q = np.array([rb.pose.orientation.x, rb.pose.orientation.y,
                                rb.pose.orientation.z, rb.pose.orientation.w])
                self._yaw = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))

    def marker_callback(self, msg):
        markers = msg.markers
        self.ellipses = np.array([])
        no_obs = len(markers)
        if no_obs > 0:
            self.ellipses = np.zeros((no_obs, 5))
            for i, marker in enumerate(markers):
                self.ellipses[i,0] = marker.pose.position.x
                self.ellipses[i,1] = marker.pose.position.y
                self.ellipses[i,2] = (marker.scale.x)*(marker.scale.x)/4
                self.ellipses[i,3] = (marker.scale.y)*(marker.scale.y)/4

                q = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
                # print(q)

                self.ellipses[i,4] = np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[2]*q[2] + q[1]*q[1])))
                # self.ellipses[i,4] = 0.0



    def genConsMatrix(self):
        if self.ellipses.size > 0:
            # self.ellipses = self.ellipses[0,:].reshape((1,-1))
            self.cons = np.zeros((len(self.ellipses), 3))
            for i in range(len(self.ellipses)):
                errPos = self._cur_pos[:2] -  self.ellipses[i,:2]
                x_term = errPos[0]
                y_term = errPos[1]
                # print(f'ErrPos: {errPos[0]}')
                x_term = errPos[0]*np.cos(self.ellipses[i,4]) + errPos[1]*np.sin(self.ellipses[i,4])
                y_term = -errPos[0]*np.sin(self.ellipses[i,4]) + errPos[1]*np.cos(self.ellipses[i,4])
                h = (x_term*x_term/self.ellipses[i,2]) + (y_term*y_term/self.ellipses[i,3]) - 1.0
                # if h > 0.0:
                #     print(f'H: {h}')
                #     print(f'Ellipse: {self.ellipses[i,0]:.2f}, {self.ellipses[i,1]:.2f}, {self.ellipses[i,2]:.2f}, {self.ellipses[i,3]:.2f}, {self.ellipses[i,4]:.2f}')
                #     print(f'Errors: {errPos[0]:.3f},  {errPos[1]:.3f}')
                #     print(f'Terms: {x_term:.3f},  {y_term:.3f}')
                dhdx = 2*(x_term*np.cos(self.ellipses[i,4])/self.ellipses[i,2] - y_term*np.sin(self.ellipses[i,4])/self.ellipses[i,3])
                dhdy = 2*(x_term*np.sin(self.ellipses[i,4])/self.ellipses[i,2] + y_term*np.cos(self.ellipses[i,4])/self.ellipses[i,3])
                # dhdx = x_term/self.ellipses[i,2]
                # dhdy = y_term/self.ellipses[i,3]

                self.cons[i, 0] = dhdx
                self.cons[i, 1] = dhdy
                self.cons[i, 2] = -3.1*h
            self.A = self.cons[:,:2]
            self.b = self.cons[:,2]
            self._cons_flag = True

        else:
            self._cons_flag = False
            # print('Cons flag false')

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