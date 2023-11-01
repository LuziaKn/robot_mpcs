#!/usr/bin/env python3
from typing import Union
import numpy as np
from robotmpcs.utils.utils import parse_setup

import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from robotmpcs.planner.mpcPlanner import MPCPlanner
from mpscenes.goals.goal_composition import GoalComposition
from mpscenes.obstacles.sphere_obstacle import SphereObstacle


def get_rotation(pose: Pose) -> float:
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw


class MPCPlannerNode(Node):

    def __init__(self):
        super().__init__("mpc_planner_node")
        self.get_logger().info('MPC Planner Node is running.')

        # Load parameters from the 'my_node' namespace
        self.declare_parameter('mpc.time_step', 0.2)
        self.declare_parameter('mpc.model_name', 'jackal')
        self.declare_parameter('robot.end_link', 'ee_link')

        robot_type = self.get_parameter('mpc.model_name').get_parameter_value().string_value

        package_name = 'robotmpcs_ros2'
        test_setup = get_package_share_directory(package_name) + "/config/" + robot_type + "_mpc_config.yaml"
        self._config = parse_setup(test_setup)
        self._config = self._config['mpc_planner_node']['ros__parameters']['mpc']



        # Retrieve the path to the YAML configuration file from the parameter



        self.establish_ros_connections()
        self._dt = self.get_parameter('mpc.time_step').get_parameter_value().double_value
        self._rate = self.create_rate(1 / self._dt)
        self.get_logger().info(f'dt: {self._dt}')
        self.init_scenario()
        self.init_planner()
        self.set_mpc_parameter()
        self.init_arrays()
        self.create_timer(self._dt, self.run)

    def init_scenario(self):
        self._goal = None
        obst1Dict = {
            "type": "sphere",
            "geometry": {"position": [400.0, -1.5, 0.0], "radius": 1.0},
        }
        sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)
        self._obstacles = [sphereObst1]
        self._r_body = 0.6
        self._limits = np.array([
            [-10, 10],
            [-10, 10],
            [-10, 10],
        ])
        self._limits_u = np.array([
            [-10, 10],
            [-10, 10],
        ])

    def init_arrays(self):
        self._action = np.zeros(2)
        self._q = np.zeros(3)
        self._qudot = np.zeros(2)
        self._qdot = np.zeros(3)

    def init_planner(self):
        self._robot_type = self.get_parameter('mpc.model_name').get_parameter_value().string_value
        package_name = 'robotmpcs_ros2'


        self._solver_directory = '/home/luzia/code/robot_mpcs/ros2_bridge/src/robotmpcs_ros2/robotmpcs_ros2/solvers/'
        print(self._solver_directory)

        self._planner = MPCPlanner(
            self._robot_type,
            self._solver_directory,
            **self._config)
        self._planner.concretize()
        self._planner.reset()

    def set_mpc_parameter(self):
        constraints = self._config['constraints']
        objectives = self._config['objectives']

        for objective in objectives:
            if objective == 'GoalReaching':
                try:
                    self._planner.setGoalReaching(self._goal)
                except AttributeError:
                    print('The required attributes for setting ' + objective + ' are not defined')
            elif objective == 'ConstraintAvoidance':
                try:
                    self._planner.setConstraintAvoidance()
                except KeyError:
                    print('The required attributes for setting ' + objective + ' are not defined in the config file')
            else:
                print('No function to set the parameters for this objective is defined')

        for constraint in constraints:
            if constraint == 'JointLimitConstraints':
                try:
                    self._planner.setJointLimits(np.transpose(self._limits))
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            elif constraint == 'VelLimitConstraints':
                try:
                    self._planner.setVelLimits(np.transpose(self._limits_vel))
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            elif constraint == 'InputLimitConstraints':
                try:
                    self._planner.setInputLimits(np.transpose(self._limits_u))
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            elif constraint == 'LinearConstraints':
                try:
                    self._planner.setLinearConstraints(self._lin_constr, self._r_body)
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            elif constraint == 'RadialConstraints':
                try:
                    self._planner.setRadialConstraints(self._obstacles, self._r_body)
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            elif constraint == 'SelfCollisionAvoidanceConstraints':
                try:
                    self._planner.setSelfCollisionAvoidanceConstraints(self._r_body)
                except AttributeError:
                    print('The required attributes for setting ' + constraint + ' are not defined')
            else:
                print('No function to set the parameters for this constraint type is defined')

    def establish_ros_connections(self):
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 1)
        self._goal_sub = self.create_subscription(Float64MultiArray, "/mpc/goal", self._goal_cb, 1)

    def _goal_cb(self, goal_msg: Float64MultiArray):

        goal_position = goal_msg.data
        # if len(goal_position) != 2:
        #     rospy.logwarn("Goal ignored because of dimension missmatch")
        goal_dict = {
            "subgoal0": {
                "weight": 1.0,
                "is_primary_goal": True,
                "indices": [0, 1],
                "parent_link": 'origin',
                "child_link": self.get_parameter('robot.end_link').get_parameter_value().string_value,
                "desired_position": [1.0, 1.0],
                "epsilon": 0.4,
                "type": "staticSubGoal"
            }
        }

        self._goal = GoalComposition(name="goal1", content_dict=goal_dict)
        self._planner.setGoalReaching(self._goal)
        print("GOAL SET")

    def _odom_cb(self, odom_msg: Odometry):
        self._q = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            get_rotation(odom_msg.pose.pose),
        ])
        self._qdot = np.array([
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.angular.z,
        ])

    def act(self):
        vel_action = self._action * self._dt + self._qudot
        cmd_msg = Twist()
        cmd_msg.linear.x = 1.0#vel_action[0]
        cmd_msg.angular.z = 0.0#vel_action[1]
        self.get_logger().info(str(vel_action))
        self._cmd_pub.publish(cmd_msg)
        self._qudot = vel_action

    def run(self):
        while rclpy.ok():
            if self._goal:
                self._action, _ = self._planner.computeAction(self._q, self._qdot, self._qudot)
            self.get_logger().info(str(self._action))
            self.get_logger().info("action computed")
            self.act()



def main(args=None):
    rclpy.init(args=args)
    mpc_planner_node = MPCPlannerNode()
    #mpc_planner_node.run()
    rclpy.spin(mpc_planner_node)
    mpc_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
