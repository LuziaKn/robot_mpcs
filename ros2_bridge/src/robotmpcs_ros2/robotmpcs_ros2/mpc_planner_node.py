#!/usr/bin/env python3
from typing import Union
import numpy as np
import time
import asyncio
from robotmpcs.utils.utils import parse_setup
import math

import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist, Point, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import NavigateToPose

from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from tf_transformations import euler_from_quaternion

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from robotmpcs.planner.mpcPlanner import MPCPlanner
from robotmpcs.utils.utils import check_goal_reaching
from mpscenes.goals.goal_composition import GoalComposition
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from ros2_bridge.src.ros_tools.ros_tools.ros_visuals_py import ROSMarkerPublisher

from forces_pro_server.srv import CallForcesPro

from copy import deepcopy
from robotmpcs.utils.utils import shift_angle_casadi

def rad2deg(angle_rad):
    return angle_rad * 180/np.pi

def get_rotation(pose: Pose) -> float:
    orientation_q = pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    return yaw

use_visualization = True
class MPCPlannerNode(Node):

    def __init__(self):
        super().__init__("mpc_planner_node")
        #self.get_logger().info('MPC Planner Node is running.')
        
    
        self.cli = self.create_client(CallForcesPro, 'call_forces_pro') # Creates the client in ROS2
        

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        

        # Load parameters from the 'my_node' namespace
        self.declare_parameter('mpc.time_step', 0.05)
        self.declare_parameter('mpc.model_name', 'pointRobot')
        self.declare_parameter('robot.end_link', 'ee_link')
        self.declare_parameter('package_path', 'default')
        self._hostname = "luzia_Alienware_m15_R4"

        robot_type = self.get_parameter('mpc.model_name').get_parameter_value().string_value
        self._package_path = self.get_parameter('package_path').get_parameter_value().string_value

        package_name = 'robotmpcs_ros2'
        test_setup = self._package_path + "/config/" + robot_type + "_mpc_config.yaml"
        self._config = parse_setup(test_setup)
        self._config = self._config['mpc_planner_node']['ros__parameters']['mpc']

        self._remaining_distance = 100
        self._success = False
        
        self.establish_ros_connections()
        
        self.tf_buffer = Buffer()
        self._listener = TransformListener(self.tf_buffer, self)
        self.tf_init_timer = self.create_timer(5.0, self.tf_init)
        
        self._dt = self.get_parameter('mpc.time_step').get_parameter_value().double_value
        self._rate = self.create_rate(1 / self._dt)
        self.get_logger().info(f'dt: {self._dt}')
        self.init_scenario()
        self.init_planner()
        self.set_mpc_parameter()
        self.init_arrays()
        if use_visualization:
            self.init_visuals()
        #self.create_timer(self._dt, self.execute_callback)
        self.create_timer(self._dt, self._get_action)
        self.present_solver_output = np.zeros((10,self._planner._nx + self._planner._nu))
        self._exitflag = 1
        self.output = np.ones((self._planner._time_horizon,  self._planner._nx + self._planner._nu))

    def tf_init(self):
        self.tf_init_timer.cancel()

    def ros_solver_function(self,problem):
        
        # Sets up the request
        self.req = CallForcesPro.Request()
    
        self.req.x0.data =  [x for x in problem["x0"]]  
        self.req.xinit.data =  [x for x in problem["xinit"]]   
        self.req.params.data = [x for x in problem["all_parameters"]]  
        #self.get_logger().info("x0: " + str(self.req.x0.data))
        #self.get_logger().info("xinit: " + str(self.req.xinit.data))
        #self.get_logger().info("params: " + str(self.req.params.data))

        #self.get_logger().info("before spin reached")
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self._get_solver_output_callback)
        #rclpy.spin_until_future_complete(self, self.future, timeout_sec = None)
        #self.get_logger().info("after spin reached")
        return self.present_solver_output, self._exitflag
    
    def _get_solver_output_callback(self, future) -> np.ndarray:
        """
        Callback when the request for the solver output was completed.
        Saves present solver output class attributes and publishes the solver outputs.
        """
        resp = future.result()

        if resp is None:
            self.get_logger().error(
                "failed")
            return self.present_solver_output, self._exitflag


        self.present_solver_output =  np.array(resp.output.data, dtype = np.float64)
        self._exitflag = resp.exit_code

        return self.present_solver_output, self._exitflag

    
    def init_scenario(self):
        self._goal = None
        obst1Dict = {
            "type": "sphere",
            "geometry": {"position": [400.0, -1.5, 0.0], "radius": 1.0},
        }
        sphereObst1 = SphereObstacle(name="simpleSphere", content_dict=obst1Dict)
        self._obstacles = [sphereObst1]
        self._r_body = 0.6
        # self._limits = np.array([
        #     [-50, 50],
        #     [-50, 50],
        #     [-6, 6],
        # ])
        self._limits_u = np.array([
            [-2, 2],
            [-2, 2],
            [-3, 3],
        ])
        
        self._limits_vel = np.array([
            [-1, 1],
            [-1, 1],
            [-3, 3],
        ])
    
    if use_visualization:
        def init_visuals(self):
            self._frame_id = self._hostname + "_map"
            self.marker_publisher = ROSMarkerPublisher(self, "ros_tools/mpc/visuals", self._frame_id, 15)
            
            self._visuals_goal = self.marker_publisher.get_circle(self._frame_id)
            self._visuals_goal.set_color(4, 1.0)
            self._visuals_goal.set_scale(0.5, 0.5, 0.01)
            self._visuals_goal.add_marker(Point(x=10.0, y=0.0, z=2.0))
            
            self._visuals_goal_angle = self.marker_publisher.get_line(self._frame_id)
            self._visuals_goal_angle.set_color(4, 1.0)
            self._visuals_goal_angle.set_scale(0.1)
            self._visuals_goal_angle.add_line(Point(x=10.0, y=0.0, z=2.0), Point(x=11.0, y=0.0, z=2.0))
            
            self._visuals_plan_circle = self.marker_publisher.get_circle(self._frame_id)
            self._visuals_plan_circle.set_color(7, 0.5)
            self._visuals_plan_circle.set_scale(0.5, 0.5, 0.01)
            self._visuals_plan_circle.add_marker(Point(x=10.0, y=0.0, z=2.0))
            

    def init_arrays(self):
        self._action = np.zeros(3)
        self._q = np.zeros(3)
        self._qudot = np.zeros(3)
        self._qdot = np.zeros(3)

    def init_planner(self):
        self._robot_type = self.get_parameter('mpc.model_name').get_parameter_value().string_value
        package_name = 'robotmpcs_ros2'
        self.get_logger().info(self._package_path)

        self._solver_directory = self._package_path + '/../' + 'forces_pro_server' + '/solver/'
        print(self._solver_directory)

        self._planner = MPCPlanner(
            robotType = self._robot_type,
            solversDir = self._solver_directory,
            solver_function = self.ros_solver_function,
            ros_flag= True,
            **self._config)
        self._planner.concretize()
        self._planner.reset()

    def set_mpc_parameter(self):
        constraints = self._config['constraints']
        objectives = self._config['objectives']
        print(constraints)
        print(objectives)

        for objective in objectives:
            if objective == 'GoalPoseReaching':
                try:
                    active_states = np.zeros((self._planner._time_horizon,1))
                    active_states[-1] = 1.0
                    self._planner.setGoalReaching(self._goal, active_states)
                except AttributeError:
                    print('The required attributes for setting ' + objective + ' are not defined')
            elif objective == 'ConstraintAvoidance':
                try:
                    self._planner.setConstraintAvoidance()
                except KeyError:
                    print('The required attributes for setting ' + objective + ' are not defined in the config file')
            elif objective == 'HeadingReaching':
                try:
                    active_states = np.ones((self._planner._time_horizon,1))
                    active_states[-1] = 0.0
                    self._planner.setHeadingReaching(active_states)
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
        
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate2pose',
            self.execute_callback)
                
        self._cmd_pub = self.create_publisher(Twist, "/luzia_Alienware_m15_R4/platform/command_limiter_node/base/cmd_vel_in_navigation", 1)
        #self._odom_sub = self.create_subscription(Odometry, "/luzia_Alienware_m15_R4/platform/odometry", self._odom_cb, 1)
        #self._nmcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/NMCLPose", self._nmcl_cb, 1)
        #self._goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 1)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        
    def _goal_cb(self, goal_msg):
        goal_position = goal_msg.data
        goal_dict = {
            "subgoal0": {
                "weight": 1.0,
                "is_primary_goal": True,
                "indices": [0, 1],
                "parent_link": 'origin',
                "child_link": self.get_parameter('robot.end_link').get_parameter_value().string_value,
                "desired_position": list(goal_position),
                "epsilon": 0.4,
                "type": "staticSubGoal"
            }
        }

        self._goal = GoalComposition(name="goal1", content_dict=goal_dict)
        self._planner.setGoalReaching(self._goal)

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
        
    def _nmcl_cb(self, msg: PoseWithCovarianceStamped):
        self._q = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            get_rotation(msg.pose.pose),
        ])
        #self.get_logger().info('Pose #######' + str(get_rotation(msg.pose.pose)))
        
        
    def execute_callback(self, goal_handle):

        self.get_logger().info('Received goal request')

        goal_pose = goal_handle.request.pose
        
        
        goal_dict = {
            "subgoal0": {
                "weight": 1.0,
                "is_primary_goal": True,
                "indices": [0, 1],
                "parent_link": 'origin',
                "child_link": self.get_parameter('robot.end_link').get_parameter_value().string_value,
                "desired_position": [goal_pose.pose.position.x, goal_pose.pose.position.y],
                "angle": get_rotation(goal_pose.pose), 
                "epsilon": 0.2,
                "type": "staticSubGoal"
            }
        }
        
        self._goal = GoalComposition(name="goal1", content_dict=goal_dict)
        self._planner.setGoalReaching(self._goal)
        
        # Publish feedback

        # while self._success==False:
        #     feedback = NavigateToPose.Feedback()
        #     #feedback.distance_remaining = self._remaining_distance
        #     goal_handle.publish_feedback(feedback)
        #     time.sleep(1)

  
        goal_handle.succeed()

        
    
        # within_goal_tolerance = check_goal_reaching(self._q[:2], self._goal)
        # while not within_goal_tolerance:
        #     within_goal_tolerance = check_goal_reaching(self._q[:2], self._goal)
        
        result = NavigateToPose.Result()
        
        
        
                
            
        return result
    
    def _get_action(self):
        
        if self._goal is not None:
            
            #self.get_logger().info(f'Curent state: {self._q}')
            


            try:
                map_to_base = self.tf_buffer.lookup_transform(
                    self._hostname + "_map",
                    self._hostname + "_base_link",
                    rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().warn(f'Could not get transform: {ex}')
            
            orientation_list = [
                map_to_base.transform.rotation.x,
                map_to_base.transform.rotation.y,
                map_to_base.transform.rotation.z,
                map_to_base.transform.rotation.w
            ]
            _, _, yaw = euler_from_quaternion(orientation_list)
            
            
            self._q = np.array([
                map_to_base.transform.translation.x,
                map_to_base.transform.translation.y,
                yaw,
            ])
            
            self._qdot = np.array([0.0,
                                 0.0,
                                 0.0])



            start_time = time.time()
            self._action, self._output, self._exitflag = self._planner.computeAction(self._q, self._qdot)            
            end_time = time.time()
            run_time = end_time - start_time

            # Here we print the result

            #self.get_logger().info("action " + str(self._action))
            #self.get_logger().info("output" + str(self._output))
            #self.get_logger().info("exitflag " + str(self._exitflag))
            #self.get_logger().info("run time " + str(run_time))
            
            primary_goal = self._goal.primary_goal()
            angle_error = shift_angle_casadi(self._goal.primary_goal().angle() - self._q[2])
            self._remaining_distance = np.linalg.norm(self._q[:2] - primary_goal.position())
            if self._remaining_distance <= primary_goal.epsilon() and np.abs(angle_error)<0.1:
                self._success = True
                self.get_logger().info("Task completed successfully")
                self.get_logger().info("Postion Error: " + str(self._remaining_distance))
                self.get_logger().info("Orientation Error: " + str(angle_error))
                


            if use_visualization:
                self.visualize()
            #if self._success == False:
            self.act()
            heading = np.arctan2(self._q[1],self._q[0])
            #self.get_logger().info("heading. robot " + str(rad2deg(heading)))
            #self.get_logger().info("orient. robot " + str(rad2deg(self._q[2])))
            #self.get_logger().info("orient. goal " + str(rad2deg(self._goal.primary_goal().angle())))
            #self.get_logger().info("error angle" + str(rad2deg(angle_error)))
        

    def act(self):
        self.get_logger().info("Control Mode: " + self._config['control_mode'])
        if self._config['control_mode'] == 'acc':
            vel_action = self._action * self._dt + self._qudot
        elif self._config['control_mode'] == 'vel':
            self.get_logger().info("actuate")
            vel_action = self._action
            acc = self._output[0,-self._planner._nu:]
        if self._exitflag > 0:
            self.get_logger().info("feasible")
        elif self._exitflag == 0:
            self.get_logger().info("max num iterations reached")
            vel_action = [0.0, 0.0, 0.0]
        else:
            self.get_logger().info("infeasible")
            vel_action = [0.0, 0.0, 0.0]
        # transfer to robot-frame
        vel_action_ego = deepcopy(vel_action)
        theta = self._q[-1]
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, s), (-s, c)))
        vel_action_ego[:2] = np.dot(R, vel_action[:2])
        vel_action_ego[2] += 0.0
        cmd_msg = Twist()
        cmd_msg.linear.x = vel_action_ego[0]
        cmd_msg.linear.y = vel_action_ego[1]
        cmd_msg.angular.z = vel_action_ego[2]
        
        self.get_logger().info("Vel action x: " + str(cmd_msg.linear.x))
        self.get_logger().info("Vel action y: " + str(cmd_msg.linear.y))
        self.get_logger().info("Vel action angle: " + str(cmd_msg.angular.z))
        
        self._cmd_pub.publish(cmd_msg)
        #self._qudot = acc
        self._qdot = vel_action
        
        self.get_logger().info("Vel action: " + str(vel_action_ego))
        
        

    if use_visualization:
        def visualize(self):
            
            if self._goal is not None:
                goal_position = Point(x=self._goal.primary_goal().position()[0], y=self._goal.primary_goal().position()[1], z=0.01)
                self._visuals_goal.add_marker(goal_position)
                d = 1
                helper_point = Point(x=goal_position.x+d*np.cos(self._goal.primary_goal().angle()), y= goal_position.y+d*np.sin(self._goal.primary_goal().angle()), z=goal_position.z)

                self._visuals_goal_angle.add_line(goal_position,helper_point)
                
                for state in self._output:
                    position = Point(x=state[0], y=state[1], z=0.01)
                    self._visuals_plan_circle.add_marker(position)
                    

            self.marker_publisher.publish()


    



def main(args=None):
    rclpy.init(args=args)
    mpc_planner_node = MPCPlannerNode()
    rclpy.spin(mpc_planner_node)
    mpc_planner_node.marker_publisher.destroy_node()
    mpc_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
