#!/usr/bin/env python

import argparse
import sys
from copy import copy
import rospy
import actionlib
import math
import random
import numpy as np
from movo.system_defines import TRACTOR_REQUEST
from geometry_msgs.msg import Twist
from movo_msgs.msg import ConfigCmd
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from sensor_msgs.msg import JointState

from control_msgs.msg import JointTrajectoryControllerState

class HeadJTAS(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['pan_joint','tilt_joint']

class TorsoJTAS(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            'movo/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['linear_joint']

class ActionApply(object):
    def __init__(self, grid_x, grid_y, grid_z, trans_vel, ang_vel, goal_pose_tolerance, goal_angle_tolerance, \
        grid_x_val, grid_y_val, grid_z_val, heading_angle):
        self._trans_vel = trans_vel
        self._ang_vel = ang_vel
        self._goal_pose_tolerance = goal_pose_tolerance
        self._goal_angle_tolerance = goal_angle_tolerance
        self._cur_robot_pose_sub = rospy.Subscriber('/movo/odometry/local_filtered', Odometry, self.cur_robot_pose_cb)
        self._base_vel_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)

        # Set robot mode to active base motion
        self._cfg_cmd = ConfigCmd()
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

        self._goal_pose_x = grid_x[grid_x_val]
        self._goal_pose_y = grid_y[grid_y_val]
        self._goal_pose_z = grid_z[grid_z_val]
        self._goal_heading_angle = heading_angle

        self._cur_robot_pose_check = False

    def cur_robot_pose_cb(self, data):
        self._cur_robot_pose_x = data.pose.pose.position.x
        self._cur_robot_pose_y = data.pose.pose.position.y
        self._cur_robot_pose_z = data.pose.pose.position.z
        cur_euler = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, \
            data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self._cur_robot_heading_angle = math.degrees(cur_euler[2])
        self._cur_robot_pose_check = True

    def compute_action(self):
        if self._cur_robot_pose_check:
            rospy.loginfo("Goal pose is (%.2f, %.2f, %.2f, %.2f)", self._goal_pose_x, self._goal_pose_y, \
                self._goal_pose_z, self._goal_heading_angle)
            diff_x = self._goal_pose_x-self._cur_robot_pose_x
            diff_y = self._goal_pose_y-self._cur_robot_pose_y

            if diff_x >= 0:
                vel_cmd_x = self._trans_vel
            else:
                vel_cmd_x = -1*self._trans_vel
            if diff_y >= 0:
                vel_cmd_y = self._trans_vel
            else:
                vel_cmd_y = -1*self._trans_vel

            self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self._cfg_cmd.gp_param = TRACTOR_REQUEST
            self._cfg_cmd.header.stamp = rospy.get_rostime()
            self._cfg_pub.publish(self._cfg_cmd)
            rospy.sleep(0.1)

            twist_cmd = Twist()
            twist_cmd.linear.x = vel_cmd_x
            twist_cmd.linear.y = 0.0
            twist_cmd.linear.z = 0.0
            twist_cmd.angular.x = 0.0
            twist_cmd.angular.y = 0.0
            twist_cmd.angular.z = 0.0

            rate = rospy.Rate(100)
            rospy.loginfo("Robot starts moving along x axis.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_pose_x-self._goal_pose_x) < self._goal_pose_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

            twist_cmd.linear.x = 0.0
            twist_cmd.linear.y = vel_cmd_y

            rospy.loginfo("Robot starts moving along y axis.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_pose_y-self._goal_pose_y) < self._goal_pose_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

            twist_cmd.linear.y = 0.0
            if self._goal_heading_angle >= 0:
                twist_cmd.angular.z = math.radians(self._ang_vel)
            else:
                twist_cmd.angular.z = -1*math.radians(self._ang_vel)

            rospy.loginfo("Robot starts rotating.")
            while True:
                self._base_vel_pub.publish(twist_cmd)
                if abs(self._cur_robot_heading_angle-self._goal_heading_angle) < self._goal_angle_tolerance:
                    self.motion_stop()
                    break
                rate.sleep()

    def motion_stop(self, duration=1.0):
        self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        self._cfg_cmd.gp_param = 0
        self._cfg_cmd.header.stamp = rospy.get_rostime()
        self._cfg_pub.publish(self._cfg_cmd)

        rospy.logdebug("Stopping velocity command to movo base from BaseVelTest class ...")
        try:
            r = rospy.Rate(10)
            start_time = rospy.get_time()
            while (rospy.get_time() - start_time) < duration:
                self._base_vel_pub.publish(Twist())
                r.sleep()
        except Exception as ex:
            print "Message of base motion failed to be published, error message: ", ex.message
            pass

    def check_cur_robot_pose(self):
        return self._cur_robot_pose_check