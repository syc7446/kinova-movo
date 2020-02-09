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

'''
Arguments(current_angles, l_r_angle, u_d_angle, ang_vel, wait_time): 
    current_angles: current head angles.
    l_r_angle: left and right (+:cw, -:ccw). 0.78 = 45 degrees, 1.57 = 90 degrees.
    u_d_angle: up and down. 0.78 = 45 degrees, 1.57 = 90 degrees.
    ang_vel: The recommended value is 0.3.
    wait_time: The recommended value is 3.0.
'''
class HeadJTAS(object):
    def __init__(self, current_angles, l_r_angle, u_d_angle, ang_vel, wait_time):
        self._client = actionlib.SimpleActionClient(
            'movo/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()
        self.add_point(list(current_angles), 0.0)

        for i in range(0,1):
            pos = [current_angles[0]+l_r_angle, current_angles[1]+u_d_angle]
            vel = ang_vel
            
            dt = 0.0
            for i in range(2):
                tmp = abs(pos[i])/vel
                if (tmp > dt):
                    dt = tmp
            total_time+=dt
       
            self.add_point(pos,total_time)
            
        self.start()
        self.wait(total_time+wait_time)

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

'''
Arguments(current_angles, height_displacement, ang_vel, wait_time):
    current_angles: current head angles.
    height_displacement: The value can be from -0.4 to 0.4. Note that the possible height of the torso is from 0.0 to 0.4.
    ang_vel: The recommended value is between 0.02 and 0.06.
    wait_time: The recommended value is 0.0.
'''
class TorsoJTAS(object):
    def __init__(self, current_angles, height_displacement, ang_vel, wait_time):
        self._client = actionlib.SimpleActionClient(
            'movo/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        total_time = 0.0
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()
        self.add_point(list(current_angles), 0.0)
    
        total_time = 0.0
        points = [list(current_angles), 0.0]
        for i in range(0,1):
            pos = height_displacement
            vel = ang_vel
            
            dt = abs(pos)/vel
            total_time+=dt
       
            self.add_point([pos],total_time)
            
        self.start()
        self.wait(total_time+wait_time)

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
        rospy.loginfo("timeout = %f", timeout)
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['linear_joint']

'''
Arguments(motion_options, displacement, vel):
    motion_options: six options ('forward', 'backward', 'left', 'right', 'rotationcw', 'rotationccw').
    displacement and vel: If option is 'rotationcw' and 'rotationccw', the recommended velocity is 30. Displacement is in degrees.
                        Otherwise, the recommended velocity is 0.3. The displacement is in meters.
'''
class BaseMotion(object):
    def __init__(self, motion_options, displacement, vel):
        self._base_vel_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)

        # set robot mode to active base motion
        self._cfg_cmd = ConfigCmd()
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

        # for motion sequence command request
        self.dist_cmdList = []
        self.vel_cmdList = []
        self.dist_tolerance = 0.001
        self.rot_tolerance = math.radians(1.0)
        self.dist_vel_tolerance = 0.001
        self.rot_vel_tolerance = math.radians(1.0)
        self.duration_limit = 3600  # a motion dont plan for more than 1hour

        # set robot mode to accept base motion via parameter _cfg_cmd
        self._motion_vel([0.0, 0.0, 0.0], 0.0)
        rospy.loginfo("motion_options = %s", motion_options)

        if motion_options is 'forward':
            self.move_forward(displacement, vel)
        elif motion_options is 'backward':
            self.move_backward(displacement, vel)
        elif motion_options is 'left':
            self.move_left(displacement, vel)
        elif motion_options is 'right':
            self.move_right(displacement, vel)
        elif motion_options is 'rotationcw':
            self.rotate_clock(displacement, vel)
        elif motion_options is 'rotationccw':
            self.rotate_anticlock(displacement, vel)
        else:
            rospy.logerr("ERROR: invalid basemotion command.")
        self.motion_stop()

    def _motion_vel(self, vel_cmd, duration):
        """
        publish velocity command to movo base for given time
        @param vel_cmd: velocity command in [meter, meter, degree/second] along translation x, y and rotation z
        @param duration: second
        @return:
        """
        vel_cmd = map(float, vel_cmd)
        duration = float(duration)

        self._cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
        self._cfg_cmd.gp_param = TRACTOR_REQUEST
        self._cfg_cmd.header.stamp = rospy.get_rostime()
        self._cfg_pub.publish(self._cfg_cmd)
        rospy.sleep(0.1)

        twist_cmd = Twist()
        twist_cmd.linear.x = vel_cmd[0]
        twist_cmd.linear.y = vel_cmd[1]
        twist_cmd.linear.z = 0.0
        twist_cmd.angular.x = 0.0
        twist_cmd.angular.y = 0.0
        twist_cmd.angular.z = math.radians(vel_cmd[2])

        rospy.logdebug("Send velocity command to movo base from BaseVelTest class ...")
        rate = rospy.Rate(100)
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < duration) and not (rospy.is_shutdown()):
            self._base_vel_pub.publish(twist_cmd)
            rate.sleep()

    def motion_dist(self, dist_cmd, vel_cmd=None):
        """
        Command the base to move certain distance
        @param dist_cmd: distance along x, y in meter, rotation along z in degree, distance is absolute value.(Positive)
        @param vel_cmd: velocity during the motion, unit in dist_cmd per second.
        @return:
        """

        # default velocity, velocity is the abosulute value. Negative should be in distance command.
        if vel_cmd is None:
            vel_cmd = [0.1, 0.1, 30]

        dist_cmd = map(math.fabs, map(float, dist_cmd))
        vel_cmd = map(float, vel_cmd)

        # duration is as the action takes most time consumption.
        duration_temp = [0.0, 0.0, 0.0]
        for i in range(3):
            # validation of command input
            if i < 2:  # translation
                if dist_cmd[i] != 0.0 and dist_cmd[i] < self.dist_tolerance:
                    rospy.logwarn("distance command " + str(dist_cmd[i]) + " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0
                elif vel_cmd[i] != 0.0 and math.fabs(vel_cmd[i]) < self.dist_vel_tolerance:
                    rospy.logwarn("translation velocity command " + str(vel_cmd[i]) + 
                                  " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0

            elif i == 2:  # rotation
                if dist_cmd[i] != 0.0 and dist_cmd[i] < self.rot_tolerance:
                    rospy.logwarn("rotation command " + str(dist_cmd[i]) + " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0
                elif vel_cmd[i] != 0.0 and math.fabs(vel_cmd[i]) < self.rot_vel_tolerance:
                    rospy.logwarn("rotation velocity command " + str(vel_cmd[i]) + 
                                  " is below threshold, execution cancelled.")
                    dist_cmd[i] = 0.0
                    vel_cmd[i] = 0.0

            if vel_cmd[i] == 0.0:
                duration_temp[i] = 0.0
            else:
                duration_temp[i] = math.fabs(dist_cmd[i] / vel_cmd[i])

        duration = max(duration_temp)

        # revise vel_cmd so that all motion finish at same duration.
        if duration == 0.0:
            rospy.logwarn("duration is zero")
            return
        elif duration > self.duration_limit:
            rospy.logwarn("motion duration exceeded " + str(self.duration_limit) + " seconds, execution cancelled")
        else:
            vel_cmd_mod = [0.0, 0.0, 0.0]
            for i in range(0, len(dist_cmd)):
                vel_cmd_mod[i] = dist_cmd[i]/duration * math.copysign(1.0, vel_cmd[i])

            self._motion_vel(vel_cmd_mod, duration)

    def move_forward(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([math.fabs(meters), 0.0, 0.0], [math.fabs(speed), 0.0, 0.0])

    def move_backward(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([math.fabs(meters), 0.0, 0.0], [-1.0*math.fabs(speed), 0.0, 0.0])

    def move_left(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([0.0, math.fabs(meters), 0.0], [0.0, math.fabs(speed), 0.0])

    def move_right(self, meters, speed=None):
        if speed is None:
            speed = 0.1
        self.motion_dist([0.0, math.fabs(meters), 0.0], [0.0, -1.0*math.fabs(speed), 0.0])

    def rotate_clock(self, degrees, speed=None):
        if speed is None:
            speed = 30
        self.motion_dist([0.0, 0.0, math.fabs(degrees)], [0.0, 0.0, math.fabs(speed)])

    def rotate_anticlock(self, degrees, speed=None):
        if speed is None:
            speed = 30
        self.motion_dist([0.0, 0.0, math.fabs(degrees)], [0.0, 0.0, -1.0*math.fabs(speed)])

    def add_motion_to_list(self, dist_cmd, vel_cmd=None):
        self.dist_cmdList.append(dist_cmd)
        self.vel_cmdList.append(vel_cmd)

    def clear_motion_list(self):
        self.dist_cmdList = []
        self.vel_cmdList = []

    def move_sequence(self):
        for i in range(0, len(self.dist_cmdList)):
            self.motion_dist(self.dist_cmdList[i], self.vel_cmdList[i])

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