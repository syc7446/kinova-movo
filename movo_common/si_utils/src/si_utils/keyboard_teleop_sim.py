#!/usr/bin/env python
# Author: Silvia Knappe
# Control movo with keyboard teleop, moves base and torso!

import rospy
import actionlib
from copy import copy

from geometry_msgs.msg import Twist
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd, LinearActuatorCmd
from control_msgs.msg import JointTrajectoryControllerState
import sys, select, termios, tty, threading
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class KeyboardTeleop:
    def __init__(self):
        self.msg = """
            Use the keyboard to move around!
            --------------------------------
                      U
                      W
                H A  S,J  D K
                      X
                      M
                to translate (U,H,K,M: fast move)
                ------------
                Y       I           R
                Q       E           
                                    F
                to rotate       to move torso
                (Y,I: fast rotation)
            --------------------------------
            """

        self.basemoves = {
            'u':[0.5,0,0],
            'w':[0.2,0,0],
            'h':[0,0.5,0],
            'a':[0,0.2,0],
            's':[0,0,0],
            'j':[0,0,0],
            'm':[-0.5,0,0],
            'x':[-0.2,0,0],
            'k':[0,-0.5,0],
            'd':[0,-0.2,0],
            'y':[0,0,1.0],
            'q':[0,0,0.5],
            'i':[0,0,-1.0],
            'e':[0,0,-0.5]
        }

        self.torsomoves = {
            'r' : .1,
            'f' : -.1
        }

        self.base_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=1)
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=1)
        self.r = rospy.Rate(10)
        self.thread = threading.Thread(target=self.setGP)
        self.thread_r = rospy.Rate(1)
        self.kill = False
        self.previous_pos = 0
        self.current_pos = None
        self.check_joint_state = False
        rospy.Subscriber('/movo/torso_controller/state', JointTrajectoryControllerState, self.joint_state_cb)

        self.settings = termios.tcgetattr(sys.stdin)

    def joint_state_cb(self, msg):
        if self.current_pos is None:
            self.current_pos = msg.actual.positions
        if not self.check_joint_state:
            self.previous_pos = self.current_pos[0]
            self.check_joint_state = True

    def getKey(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd,termios.TCSADRAIN,old)
        return ch

    def setGP(self):
        while not rospy.is_shutdown() and not self.kill:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = TRACTOR_REQUEST
            self.cfg_cmd.header.stamp = rospy.get_rostime()
            self.cfg_pub.publish(self.cfg_cmd)
            self.thread_r.sleep()

    def start(self):
        print self.msg
        self.thread.start()

        while not rospy.is_shutdown():
            if self.check_joint_state:
                try:
                    twist = Twist()
                    lincmd = LinearActuatorCmd()

                    key = self.getKey()
                    v_x = 0
                    v_y = 0
                    a_z = 0
                    torso_dz = 0

                    if key in self.basemoves:
                        v_x = self.basemoves[key][0]
                        v_y = self.basemoves[key][1]
                        a_z = self.basemoves[key][2]

                    elif key in self.torsomoves:
                        torso_dz = self.torsomoves[key]

                    else:
                        v_x = 0
                        v_y = 0
                        a_z = 0
                        torso_dz = 0
                        if (key == '\x03'):
                            print "Goodbye!"
                            self.kill = True
                            break

                    rospy.loginfo("Velocity command: (v_x:%.2f, v_y:%.2f, a_z:%.2f)", v_x, v_y, a_z)
                    twist.linear.x = v_x
                    twist.linear.y = v_y
                    twist.angular.z = a_z

                    if (key is 'r') or (key is 'f'):
                        if self.current_pos[0]+torso_dz > 0.4:
                            print "Torso already reached to the maximum height. Can't move!"
                            torso_dz = 0
                        elif self.current_pos[0]+torso_dz < 0:
                            print "Torso already reached to the minimum height. Can't move!"
                            torso_dz = 0

                    if torso_dz is not 0:
                        traj = TorsoJTAS()
                        traj.add_point(list(self.current_pos), 0.0)
                        
                        total_time = 0.0
                        for i in range(0,10):
                            
                            pos = self.current_pos[0]+torso_dz # Maximum height: 0.4.
                            vel = 0.5
                            
                            dt = abs(pos-self.previous_pos)/vel
                            total_time+=dt
                       
                            traj.add_point([pos],total_time)
                            
                        traj.start()
                        traj.wait(total_time)

                    self.base_pub.publish(twist)
                    self.previous_pos = self.current_pos[0]+torso_dz
                    self.current_pos = None

                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

                except Exception as e:
                    print(e)

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

if __name__ == "__main__":
    rospy.init_node('keyboard_teleop_sim')
    kt = KeyboardTeleop()
    kt.start()
