#!/usr/bin/env python

import argparse
import sys
from copy import copy
import rospy
import actionlib
import math

from control_msgs.msg import JointTrajectoryControllerState
from utils.sim_motion import BaseMotion
from utils.sim_motion import HeadJTAS
from utils.sim_motion import TorsoJTAS

def main():
    rospy.init_node('sim_action_demo')
    
    # Base motion.
    '''
    Arguments(motion_options, displacement, vel):
        motion_options: six options ('forward', 'backward', 'left', 'right', 'rotationcw', 'rotationccw').
        displacement and vel: If option is 'rotationcw' and 'rotationccw', the recommended velocity is 30. Displacement is in degrees.
                            Otherwise, the recommended velocity is 0.3. The displacement is in meters.
    '''
    # BaseMotion('forward',1.5,0.3)

    # Torso motion.
    '''
    Arguments(current_angles, height_displacement, ang_vel, wait_time):
        current_angles: current head angles.
        height_displacement: The value can be from -0.4 to 0.4. Note that the possible height of the torso is from 0.0 to 0.4.
        ang_vel: The recommended value is between 0.02 and 0.06.
        wait_time: The recommended value is 0.0.
    '''
    cur_torso = rospy.wait_for_message("/movo/torso_controller/state", JointTrajectoryControllerState)
    current_torso_angles = cur_torso.desired.positions
    TorsoJTAS(current_torso_angles,-0.3,0.05,0.0)

    # Head motion.
    '''
    Arguments(current_angles, l_r_angle, u_d_angle, ang_vel, wait_time): 
        current_angles: current head angles.
        l_r_angle: left and right (+:cw, -:ccw). 0.78 = 45 degrees, 1.57 = 90 degrees.
        u_d_angle: up and down. 0.78 = 45 degrees, 1.57 = 90 degrees.
        ang_vel: The recommended value is 0.3.
        wait_time: The recommended value is 3.0.
    '''
    # cur_head = rospy.wait_for_message("/movo/head_controller/state", JointTrajectoryControllerState)
    # current_head_angles = cur_head.desired.positions
    # HeadJTAS(current_head_angles,0,-0.78,0.3,3.0)

if __name__ == "__main__":
    main()