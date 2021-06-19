#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman, Yoonchang Sung

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_utils.sim_planning import MoveGroupInterface

def main():
    bool_raw_input = False

    try:
        base_link = "base_link"
        right_group_name = "right_arm"
        right_ref_link = "right_base_link"
        right_gripper_group_name = "right_gripper"

        right_arm = MoveGroupInterface(right_group_name, right_gripper_group_name)

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = base_link
        pose_goal.pose.position.x = 0.003
        pose_goal.pose.position.y = 0.243
        pose_goal.pose.position.z = 0.895
        pose_goal.pose.orientation.x = -0.000
        pose_goal.pose.orientation.y = 0.000
        pose_goal.pose.orientation.z = -0.705
        pose_goal.pose.orientation.w = 0.710
        planning_time = 60
        goal_position_tolerance = 0.01
        goal_orientation_tolerance = 0.01
        right_arm.go_to_pose_goal(pose_goal, planning_time, goal_position_tolerance, goal_orientation_tolerance)

        print "============ Python manipulation reaching a goal pose test complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
