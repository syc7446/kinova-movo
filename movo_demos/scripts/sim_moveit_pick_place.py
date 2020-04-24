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
        left_group_name = "left_arm"
        left_ref_link = "left_base_link"
        left_gripper_group_name = "left_gripper"
        target_object = "object"

        right_arm = MoveGroupInterface(right_group_name, right_gripper_group_name)
        left_arm = MoveGroupInterface(left_group_name, left_gripper_group_name)

        target_joint_goal = [1.1260537695066337, 1.4939862738417884, -0.07007152850941699, 1.609509339071485, 
            -0.5450575788391218, 0.13493387482783845, -0.5797459527790872]
        left_arm.go_to_joint_state(target_joint_goal)

        target_joint_goal = [-3.1414979026815826, 0.509499570405352, 1.3683901943334078, 1.7349954659724591, 
            0.49864985142141166, 1.616744121511612, 1.8265596732989007]
        right_arm.go_to_joint_state(target_joint_goal)

        obj_size = (0.2, 0.4, 0.4)
        obj_pose = geometry_msgs.msg.PoseStamped()
        obj_pose.header.frame_id = base_link
        obj_pose.pose.position.x = 1.073
        obj_pose.pose.position.y = -0.129
        obj_pose.pose.position.z = 0.2
        obj_pose.pose.orientation.w = 1.0
        obj_name = "table_1"
        right_arm.add_obj(obj_size, obj_pose, obj_name)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        obj_size = (0.4, 0.2, 0.4)
        obj_pose.header.frame_id = base_link
        obj_pose.pose.position.x = 0.5
        obj_pose.pose.position.y = -0.5
        obj_pose.pose.position.z = 0.2
        obj_pose.pose.orientation.w = 1.0
        obj_name = "table_2"
        right_arm.add_obj(obj_size, obj_pose, obj_name)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        obj_size = (0.02, 0.02, 0.20)
        obj_pose.header.frame_id = base_link
        obj_pose.pose.position.x = 1.103
        obj_pose.pose.position.y = -0.129
        obj_pose.pose.position.z = 0.50
        obj_pose.pose.orientation.w = 1.0
        obj_name = target_object
        right_arm.add_obj(obj_size, obj_pose, obj_name)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = right_ref_link
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
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        # Pre-grasping
        waypoint = geometry_msgs.msg.Point()
        waypoint.x = 0.1
        waypoint.y = 0.0
        waypoint.z = 0.0
        cartesian_plan, fraction = right_arm.plan_cartesian_path(waypoint)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        right_arm.display_trajectory(cartesian_plan)

        right_arm.attach_obj("object")
        right_arm.gripper_action("close")
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        # Post-grasping
        waypoint.x = 0.0
        waypoint.y = 0.0
        waypoint.z = 0.1
        cartesian_plan, fraction = right_arm.plan_cartesian_path(waypoint)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        pose_goal.header.frame_id = right_ref_link
        pose_goal.pose.position.x = 0.371
        pose_goal.pose.position.y = 0.148
        pose_goal.pose.position.z = 0.432
        pose_goal.pose.orientation.x = 0.513
        pose_goal.pose.orientation.y = -0.502
        pose_goal.pose.orientation.z = 0.490
        pose_goal.pose.orientation.w = -0.495
        planning_time = 60
        goal_position_tolerance = 0.01
        goal_orientation_tolerance = 0.01
        right_arm.go_to_pose_goal(pose_goal, planning_time, goal_position_tolerance, goal_orientation_tolerance)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        # Pre-grasping
        waypoint.x = 0.0
        waypoint.y = 0.0
        waypoint.z = -0.1
        cartesian_plan, fraction = right_arm.plan_cartesian_path(waypoint)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        right_arm.gripper_action("open")
        right_arm.detach_obj(target_object)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        # Post-grasping
        waypoint.x = 0.0
        waypoint.y = 0.0
        waypoint.z = 0.1
        cartesian_plan, fraction = right_arm.plan_cartesian_path(waypoint)
        if bool_raw_input:
            print "============ Press `Enter` to execute ..."
            raw_input()

        right_arm.go_to_joint_state(target_joint_goal)

        print "============ Python pick_place demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
