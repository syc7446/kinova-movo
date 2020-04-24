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
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupInterface(object):
    def __init__(self, group_name, gripper_group_name):
        super(MoveGroupInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        scene_msg = moveit_msgs.msg.PlanningScene()
        group = moveit_commander.MoveGroupCommander(group_name)
        gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # group.set_pose_reference_frame(ref_frame_name)
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.scene_msg = scene_msg
        self.group = group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_group_name = gripper_group_name
        self.co_obj = moveit_msgs.msg.CollisionObject()
        self.aco_obj = moveit_msgs.msg.AttachedCollisionObject()


    def go_to_joint_state(self, target_joint_goal):
        print "============ Go to joint state"
        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal = target_joint_goal

        group.go(joint_goal, wait=True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self, pose_goal, planning_time, goal_position_tolerance=0.01, goal_orientation_tolerance=0.01, 
        bool_path_contraints=False, constraints_frame_id=''):
        print "============ Go to pose goal"
        group = self.group
        group.set_pose_target(pose_goal)
        group.set_planning_time(planning_time)
        group.set_goal_position_tolerance(goal_position_tolerance)
        group.set_goal_orientation_tolerance(goal_orientation_tolerance)

        if bool_path_contraints:
            constraints = moveit_msgs.msg.Constraints()
            orientation_constraints = moveit_msgs.msg.OrientationConstraint()
            orientation_constraints.header.frame_id = constraints_frame_id
            orientation_constraints.link_name = group.get_end_effector_link()
            orientation_constraints.absolute_x_axis_tolerance = 0.4
            orientation_constraints.absolute_y_axis_tolerance = 0.4
            orientation_constraints.absolute_z_axis_tolerance = 0.4
            orientation_constraints.orientation = pose_goal.pose.orientation
            orientation_constraints.weight = 1.0
            constraints.orientation_constraints.append(orientation_constraints)
            group.set_path_constraints(constraints)

        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        if bool_path_contraints:
            group.clear_path_constraints()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal.pose, current_pose, goal_position_tolerance)


    def plan_cartesian_path(self, waypoint, scale=1):
        print "============ Plan cartesian path"
        group = self.group
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.x += scale * waypoint.x # Warning: currently this waypoint is with respect to the /base_link, not /right_base_link.
        wpose.position.y += scale * waypoint.y
        wpose.position.z += scale * waypoint.z
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        group.execute(plan, wait=True)
        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);


    def execute_plan(self, plan):
        group = self.group

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


    def gripper_action(self, action):
        gripper_group = self.gripper_group

        if action is "open":
            gripper_group.set_named_target("gripper_open")
        elif action is "close":
            gripper_group.set_named_target("gripper_close")
        gripper_group.go(wait=True);
        gripper_group.stop()


    def wait_for_state_update(self, obj_name, obj_is_known=False, obj_is_attached=False, timeout=4):
        scene = self.scene

        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the obj will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the obj is in attached objects
            attached_objects = scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the obj is in the scene.
            # Note that attaching the obj will remove it from known_objects
            is_known = obj_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (obj_is_attached == is_attached) and (obj_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_obj(self, obj_size, obj_pose, obj_name, timeout=4):
        print "============ Add objects"
        scene = self.scene
        scene.add_box(obj_name, obj_pose, obj_size)

        primitives = shape_msgs.msg.SolidPrimitive()
        primitives.type = shape_msgs.msg.SolidPrimitive.BOX
        primitives.dimensions = obj_size

        self.co_obj.id = obj_name
        self.co_obj.primitives.append(primitives)
        self.co_obj.operation = moveit_msgs.msg.CollisionObject.ADD
        self.co_obj.primitive_poses.append(obj_pose)
        self.scene_msg.world.collision_objects.append(self.co_obj)

        return self.wait_for_state_update(obj_name, obj_is_known=True, timeout=timeout)


    def attach_obj(self, obj_name, timeout=4):
        print "============ Attach objects"
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        gripper_group_name = self.gripper_group_name

        touch_links = robot.get_link_names(group=gripper_group_name)
        scene.attach_box(eef_link, obj_name, touch_links=touch_links)

        self.co_obj.id = obj_name
        self.co_obj.operation = moveit_msgs.msg.CollisionObject.REMOVE

        self.aco_obj.object = obj_name
        self.aco_obj.link_name = eef_link
        for i in range(0, len(touch_links)):
            self.aco_obj.touch_links.append(touch_links[i])

        # We wait for the planning scene to update.
        return self.wait_for_state_update(obj_name, obj_is_attached=True, obj_is_known=False, timeout=timeout)


    def detach_obj(self, obj_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=obj_name)

        self.co_obj.id = obj_name
        self.co_obj.operation = moveit_msgs.msg.CollisionObject.ADD

        # We wait for the planning scene to update.
        return self.wait_for_state_update(obj_name, obj_is_known=True, obj_is_attached=False, timeout=timeout)


    def remove_obj(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the obj from the world.
        scene.remove_world_object(obj_name)

        ## **Note:** The object must be detached before we can remove it from the world

        # We wait for the planning scene to update.
        return self.wait_for_state_update(obj_name, obj_is_attached=False, obj_is_known=False, timeout=timeout)