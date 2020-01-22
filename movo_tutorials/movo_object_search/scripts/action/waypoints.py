#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class WaypointApply(object):
    def __init__(self, position, orientation):
        self.goal_cnt = 0
        self._base_vel_pub = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)
        # Get an action client
        self.client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
        rospy.loginfo("Waiting for movo_move_base AS...")
        if not self.client.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to movo_move_base AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)

        # Define the goal
        rospy.loginfo("Waypoint (%.2f,%.2f) and (%.2f,%.2f,%.2f,%.2f) is sent.", position[0], position[1], orientation[0], \
            orientation[1], orientation[2], orientation[3])
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        self.waypoint_execute()

    def waypoint_execute(self):
        self.client.send_goal(self.goal)
        if self.client.wait_for_result():
            rospy.loginfo("Goal is reached at (%.2f, %.2f).", self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y)