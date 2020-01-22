#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from movo_msgs.msg import ConfigCmd
from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class WaypointApply(object):
    def __init__(self, position, orientation):
        # Get an action client
        self.client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
        rospy.loginfo("Waiting for movo_move_base AS...")
        if not self.client.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to movo_move_base AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)

        # Set robot mode to active base motion
        self._cfg_cmd = ConfigCmd()
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)

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
        self.client.wait_for_result()
        result = self.client.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                rospy.logerr("Failed to pick, not trying further")
                return
        rospy.loginfo("111")
        # self.motion_stop()
        # return True

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