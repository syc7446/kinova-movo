#!/usr/bin/env python

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class WorldModel(object):
    def __init__(self, world_x_res, world_y_res, world_z_res, horizontal_cell_size, vertical_cell_size, odd_number):
        self._grid_x = []
        self._grid_y = []
        self._grid_z = []
        self._world_x_res = world_x_res
        self._world_y_res = world_y_res
        self._world_z_res = world_z_res
        self._horizontal_cell_size = horizontal_cell_size
        self._vertical_cell_size = vertical_cell_size
        self._odd_number = odd_number
        self._init_robot_pose_sub = rospy.Subscriber('/movo/odometry/local_filtered', Odometry, self.init_robot_pose_cb)
        self._init_robot_pose_check = False

    def init_robot_pose_cb(self, data):
        if not self._init_robot_pose_check:
            thx, rthy, rthz = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
            init_robot_pose = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, rthz)
            print "Initial robot position: ", init_robot_pose

            # Origin of the world.
            if self._odd_number:
                x_st = init_robot_pose[0]-(math.floor(self._world_x_res/2)*self._horizontal_cell_size)
            else:
                x_st = init_robot_pose[0]-((self._world_x_res/2-1)*self._horizontal_cell_size)
            y_st = init_robot_pose[1]
            z_st = 0

            # Positions for all grid cells.
            for i in range(0, self._world_x_res):
                self._grid_x.append(x_st+self._horizontal_cell_size*i)
            for i in range(0, self._world_y_res):
                self._grid_y.append(y_st+self._horizontal_cell_size*i)
            for i in range(0, self._world_z_res):
                self._grid_z.append(z_st+self._vertical_cell_size*i)

            self._init_robot_pose_check = True

    def get_grid(self):
        return self._grid_x, self._grid_y, self._grid_z

    def check_init_robot_pose(self):
        return self._init_robot_pose_check