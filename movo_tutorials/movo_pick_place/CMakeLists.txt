cmake_minimum_required(VERSION 2.8.3)
project(movo_pick_place)

# C++ 11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  aruco_ros
  geometry_msgs
  moveit_msgs
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  rospy
  std_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/SphericalGrasp.cfg
  )

add_action_files(
  FILES
  PickUpPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface

  DEPENDS
    EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

install(PROGRAMS
        scripts/pick_and_place_server.py
        scripts/pick_client.py
        scripts/spherical_grasps_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
