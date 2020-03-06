

# Table of Contents
1. [Installation guide](#install)
	1. [movo_v1](#install-movo)
	2. [kinectic-devel](#install-kinetic)
2. [Perception](#percept)
	1. [Point cloud](#percept-point)
	2. [A little survey](#percept-survey)
2. [Navigation](#nav)
	1. [Mapping and localization](#nav-mapping)
	2. [SLAM](#nav-slam)
3. [Manipulation](#mani)
	1. [Grasping](#mani-grasp)
	2. [Useful references](#mani-ref)
4. [Demo in Sim](#sim_demo)
	1. [Pick and place](#sim_demo-pick)
5. [Demo in Real-World](#real_demo)


# <a name="install"></a>Installation Guide
## <a name="install-movo"></a>movo_v1
- MOVO repository for the Kinova mobile manipulator. Remote PC and sim does not need movo_network or movo_robot.
- Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions
- Note: voice control requires installation of pocketsphinx. e.g.: `sudo apt-get install ros-kinetic-pocketsphinx`.
    Voice navigation requires installation of SpeechRecognition. e.g.: `pip install SpeechRecognition`.
    
## <a name="install-kinetic">kinectic-devel
### How to install
- Follow the steps located in `movo_common/si_utils/src/si_utils/setup_movo_pc_migration`. Start from Install third parties and additional libraries. But do it line by line manually instead of running the `setup_movo_pc_migration` script.
- In the above steps, make sure you use gcc-5. When doing `cmake`, do `env CXX=g++-5 cmake` instead.
- Somehow making `AssImp` gives the gtest-related error and I wasn't able to solve it yet. However, this Package was compiled successfully and not having `AssImp` seems okay for now. 
- For libfreenect2, follow the instruction given by Kinova: <https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions>.

### Troubleshooting
- If kinect does not work in Gazebo, make sure to set the Gazebo reference to `${prefix}_ir_frame` from the `kinect_one_sensor.urdf.xacro` file located in `/movo_common/movo_description/urdf/sensors/`.

### Useful references
- Paper describing the MOVO software, hardware and architecture: [Snoswell et al.](https://espace.library.uq.edu.au/data/UQ_4f8a4e0/pap126s1-file1.pdf?Expires=1580165124&Key-Pair-Id=APKAJKNBJ4MJBJNC6NLQ&Signature=F~tbFYRDenScPux-868miX4d86-ud~Tgsp8vQ5aK4VyIHCr0ANWdKrqM1Z2eWgOEhvRSejZnI4wZLco2s00XdEWIQ7-P3lSsq0t50LMjiq1O5Ncw9tSGD0eEKERwqMHx1wWbHpMH52E1GblQk4OwzkrewM~cQN7O2sCOO6ifVGOxgKQb9ratZU97sHZYAUA09y30tDVfY4xmwX~VIxsG~JWLvDLKGLr5WFY2FKsmbLiLlfciAX2oHJRVe-768xOMX5KbYYz0bB-Ucraiq6Uv9tizUIw-zLaB6U7iyv0dBIYCKQTmupdFf5rnNrEmLb191JKjZ0RrHRxZ71mxr1Y18w__).


# <a name="percept"></a>Perception
## <a name="percept-point"></a>Point cloud
- Coversion between the depth image and the point cloud: [depth_image_proc](http://wiki.ros.org/depth_image_proc).

## <a name="percept-survey"></a>A little survey
- Bandwidth usage per message
	- `/movo_camera/point_cloud/points`: >300MB/MSG.
	- `/movo_camera/sd/image_depth`: \~8MB/MSG. The compressed depth image is \~4MB/MSG.
	- `/movo_camera/hd/image_depth_rect/compressed`: \~19MB/MSG.
	- `/movo_camera/qhd/image_depth_rect/compressed`: \~6.1MB/MSG. The compressed color image is 1.8MB/MSG.


# <a name="nav"></a>Navigation
## <a name="nav-mapping"></a>Mapping and localization
- Refer to How Tos given by Kinova: [for real robot](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-real-robot) and [for simulation](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-virtual-robot).
- Most relevant parameters are called in `move_base.launch` located in `/movo_demos/launch/nav/`. `eband_planner_params.yaml` contains local planner-related parameters.

## <a name="nav-slam"></a>SLAM
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros).
- Installation guide: [here](https://github.com/introlab/rtabmap_ros/tree/kinetic-devel).
- How to run:
	- In a terminal, do `roslaunch movo_demos sim_rtabmap_slam.launch`.
	- In another terminal, do `roslaunch movo_demos rtabmap_slam.launch rtabmap_args:="--delete_db_on_start"`. Use `rtabmap_args:="--delete_db_on_start"` if you want to start over the map. Otherwise, take this out.
- Useful arguments (attach after calling the launch file):
	- `rtabmap_args:="--delete_db_on_start"`: this deletes the database saved in `~/.ros/rtabmap.db` at each start.

### Troubleshooting
- If you face the error when catkin making: `make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libfreenect.so', needed by '/home/yoon/movo_ws/devel/lib/rtabmap_ros/pointcloud_to_depthimage'.  Stop.`, do `sudo apt-get install libfreenect-dev`.


# <a name="mani"></a>Manipulation
## <a name="mani-grasp"></a>Grasping
### simple_grasping
- Currently, this feature is not available. We only use vanilla MoveIt for now.
- The grasping largely consists of three packages as follows.
- [simple_grasping](https://github.com/mikeferguson/simple_grasping).
- [moveit_python](https://github.com/mikeferguson/moveit_python).
- [grasping_msgs](https://github.com/mikeferguson/grasping_msgs).
- Refer to the Gazebo tutorial provided by Fetch Robotics: [here](http://docs.fetchrobotics.com/gazebo.html).
- Grasping poses are hardcoded in `createGraspSeries()` and `createGrasp()` in `shape_grasp_planner.cpp`.

### <a name="mani-ref"></a>Useful references
- Actionlib-detailed description: [here](http://wiki.ros.org/actionlib/DetailedDescription).


# <a name="sim_demo"></a>Demo in Sim
Demo-related files are located in `/movo_demos`.
## <a name="sim_demo-pick"></a>Pick and place
MoveIt-based demo. As a simulator only rviz is used, not Gazebo. Do the following to run the demo.
<img style="float: right;" src="https://github.com/syc7446/kinova-movo/blob/kinetic-devel/docs/Images/pick_place_demo.png" width="500" height="600">
1.  `roslaunch movo_7dof_moveit_config demo.launch`.
2. `rosrun movo_demos sim_moveit_pick_place.py`.


# <a name="real_demo"></a>Demo in Real-World
