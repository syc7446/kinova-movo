
# Table of Contents
1. [Installation guide](#install)
2. [Perception](#percept)
2. [Navigation](#nav)
3. [Manipulation](#mani)

# <a name="install"></a>Installation Guide
## movo_v1
- MOVO repository for the Kinova mobile manipulator. Remote PC and sim does not need movo_network or movo_robot.
- Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions
- Note: voice control requires installation of pocketsphinx. eg: sudo apt-get install ros-kinetic-pocketsphinx
    voice navigation requires installation of SpeechRecognition. eg: pip install SpeechRecognition
    
## kinect-devel
### How to install
- Follow the steps located in `movo_common/si_utils/src/si_utils/setup_movo_pc_migration`. Start from Install third parties and additionnal libraries. But do it line by line manually.
- In the above steps, make sure you use gcc-5. When doing `cmake`, do `env CXX=g++-5 cmake` instead.
- For libfreenect2, follow the instruction given by Kinova: <https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions>.

### Troubleshooting
- If kinect does not work in Gazebo, make sure to set the Gazebo reference to `${prefix}_ir_frame` from the `kinect_one_sensor.urdf.xacro` file located in `/movo_common/movo_description/urdf/sensors/`.

### Useful references
- Paper describing the MOVO software, hardware and architecture: [Snoswell et al.](https://espace.library.uq.edu.au/data/UQ_4f8a4e0/pap126s1-file1.pdf?Expires=1580165124&Key-Pair-Id=APKAJKNBJ4MJBJNC6NLQ&Signature=F~tbFYRDenScPux-868miX4d86-ud~Tgsp8vQ5aK4VyIHCr0ANWdKrqM1Z2eWgOEhvRSejZnI4wZLco2s00XdEWIQ7-P3lSsq0t50LMjiq1O5Ncw9tSGD0eEKERwqMHx1wWbHpMH52E1GblQk4OwzkrewM~cQN7O2sCOO6ifVGOxgKQb9ratZU97sHZYAUA09y30tDVfY4xmwX~VIxsG~JWLvDLKGLr5WFY2FKsmbLiLlfciAX2oHJRVe-768xOMX5KbYYz0bB-Ucraiq6Uv9tizUIw-zLaB6U7iyv0dBIYCKQTmupdFf5rnNrEmLb191JKjZ0RrHRxZ71mxr1Y18w__).


# <a name="percept"></a>Perception
## Point cloud
- Coversion between the depth image and the point cloud: [depth_image_proc](http://wiki.ros.org/depth_image_proc).

## A little survey
- Bandwidth usage per message
	- `/movo_camera/point_cloud/points`: >300MB/MSG.
	- `/movo_camera/sd/image_depth`: \~8MB/MSG. The compressed depth image is \~4MB/MSG.
	- `/movo_camera/hd/image_depth_rect/compressed`: \~19MB/MSG.
	- `/movo_camera/qhd/image_depth_rect/compressed`: \~6.1MB/MSG. The compressed color image is 1.8MB/MSG.


# <a name="nav"></a>Navigation
## Mapping and localization
- Refer to How Tos given by Kinova: [for real robot](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-real-robot) and [for simulation](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-virtual-robot).
- Most relevant parameters are called in `move_base.launch` located in `/movo_demos/launch/nav/`. `eband_planner_params.yaml` contains local planner-related parameters.

## SLAM
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros).
- Installation guide: [here](https://github.com/introlab/rtabmap_ros/tree/kinetic-devel).
- How to run:
	- In a terminal, do `roslaunch movo_demos sim_rtabmap_slam.launch`.
	- In another terminal, do `roslaunch movo_demos rtabmap_slam.launch rtabmap_args:="--delete_db_on_start"`.
- Useful arguments (attach after calling the launch file):
	- `rtabmap_args:="--delete_db_on_start"`: this deletes the database saved in `~/.ros/rtabmap.db` at each start.


# <a name="mani"></a>Manipulation
## Grasping
- [moveit_grasps](https://github.com/ros-planning/moveit_grasps/tree/kinetic-devel).

## Useful references
- Actionlib-detailed description: [here](http://wiki.ros.org/actionlib/DetailedDescription).