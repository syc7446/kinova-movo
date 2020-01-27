
# Contents
1. [Installation guide](#install)
2. [Perception](#percept)
2. [Navigation](#nav)
3. [Manipulation](#mani)

# <a name="install"></a>Installation guide
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


# <a name="percept"></a>Perception
## Point cloud
- Coversion between the depth image and the point cloud: [depth_image_proc](http://wiki.ros.org/depth_image_proc).


# <a name="nav"></a>Navigation
## Mapping and localization
- Refer to How Tos given by Kinova: [for real robot](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-real-robot) and [for simulation](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-virtual-robot).
- Most relevant parameters are called in `move_base.launch` located in `/movo_demos/launch/nav/`. `eband_planner_params.yaml` contains local planner-related parameters.

## SLAM
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros).
- Installation guide: [here](https://github.com/introlab/rtabmap_ros/tree/kinetic-devel).
- Useful arguments (attach after calling the launch file):
	- `rtabmap_args:="--delete_db_on_start"`: this deletes the database saved in `~/.ros/rtabmap.db` at each start.


# <a name="mani"></a>Manipulation
## Grasping
- [moveit_grasps](https://github.com/ros-planning/moveit_grasps/tree/kinetic-devel).

## Useful references
- Actionlib-detailed description: [here](http://wiki.ros.org/actionlib/DetailedDescription).