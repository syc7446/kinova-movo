# Contents
1. Installation guide: [goto](#install)
2. Navigation: [goto](#nav)
3. Manipulation [goto](#mani)

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

# <a name="nav"></a>Navigation
## References
- Refer to How Tos given by Kinova: [for real robot](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-real-robot) and [for simulation](https://github.com/Kinovarobotics/kinova-movo/wiki/2.-How-Tos#creating-a-map-with-virtual-robot).
- Most relevant parameters are called in `move_base.launch` located in `/movo_demos/launch/nav/`. `eband_planner_params.yaml` contains local planner-related parameters.

# <a name="mani"></a>Manipulation
## References
- Moveit python: [here](https://github.com/mikeferguson/moveit_python)
- Simple grasping: [here](https://github.com/mikeferguson/simple_grasping)
- Actionlib-detailed description: [here](http://wiki.ros.org/actionlib/DetailedDescription)