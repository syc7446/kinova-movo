

# Table of Contents
1. [Installation guide](#install)
	1. [movo_v1](#install-movo)
	2. [kinectic-devel](#install-kinetic)
2. [Perception](#percept)
	1. [Fiducial marker](#percept-tag)
	2. [Mask RCNN](#percept-rcnn)
	3. [Point cloud](#percept-point)
	4. [A little survey](#percept-survey)
2. [Navigation](#nav)
	1. [Mapping and localization](#nav-mapping)
	2. [SLAM](#nav-slam)
3. [Manipulation](#mani)
	1. [Grasping](#mani-grasp)
	2. [Useful references](#mani-ref)
4. [Demo in Sim](#sim_demo)
	1. [Pick and place](#sim_demo-pick)
5. [Demo in Real-World](#real_demo)
6. [Other troubleshooting tips](#trouble)


# <a name="install"></a>Installation Guide
## <a name="install-movo"></a>movo_v1
- MOVO repository for the Kinova mobile manipulator. Remote PC and sim does not need movo_network or movo_robot.
- Setup Instructions: https://github.com/Kinovarobotics/kinova-movo/wiki/Setup-Instructions
- Note: voice control requires installation of pocketsphinx. e.g.: `sudo apt-get install ros-kinetic-pocketsphinx`.
    Voice navigation requires installation of SpeechRecognition. e.g.: `pip install SpeechRecognition`.

### Troubleshooting
- Time synchronization issue between movo1 and movo2: if you get error messages regarding time synchronization, do the following: 
	- Connect via ssh to movo1.
	- In a terminal of movo1, enter : ntpdate 10.66.171.1 (the ip adresse should be the adress of movo2).
- Battery-related issue: 
	- Connect to the Ethernet port of MOVO with a remote computer. 
	- Power on the robot and quickly do the following: 
		- SSH into MOVO2.
		- `rosrun movo_ros movo_faultlog_parser`. This will produce a directory called "SI_FAULTLOGS" in the `~/.ros/` directory.
    
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
## <a name="percept-tag"></a>Fiducial marker
- We have two fiducial marker systems installed (AprilTag is preferred). 
	1. AprilTag: The `tag36h11` type is currently used and it is set in `settings.yaml` along with other AprilTag-related parameters. Set the tag ID and size you want to use in `tags.yaml`, e.g. `stanalone_tags: [{id: 0, size: 0.095}]` for the tag whose size is 9.5 by 9.5 cm square. Otherwise, tags are not going to be recognized. 

	In `continuous_detection.launch`, set `camera_name=/movo_camera/color`, `camera_frame=movo_camera_color_optical_frame`, and `image_topic=image_color_rect`. 

	[apriltag_ros GitHub repository](https://github.com/AprilRobotics/apriltag_ros). [AprilTag tutorials](http://wiki.ros.org/apriltag_ros/Tutorials).
	2. Aruco: [aruco_ros GitHub repository](https://github.com/pal-robotics/aruco_ros).

### Troubleshooting
- When catkin-making AprilTag, you may see the error of `This workspace contains non-catkin packages in it, and catkin cannot build a non-homogeneous workspace without isolation.  Try the catkin_make_isolated command instead.` due to the non-catkin `apriltag` package installed together. Since we must stick with `catkin_make`, not `catkin build`, install the `apriltag` package first as follows:
	1. `make`
	2. `PREFIX=/opt/ros/kinetic sudo make install`
	3. Then do `catkin_make` inside `movo_ws`


## <a name="percept-rcnn"></a>Mask R-CNN
- The ROS package for Mask R-CNN: [mask_rcnn_ros](https://github.com/akio/mask_rcnn_ros).

### Troubleshooting
- If you get `ImportError: libcudnn.so.6: cannot open shared object file`, then see [this issue](https://github.com/Franck-Dernoncourt/NeuroNER/issues/66#issuecomment-381317496).
- If you get `IOError: Unable to open file (Truncated file: eof = 47251456, sblock->base_addr = 0, stored_eoa = 257557808)`, then download `mask_rcnn_coco.h5` from [here](https://github.com/matterport/Mask_RCNN/releases) and place the file in `~/.ros/`.

### NVIDIA Jetson AGX Xavier 
- We use Xavier as a GPU machine to handle perception for MOVO. Xavier is on Ubuntu 18.04, ROS Melodic, and Python 3.6. To test `example.launch` provided by Mask R-CNN, follow the steps:
	1. Activate virtualenv to change to python3: `source Workspace/python-virtualenv/venv/bin/activate`
	2. Source the package of Mask R-CNN: `source Workspace/mask_rcnn_ros/devel/setup.bash`
	3. Source vision_opencv to be able to use cv_bridge: `source Workspace/catkin_build_ws/install/setup.bash --extend`


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
- Installation guide: [here](https://github.com/introlab/rtabmap_ros/tree/kinetic-devel). You can follow the `Build from source` and make sure you do the following when cloning rtabmap: `git clone -b kinetic-devel https://github.com/introlab/rtabmap.git rtabmap`.
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
<img style="float: right;" src="https://github.com/syc7446/kinova-movo/blob/kinetic-devel/docs/Images/pick_place_demo.png" width="250" height="300">
1.  `roslaunch movo_7dof_moveit_config demo.launch`.
2. `rosrun movo_demos sim_moveit_pick_place.py`.


# <a name="real_demo"></a>Demo in Real-World
TBD

# <a name="trouble"></a>Other troubleshooting tips
## Arms not working
Check if the light under the ethernet port on both arm bases is blinking. If not, one of the followings could be a reason:
1. If you can manually move the arm after powering on MOVO, this implies one or more of fuses are blown. Check the status of the fuse using the multimeter and replace with the spare fuse.
2. If the arm is stiff after powering on MOVO and cannot be moved manually, this may imply that the arm is stuck in a bootloader state. To fix this, ask Kinova to receive the Base Bootloader Upgrade service bulletin (and see Section 11) as well as the latest version of the firmware. During the bootloader upgrade, you may need to go through `short-circuit the 2 pins`, which can be highly risky. Make sure you triple check the right pints to short-circuit.

If none of the above works, ask Kinova for help.


### Fuse specs
Two types: 028707.5PXCN (7.5 A AC 32 V DC), and 0287002.PXCN (2 A AC 32 V DC).
