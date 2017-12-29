# Aurora Unior ROS package

### General description:
Package implements A* path planning algorithm and movement control for Aurora Unior car-like robot based on Kinect only RTAB-MAP SLAM or Kinect + odometry gmapping SLAM

### Dependencies:
[ROS] - Robot Operation System\
[Qt] - Qt5 C++ Framework\
[RTAB-Map] - Visual SLAM\
[gmapping] - 2D Odometry and Laser scanner based SLAM\
[depthimage_to_laserscan] - Depth image conversion to laser scanner data\
[tf] - Coordinate systems transform\
[OpenNI2_launch] - library for Asus Xtion Live Pro sensor\
[libfreenect_stack] - library for Microsoft Kinect\
[ur_hardware_driver] - Aurora Unior hardware driver\
[ros_control] - Set of packages that include controller interfaces, controller managers, transmissions and hardware_interfaces\
[velocity_controllers] - velocity controllers

### Installation:
1. Install ROS and all dependencies
2. Create ROS Workspace
3. clone this repository to your src directory in ROS workspace
4. Build package:
```sh
    $ cd ros_workspace
    $ catkin_make
```

### Usage:
1. Build map using RTAB-MAP and control robot:
```sh
    $ roslaunch aurora rtabmap_build_map.launch 
```
parameters:\
a) openni2 (default: false) - use OpenNI2 instead of libfreenect (for Asus Xtion Live Pro instead of Microsoft Kinect)\
b) ur_hardware_driver (default: true) - use ur_hardware_driver in order to control real robot

2. Build map using gmapping (doesn't work well yet):
```sh
    $ roslaunch aurora gmapping_build_map.launch
```

[ROS]: http://www.ros.org
[Qt]: https://www.qt.io
[RTAB-Map]: http://wiki.ros.org/rtabmap_ros
[gmapping]: http://wiki.ros.org/gmapping
[depthimage_to_laserscan]: http://wiki.ros.org/depthimage_to_laserscan
[tf]: http://wiki.ros.org/tf
[OpenNI2_launch]: http://wiki.ros.org/openni2_launch
[libfreenect_stack]: http://wiki.ros.org/freenect_stack
[ur_hardware_driver]: https://github.com/avrora-robotics/ur_hardware_driver
[ros_control]: http://wiki.ros.org/ros_control
[velocity_controllers]: http://wiki.ros.org/velocity_controllers
