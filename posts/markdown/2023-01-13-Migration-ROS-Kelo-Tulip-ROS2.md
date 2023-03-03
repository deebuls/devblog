---
aliases:
- /robotics/ros2/2023/01/13/Migration-ROS-Kelo-Tulip-ROS2
categories:
- robotics
- ros2
date: '2023-01-13'
description: Migrating Kelo Tulip to ROS2
image: https://www.kelo-robotics.com/wp-content/uploads/2021/04/KELO_Drives_2_cropped.jpg
layout: post
title: Migrating Kelo Tulip to ROS2
toc: true

---

# Migration of ROS2

1. As per the blog the migration steps are (ROS2 migration guide)[https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html]
    * Package manifests
    * Metapackages
    * Message, service, and action definitions
    * Build system
    * Update source code
3. Step 1 : Package manifests

4. Migration Gazebo
   * https://github.com/ros-simulation/gazebo_ros_pkgs/wiki

5. Messages 
   * For messages and services replace the message using the formulation below
   * ``` #include <sensor_msgs/JointState.hpp> ``` ➡️ ``` #include <sensor_msgs/msg/joint_state.hpp> ```
   * ``` #include <nav_msgs/Odometry.hpp> ``` ➡️ ``` #include <nav_msgs/msg/odometry.hpp> ```
   * You add the msg and in small letters

6. Main.cpp
   * ``` #include "ros/ros.h"  ``` ▶️ ``` #include "rclcpp/rclcpp.hpp" ```

7. Launching robot in Gazebo : Error model visalization 
```
For ROS2 usage, by changing the following in any of the cameras' urdf file
<mesh filename="package://realsense2_description/meshes/d415.stl" />
into
<mesh filename="file://$(find realsense2_description)/meshes/d415.stl" />
will resolve the issue, as it will evaluate to the full path when xacro generates the URDF.
```

8. Adding laser scanner in gazebo . Errors
    * Used the xacro method from ros1 
    * xacro method works but plugins have changed 
    * libgazebo_ros_laser.so is now changed to ➡️ libgazebo_ros_ray_sensor.so
    * Also the paraeter to initialize it is different 
    * Check the github [issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1266#issuecomment-826249858) for proper initialization
    * 
9. Ros2 node with both cpp and python then do the following 
 
    * I have a mixed C++/Python package. To install a Python executable in this package, I've found it sufficient to do this:
    * Add the Python source file to my_package/scripts
    * Add this line at the top of ^that file: #!/usr/bin/env python3
    * Make it executable with chmod +x
    * Add an empty __init__.py file in my_package/scripts
    * Add this to CMakeLists:
```
install(PROGRAMS  
  scripts/my_python_file.py  
  DESTINATION lib/${PROJECT_NAME} )
  
```
    * from https://answers.ros.org/question/299269/ros2-run-python-executable-not-found/

## Errors 
1. Starting gazebo in the launch file gives "camera assertion error "
    * The solution is to source Gazebo's setup file, i.e.: ```. /usr/share/gazebo/setup.sh```
    * https://answers.gazebosim.org//question/28066/is-libgazebo_ros_multicameraso-deprecated/
    * ``` echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc ``` ▶️ for persistence 
