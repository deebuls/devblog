---
toc: True
layout: post
description: Migrating Kelo Tulip to ROS2
categories: [robotics, ros2]
image: https://www.kelo-robotics.com/wp-content/uploads/2021/04/KELO_Drives_2_cropped.jpg
title: Migrating Kelo Tulip to ROS2
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

   
