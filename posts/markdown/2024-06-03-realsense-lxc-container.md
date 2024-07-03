---
categories:
- robotics
date: '2024-06-03'
description: RealSense camera in LXC container 
image: https://upload.wikimedia.org/wikipedia/commons/3/3d/Intel_Realsense_depth_camera_D435.jpg
layout: post
title: Connecting Realsense camera in LXC container
toc: true

---
# Setting Up RealSense in LXC 

## Introduction
This guide explains how to set up an additional RealSense  camera in an LXC container running ubuntu 20.04 Focal. 
The container uses Ubuntu 22.04 for newer software compatibility.

## Steps

### 1. Prepare the Container
1. **Install LXC**:
   ```bash
   sudo apt-get install lxc
   ```
2. **Launch the Container**:
   ```bash
   lxc launch ubuntu:22.04 my-container
   ```
3. **Install Required Software in the Container**:
   ```bash
   sudo apt-get install librealsense2 ros-foxy-desktop realsense2-camera
   ```

### 2. Device Mapping
1. **Identify Device Information**:
   ```bash
   lsusb
   ```
   Note the `vendorid` and `productid`.

2. **Map the USB Device**:
   ```bash
   lxc config device add my-container realsense usb vendorid=8086 productid=0b07 gid=1000
   ```

### 3. Video Devices Mapping
1. **Map `/dev/videoX` Devices**:
   ```bash
   ls -la /dev/video*
   lxc config device add my-container video0 unix-char path=/dev/video0 gid=1000
   lxc config device add my-container video1 unix-char path=/dev/video1 gid=1000
   ```

### 4. Restart and Test
1. **Restart the Container**:
   ```bash
   lxc restart my-container
   ```
2. **Test the Setup**:
   ```bash
   source /opt/ros/foxy/setup.bash
   ros2 launch realsense2_camera rs_launch.py
   ```

## Conclusion
This setup allows the use of the RealSense D435i camera within an LXC container on a Jetson device, leveraging the host's kernel and maintaining compatibility with other RealSense cameras.

For more details, refer to the [original GitHub issue](https://github.com/IntelRealSense/librealsense/issues/11151).

# Notes
Title Image - Marc Auledas, CC BY-SA 4.0 <https://creativecommons.org/licenses/by-sa/4.0>, via Wikimedia Commons
