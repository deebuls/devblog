---
toc: True
layout: post
description: Running ros2 humble in ubuntu lxd container 
categories: [robotics]
image: https://docs.ros.org/en/humble/_static/humble-small.png
title: Running ros2 humble in ubuntu lxd container 
---


# ROS2 in Lxd container 

Its always dificult to get the appropriate ubuntu version for a running a particular software. For example, currently I have ubuntu 20.04 and I want to test a software in ros2 humble. Humble requires ubuntu version 22.04.
I wanted to try if we can use lxd containers for running the appropriate ubuntu version and installing ros in those. 


## LXD and ROS2 installation 
1. Started with [blog by ubuntu](https://canonical.com/blog/install-ros-2-humble-in-ubuntu-20-04-or-18-04-using-lxd-containers)
2. It gets you through the installation . 
    - I created a ubuntu 22.04 container and installed ros2 inside it . 
    - I create a user called ubuntu 
    - for ros installation I followed ros2 documentation 
3. For using ros started ros2 tutorials
4. First problem gui not working 
    - Blog by [Nick D Greg](https://nickdgreg.github.io/software/2020/08/06/running-ros-in-lxd/) introduced the topic of using profiles 
    - Following the blog I created a lxc profile named gui 
    - The blog assumes you start from begining but since we already had the container running we used the below command
    - > lxc profile assign ros-humble default,gui 
    - inside the container 'export DISPLAY=:0' in the bash (also added in bashrc)
    - now gui is running 
    - ```$> ros2 run turtlesim turtlesim_node```



## Comman Commands to get started with lxc development

1. Boot your local ubuntu and first you want to check the active containers the command is `lxc list`
```
$> lxc list
+------------+---------+----------------------+-----------------------------------------------+-----------+-----------+
|    NAME    |  STATE  |         IPV4         |                     IPV6                      |   TYPE    | SNAPSHOTS |
+------------+---------+----------------------+-----------------------------------------------+-----------+-----------+
| ros-humble | RUNNING | 10.171.226.72 (eth0) | fd42:13f7:78ed:7795:216:3eff:fe9c:bde9 (eth0) | CONTAINER | 0         |
+------------+---------+----------------------+-----------------------------------------------+-----------+-----------+
```

2. Getting bash access 
```
$ lxc exec ros-humble -- su --login ubuntu
ubuntu@ubuntu-container:~$ 
```




## Appendix 
History of comands used insde the container, after logging 
```
2  apt-cache policy | grep universe
3  sudo apt install software-properties-common
4  sudo add-apt-repository universe
5  sudo apt update && sudo apt install curl gnupg lsb-release
6  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
7  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
8  sudo apt update && sudo apt install -y   build-essential   cmake   git   python3-colcon-common-extensions   python3-flake8   python3-flake8-blind-except   python3-flake8-builtins   python3-flake8-class-newline   python3-flake8-comprehensions   python3-flake8-deprecated   python3-flake8-docstrings   python3-flake8-import-order   python3-flake8-quotes   python3-pip   python3-pytest   python3-pytest-cov   python3-pytest-repeat   python3-pytest-rerunfailures   python3-rosdep   python3-setuptools   python3-vcstool   wget
9  sudo apt update
10  sudo apt install ros-humble-desktop
```
