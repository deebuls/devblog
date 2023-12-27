---
aliases:
- /robotics/2022/10/20/ROS2-LXD-Container
categories:
- robotics
date: '2022-10-20'
description: Running ros2 humble in ubuntu lxd container
image: https://docs.ros.org/en/humble/_static/humble-small.png
layout: post
title: Running ros2 humble in ubuntu lxd container
toc: true

---

# ROS2 in Lxd container 

Its always dificult to get the appropriate ubuntu version for a running a particular software. For example, currently I have ubuntu 20.04 and I want to test a software in ros2 humble. Humble requires ubuntu version 22.04.
I wanted to try if we can use lxd containers for running the appropriate ubuntu version and installing ros in those. 

## LXD start easy steps

1. Install lxd in your ubuntu (Using snap)
2. Use Lxc to  make sure its  without sudo 
3. For using gui inside the container we need to attach profiles to the container.
4. Create a profile using the following command 
```
$ lxc profile create x11
Profile x11 created
```
5. Edit the profile with the following command
```
$ lxc profile edit x11
```
6. Copy paste below in the editor below

```yaml
config:
  environment.DISPLAY: :0
  environment.PULSE_SERVER: unix:/home/ubuntu/pulse-native
  nvidia.driver.capabilities: all
  nvidia.runtime: "true"
  user.user-data: |
    #cloud-config
    runcmd:
      - 'sed -i "s/; enable-shm = yes/enable-shm = no/g" /etc/pulse/client.conf'
    packages:
      - x11-apps
      - mesa-utils
      - pulseaudio
description: GUI LXD profile
devices:
  PASocket1:
    bind: container
    connect: unix:/run/user/1000/pulse/native
    listen: unix:/home/ubuntu/pulse-native
    security.gid: "1000"
    security.uid: "1000"
    uid: "1000"
    gid: "1000"
    mode: "0777"
    type: proxy
  X0:
    bind: container
    connect: unix:@/tmp/.X11-unix/X1
    listen: unix:@/tmp/.X11-unix/X0
    security.gid: "1000"
    security.uid: "1000"
    type: proxy
  mygpu:
    type: gpu
name: x11
used_by: []
```

7. In the above file check the line ```connect: unix:@/tmp/.X11-unix/X1``` This depends on how your local machine DISPLAY is set. If your local machine DISPLAY is at X1 replace there with X0.
8. To check your local machine display```echo $DISPLAY ```.
9. Now lets create a container with the profile
```bash
lxc launch ubuntu:22.04 --profile default --profile x11 mycontainer
```
9. To get a shell in the container, run the following.
```
$ lxc exec mycontainer -- sudo --user ubuntu --login
mycontainer $ sudo apt install xclock 
mycontainer $ export DISPLAY=:0  #Add this in bashrc
mycontainer $ xclock
```
10. The above command will open the xclock in GUI . If the GUI doesnt come then check your local DISPLAY and the DISPLAY inside the container. 
11. Complete blog with explanation on the process is provided by (Simos)[https://blog.simos.info/running-x11-software-in-lxd-containers/]


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


## Additional: Noetic in LXC
1. Steps in the host computer to dowload ubuntu 20  and login to the virtual machine

```
$ lxc launch images:ubuntu/20.04 noetic
Creating noetic
Starting noetic  
$ lxc list
+------------+---------+----------------------+----------------------------------------------+-----------+-----------+
|    NAME    |  STATE  |         IPV4         |                     IPV6                     |   TYPE    | SNAPSHOTS |
+------------+---------+----------------------+----------------------------------------------+-----------+-----------+
| noetic     | RUNNING | 10.171.226.67 (eth0) | fd42:13f7:78ed:7795:216:3eff:fe25:30f (eth0) | CONTAINER | 0         |
+------------+---------+----------------------+----------------------------------------------+-----------+-----------+
$ lxc exec noetic -- su --login ubuntu
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.

ubuntu@noetic:~$ 

```

2. For the GUI you have to stop and attach profile read above about creating a profile names x11
```
ubuntu@noetic:~$  exit
$ lxc stop noetic
$ lxc profile assign noetic default,x11 #or whatever name is given to profile. Check above on how to create profile
$ lxc start noetic
ubuntu@noetic:~$ sudo apt install x11-apps
ubuntu@noetic:~$ xclock #should display the clock if not then check the DISPLAY value in both host and container
```
3. Steps in the the container for installing ros and graphics
```bash
// Follow the ros noetic page 
// Additional before adding keys install gpg 

sudo apt install gpg

```

You have a working ubuntu noetic container with ros noetic.

## Adding a camera to the container 

Information from here [1](https://askubuntu.com/questions/760473/how-to-use-a-video-device-in-a-lxd-container)
```
lxc config device add my-container video0 unix-char path=/dev/video0
```
restart the container and check



The group and permissions for your /dev/video0 are not correct. The groop root for your /dev/video0 will deny access to the camera for users outside this group.

The output of ```ls -l /dev/video0``` should look like this:
```
crw-rw----+ 1 root video 81, 1 Apr 19 22:25 /dev/video0
```
Try fixing the group by running:
```
sudo chown root:video /dev/video0
```
Then fix permissions by running:
```
sudo chmod 660 /dev/video0
```

if the group video is missing try fixing it by these commands and running again the above commands 
```
sudo adduser www-data video
sudo usermod -a -G video www-data
```



You might need to give permissions
