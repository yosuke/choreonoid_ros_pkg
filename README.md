About
-----

This repository provides ROS support for Choreonoid

choreonoid\_ros\_pkg: Meta-package to build Choreonoid packages

choreonoid\_ros: Choreonoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic

jvrc\_models: Simulation models and tasks for JVRC

Usage
-----

Make sure you have installed wstool and catkin

```
$ sudo apt-get install python-wstool python-catkin-tools
```

Create catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

Checkout choreonoid\_ros\_pkg

```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool set choreonoid_ros_pkg https://github.com/fkanehiro/choreonoid_ros_pkg.git --git
$ wstool update choreonoid_ros_pkg
```

Build

```
$ cd ~/catkin_ws
$ catkin clean -b
$ catkin build choreonoid_ros_pkg
```

Run

```
$ roslaunch choreonoid_ros choreonoid.launch
```

Configure AISTSimulator item to use High-gain dynamics mode.
Create and place BodyRos item under the robot you want to control.

Each joint states are published to /[robotname]/joint\_states topic.
Your control signal can be sent using /[robotname]/set\_joint\_trajectory topic.


Note
-----

Launchscript and other useful tools will be provided soon.
