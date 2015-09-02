About
-----

This repository provides ROS support for Choreonoid

choreonoid\_ros\_pkg: Chorenoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic


Usage
-----

Create catkin workspace

```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ wstool init
```

Checkout choreonoid\_ros\_pkg

```
$ git clone https://github.com/fkanehiro/choreonoid_ros_pkg.git
```

Build

```
$ cd ~/catkin_ws
$ catkin_make install
```

Run

```
$ roscore (on the different terminal)
$ ./devel/bin/choreonoid
```

Configure AISTSimulator item to use High-gain dynamics mode.
Create and place BodyRos item under the robot you want to control.

Each joint states are published to /[robotname]/joint\_states topic.
Your control signal can be sent using /[robotname]/set\_joint\_trajectory topic.


Note
-----

Launchscript and other useful tools will be provided soon.
