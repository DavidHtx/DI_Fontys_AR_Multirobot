multibot  
========

ROS Based multi robot environment for testing autonomous navigation algorithms


This is a fork of aau-ros/aau_multi_robot project (see: https://github.com/aau-ros/aau_multi_robot).

### Installation
If you are using Ubuntu-based linux, then you can simply run

```
wget -O /tmp/ros_multi_robot_ready.sh https://raw.githubusercontent.com/OSLL/multibot/master/scripts/ros_multi_robot_ready.sh; . /tmp/ros_multi_robot_ready.sh
```

Otherwise, you can follow manual installation:

1. Install ROS itself: http://wiki.ros.org/ROS/Installation
2. Initialize catkin: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Install additional packages:

1. slam-gmapping: command for ubuntu `sudo apt-get install ros-jade-slam-gmapping`
2. navigation: command for ubuntu `sudo apt-get install ros-jade-navigation`
3. teleop_twist_keyboard: clone sources to `catkin_ws/src` using `git clone https://github.com/LeoSko/teleop_twist_keyboard.git` command

These packages should be enough to run `map_merger/launch/test_one.launch`
