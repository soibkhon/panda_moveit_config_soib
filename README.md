# panda_moveit_config_soib
Panda moveit config created by Moveit! setup assistant, with written controllers and a sample codes

**Ubuntu 20.04**

**ROS Noetic**

**Franka FCI 4.2.1, libfranka 0.8.0**

One sample code is taken from the moveit tutorial package, other one is custom, for making circular movements

**How to test on real robot:**

```
roslaunch panda_moveit_config franka_control.launch robot_ip:=<franka_ip>
```

```
rosrun panda_moveit_config example_control.py
```
