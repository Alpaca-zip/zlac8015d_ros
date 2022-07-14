# ROS package for ZLAC8015D dual-channel servo driver  
[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/zlac8015d_ros)

<img src="https://github.com/Alpaca-zip/zlac8015d_ros/blob/main/circuit_scheme.png" width="850px">

## 1 Installation
```
$ cd catkin_ws/src 

$ git clone https://github.com/Alpaca-zip/zlac8015d_ros.git

$ cd .. && catkin_make
```

## 2 Change full pathname of parameter file
`scripts/motor_driver_node.py` refers to `/home/ubuntu/catkin_ws/src/zlac8015d_ros/params/motor_driver_params.yaml` as default.  
If you want to specify a different full pathname, change line 34 in `scripts/motor_driver_node.py`

## 3 Usage 
### Run motor_driver_node
```
$ rosrun zlac8015d_ros motor_driver_node.py
```
### Parameters
- `port`: Name of the zlac8015d port. Default is `dev/ttyUSB0`.
- `travel_in_one_rev`: Tire circumference. Default is `0.655`[m].
- `R_Wheel`: Tire radius. Default is `0.105`[m].
- `cpr`: CPR(Counts Per Revolution). Default is `16385`.
- `wheels_base_width`: Distance between tires. Default is `0.440`[m].
- `control_mode`: `1` is relative position control mode, `3` is speed rpm control mode. Default is `3`.
- `callback_timeout`: Motor automatically stops if no topics are received for a certain period of time. Default is `0.5`[s].
- `decimil_coefficient`: Must be specified in digits. Smaller values will cause small changes in tire motion to have a greater impact on the odometry calculations. `0.01` or `0.001` is highly recommended. Default is `0.001`.
- `set_accel_time_left`: Acceleration time for left tire. Default is `200`[ms].
- `set_accel_time_right`: Acceleration time for right tire. Default is `200`[ms].
- `set_decel_time_left`: Deceleration time for left tire. Default is `200`[ms].
- `set_decel_time_right`: Deceleration time for right tire. Default is `200`[ms].
- `max_left_rpm`: Maximum rpm of left tire. Default is `150`.
- `max_right_rpm`: Maximum rpm of right tire. Default is `150`.
- `deadband_rpm`: Width of rpm to be regarded as 0. If `3`, then -3 to 3 is considered rpm 0. Default is `3`.
- `TF_header_frame`: Header frame of TF. Default is `odom`.
- `TF_child_frame`: Child frame of TF. Default is `base_link`.
- `odom_header_frame`: Header frame of odom. Default is `odom`.
- `odom_child_frame`: Child frame of odom. Default is `base_link`.

### Topics
This node publishes the following topics.
- `/wheels_rpm`: The speed in RPM of each tire as [left, right].
- `/odom`: Odometry data for the robot. [More detail](http://docs.ros.org/en/diamondback/api/nav_msgs/html/msg/Odometry.html)

This node subscribes to the following topics.
- `/zlac8015d/twist/cmd_vel`: Send command as linear velocity and angular velocity in speed rpm control. [More detail](https://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/Twist.html)
- `/zlac8015d/vel/cmd_vel`: Send command as velocity in speed rpm control, e.g. [0.6, 0.5] 0.6[m/s] of left tire, 0.5[m/s] of right tire.
- `/zlac8015d/vel/cmd_rpm`: Send command as rpm in speed rpm control, e.g. [100, 50] 100 rpm of left wtire, 50 rpm of right tire.
- `/zlac8015d/pos/deg_cmd`: Send command as angle degree in position control, e.g. [90,70] 90 [deg] of left tire, 70 [deg] of right tire.
- `/zlac8015d/pos/dist_cmd`: Send command as desired travelling distance in position control, e.g. [1.0, 1.0] for 1[m] travelling distance of each tire.
