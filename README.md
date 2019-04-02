# eDO Control package

This package take control over eDO robots.

It provides a simple ROS interface in order to execute trajectories with position PID control through the `joint_trajectory_action_server` and publishes the current `/joint_states`.

## How to start?
Start 
```
 # If you're running this package on a workstation external to eDO, setup your environment variables:
export ROS_MASTER_URI=http://10.42.0.49:11311
export ROS_IP=10.42.0.1  # Set your workstation's IP instead!

 # Calibration procedure is compulsory after any robot boot  
 roslaunch edo_control calibrate.launch
 
 # Set the action server and the joint state publisher up
 roslaunch edo_control control.launch
```

## How to use?
```
rostopic echo /joint_states      # Echoes joint states at about 90 Hz
```
You can send trajectories in position control to action server `/follow_joint_trajectory` (e.g. with `eDO_moveit` package)

Optionally, this itneractive script offers more features for calibration, manual jogs, motions in cartesian space or returning to home position:

```
rosrun edo_control setup_edo.py
```