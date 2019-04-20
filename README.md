# eDO Control package

This package take control over eDO robots.

It provides a simple ROS interface in order to execute trajectories with position PID control through the `joint_trajectory_action_server` and publishes the current `/joint_states`.

## Advised ROS network configuration
Although other configurations are possible, this package intends to run on a deported workstation connected through Ethernet to a 6-axis e.Do on which an electric gripper is mounted.
Other configurations (Wifi, no gripper, run on the internal Pi...) might need to tweak a bunch of parameters, as they have not been tested. We do not advise Wifi connection: since joint commands are sent over the network, it might result is jerky trajectories because of Wifi lags.

### 1. Local workstation network
Before anything, configure your workstation's IP address so that's in the 10.42.0.XXX network. Here we assume `10.42.0.1` so **make sure** you replace all `10.42.0.1` by you own IP if you choose another one.

### 2. EDo's raspi configuration over Ethernet
Allow your robot to communicate with your local workstation through Ethernet with this configuration step.
Connect to the EDo's internal Raspberry Pi via SSH:
```
ssh edo@10.42.0.49
```
Default password is `raspberry`. You should see an italian-speaking `comandi tmux` prompt with explanations about rostopic.
If you don't, you have trouble in networking configuraiton, or the internal Raspberry Pi does not run.

We need to edit the `ministarter` script:
```
nano ~/ministarter
``` 
Locate these 2 ROS configuration lines:
```
export ROS_MASTER_URI=http://192.168.12.1:11311
export ROS_IP=192.168.12.1
```
and replace them by the IP address of the robot's Ethernet interface:
```
export ROS_MASTER_URI=http://10.42.0.49:11311
export ROS_IP=10.42.0.49
```
Press `ctrl=X`, `y` and then `Enter` to valide the filename and overwrite the file.

Type `sudo reboot` to reboot the robot and wait for it to be up again.

### 3. Local workstation ROS config
setup your ROS environment variables on your local workstation:
```
export ROS_MASTER_URI=http://10.42.0.49:11311
export ROS_IP=10.42.0.1       # Set your workstation's IP instead!
```

Check that everything is fine: If this command returns the message below, you're done for the ROS network configuration!
```
$ rostopic echo /machine_state -n1
current_state: 0
opcode: 0
```
## Now I want to start the ROS control interface
### 1. Calibration procedure is compulsory after any robot boot
```
 roslaunch edo_control calibrate.launch
```
You should first see a `JOINT_UNCALIBRATED` warning message and then the explanations for the calibration procedure.
Follow the instructions every time you see `Calibrating joint X`, press left and right arrow keys to align the joint center marker and press Enter to switch to the next joint.

Go on wth the entire calibration before continuing.

### 2. Bring up the action server and the joint state publisher
```
 roslaunch edo_control control.launch
```

This will bring up the following standard ROS interfaces:

* Topic `/joint_states`      # Echoes joint states at about 90 Hz
* Action server `/follow_joint_trajectory` (e.g. to be used with `eDO_moveit` package)

Optionally, this interactive script offers more features for calibration, manual jogs, motions in cartesian space or returning to home position:

```
rosrun edo_control setup_edo.py
```
