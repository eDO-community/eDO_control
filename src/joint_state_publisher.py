#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Basic joint states publisher for real eDo robot
# The source node usually publishes at 90Hz, hence this node

import rospy
from sensor_msgs.msg import JointState
from edo_core_msgs.msg import JointStateArray
from edo.states import ordered_joint_names

# TODO: Gripper has been removed (with selector [:-1]) because unknown to the URDF

js = JointState(name=ordered_joint_names[:-1])
js_publisher = rospy.Publisher('joint_states', JointState, queue_size=45)

def js_callback(jsa):
    if jsa.joints_mask == 127:
        js.header.stamp = rospy.Time.now()
        js.position = [joint.position * 0.01745 for joint in jsa.joints][:-1]  # Convert Deg to Rad and delete last joint
        js.velocity = [joint.velocity * 0.01745 for joint in jsa.joints][:-1]  # Convert Deg to Rad and delete last joint
        js.effort = [joint.current for joint in jsa.joints][:-1] # TODO Approximate conversion motor_current => effort?
        js_publisher.publish(js)
    elif jsa.joints_mask < 9999999999999:  # Huge number pops when robot isn't ready
        raise NotImplementedError("Joint State publisher for real robot does not know edo joints mask {}".format(jsa.joints_mask))

rospy.init_node('joint_state_publisher')

rospy.Subscriber("usb_jnt_state", JointStateArray, js_callback)
rospy.loginfo("Starting eDo joint state publisher for real robot...")
rospy.spin()
