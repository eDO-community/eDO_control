#!/usr/bin/env python

import rospy
from edo.states import EdoStates


if __name__ == '__main__':
    rospy.init_node('edo_calibrate', anonymous=True)
    states = EdoStates(-1, -1) 
    rospy.logwarn("Starting robot calibration procedure...")
    try:
        states.calibration()
    except:
        rospy.logerr("Robot calibration did not finish")
    else:
        rospy.logwarn("Robot is calibrated!")

