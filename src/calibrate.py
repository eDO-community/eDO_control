#!/usr/bin/env python

import rospy
from edo.states import EdoStates


if __name__ == '__main__':
    rospy.init_node('edo_calibrate', anonymous=True)
    states = EdoStates(-1, -1) 
    calibrated = False
    rospy.logwarn("Starting robot calibration procedure...")
    try:
        calibrated = states.calibration()
    except:
        rospy.logerr("Robot calibration did not finish")
    else:
        if calibrated:
            rospy.logwarn("Robot is calibrated!")
        else:
            rospy.logerr("Robot calibration did not finish successfully")

