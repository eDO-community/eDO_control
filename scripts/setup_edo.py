#! /usr/bin/env python
# From https://github.com/andrejpan/edo_ui_python/blob/master/setup_edo.py

import rospy
from edo_core_msgs.msg import MachineState
from edo_core_msgs.msg import JointInit
from edo_core_msgs.msg import JointReset
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointCalibration
from edo_core_msgs.msg import MovementFeedback

# sudo -H pip3 install getkey on 16.04
# pip install getkey on 18.04
from getkey import getkey, keys


class EdoStates(object):
    NUMBER_OF_JOINTS = 7

    JOG_SPEED_MIN = 0.11
    JOG_SPEED_MAX = 0.99

    CS_DISCONNECTED = -2
    CS_UNKNOWN = -1
    CS_INIT = 0             # Initial state
    CS_NOT_CALIBRATED = 1   # uncalibrated machine
    CS_CALIBRATED = 2       # calibrated machine
    CS_MOVE = 3             # machine in execution of a move
    CS_JOG = 4              # machine running a jog
    CS_MACHINE_ERROR = 5    # machine in error status and waiting for a restart
    CS_BREAKED = 6          # brake active, no motor power supply
    CS_INIT_DISCOVER = 254  # UI internal state if we are initializing joints
    CS_COMMAND = 255        # state machine busy keep previous state, temporary status when there is a command running

    def __init__(self, current_state, opcode):
        self.edo_current_state = current_state
        self._edo_opcode_previous = opcode
        self._edo_current_state_previous = current_state
        self.edo_opcode = opcode
        self._edo_jog_speed = 0.5
        self.send_first_step_bool = False  # select 6-axis configuration
        self.send_second_step_bool = False  # disconnect the brakes
        self.send_third_step_bool = False  # calibration process will start
        self._current_joint = 0
        self._sent_next_movement_command_bool = False

        self._joint_init_command_pub = None
        self._joint_reset_command_pub = None
        self._jog_command_pub = None
        self._joint_calibration_command_pub = None
        self._movement_command_pub = None

        self.reselect_joints_bool = False

        self.disengage_brakes_timer = None
        self.disengage_brakes_bool = False
        self.read_input_bool = False

    def callback(self, msg):
        self.edo_current_state = msg.current_state
        self.edo_opcode = msg.opcode
        if self.edo_current_state != self._edo_current_state_previous or self.edo_opcode != self._edo_opcode_previous:

            rospy.loginfo("Current machine state: %s (%d), opcode %d" % (self.get_current_code_string(),
                                                                         self.edo_current_state,
                                                                         self.edo_opcode))
            self._edo_current_state_previous = self.edo_current_state
            self._edo_opcode_previous = self.edo_opcode

            # hack: input is blocking main while lopp ...
            if self.edo_current_state == self.CS_BREAKED and self.edo_opcode == 64:
                rospy.loginfo("Disengage emergency brake and press Enter to continue")

            if self.edo_current_state == self.CS_MACHINE_ERROR:
                rospy.logerr("Robot is in error state, terminating this node!")
                rospy.signal_shutdown("edo error")

    def move_ack_callback(self, msg):
        if msg.type == 0:
            pass
        elif msg.type == 1:
            pass
        elif msg.type == 2:
            rospy.loginfo("FB: Command executed, send next one if available")
            self._sent_next_movement_command_bool = True
        elif msg.type == -1:
            rospy.logerr("FB Error, msg.data: %d, " % msg.data)
        else:
            rospy.logerr("Feedback from a robot is not specified!!!: msg.type: $d, msg.data: %d" % msg.type, msg.data)
        # rospy.loginfo(msg)

    def get_current_code_string(self):
        if self.edo_current_state == self.CS_DISCONNECTED:
            return "DISCONNECTED"
        elif self.edo_current_state == self.CS_UNKNOWN:
            return "UNKNOWN"
        elif self.edo_current_state == self.CS_INIT:
            return "INIT"
        elif self.edo_current_state == self.CS_NOT_CALIBRATED:
            return "NOT_CALIBRATED"
        elif self.edo_current_state == self.CS_CALIBRATED:
            return "CALIBRATED"
        elif self.edo_current_state == self.CS_MOVE:
            return "MOVE"
        elif self.edo_current_state == self.CS_JOG:
            return "JOG"
        elif self.edo_current_state == self.CS_MACHINE_ERROR:
            return "MACHINE_ERROR"
        elif self.edo_current_state == self.CS_BREAKED:
            return "BREAKED"
        elif self.edo_current_state == self.CS_INIT_DISCOVER:
            return "INIT_DISCOVER"
        elif self.edo_current_state == self.CS_COMMAND:
            return "ROBOT IS BUSSY"

    def send_movement_command_init(self, msg):
        self._sent_next_movement_command_bool = False
        self._movement_command_pub.publish(msg)

    def send_movement_command(self, msg):
        while not self._sent_next_movement_command_bool and not rospy.is_shutdown():
            rospy.sleep(0.01)

        self._sent_next_movement_command_bool = False
        self._movement_command_pub.publish(msg)

    def select_6_axis_with_gripper_edo(self):
        rospy.loginfo("Trying to select 6-axis robot with a gripper...")
        # selecting 6 axis robot
        msg_ji = JointInit()
        # this code is from ros.service.ts
        msg_ji.mode = 0
        msg_ji.joints_mask = (1 << self.NUMBER_OF_JOINTS) - 1  # 127
        msg_ji.reduction_factor = 0.0
        # rospy.loginfo(msg_ji)
        self._joint_init_command_pub.publish(msg_ji)  # /bridge_init
        rospy.loginfo("Message sent, 6-axis configuration was selected")

    def disengage_brakes(self):
        rospy.loginfo("Trying to disengage brakes")
        msg_jr = JointReset()
        msg_jr.joints_mask = (1 << self.NUMBER_OF_JOINTS) - 1
        msg_jr.disengage_steps = 2000
        msg_jr.disengage_offset = 3.5
        self._joint_reset_command_pub.publish(msg_jr)  # /bridge_jnt_reset
        rospy.loginfo("Brakes should be disengaged...")

    def disengage_brakes_callback(self, timer_event):
        # TODO check what I can do with parameter event
        self.disengage_brakes()

    def create_jog_joint_command_message(self, sign):
        msg_mc = MovementCommand()
        msg_mc.move_command = 74
        msg_mc.move_type = 74
        msg_mc.ovr = 100
        msg_mc.target.data_type = 74
        msg_mc.target.joints_mask = 127
        msg_mc.target.joints_data = [0.0] * 10
        msg_mc.target.joints_data[self._current_joint] = sign * self._edo_jog_speed
        return msg_mc

    def create_jog_cartesian_command_message(self, sign):
        msg_mc = MovementCommand()
        msg_mc.move_command = 74
        msg_mc.move_type = 76
        msg_mc.ovr = 100
        # last joint/gripper has a diffrerent message
        if self._current_joint == 6:
            msg_mc.target.data_type = 88
            msg_mc.target.joints_mask = 127
            msg_mc.target.joints_data = [0.0] * 10
            msg_mc.target.joints_data[self._current_joint] = sign * self._edo_jog_speed
        else:
            msg_mc.target.data_type = 80
            if self._current_joint == 0:
                msg_mc.target.cartesian_data.x = sign * self._edo_jog_speed
            elif self._current_joint == 1:
                msg_mc.target.cartesian_data.a = sign * self._edo_jog_speed
            elif self._current_joint == 2:
                msg_mc.target.cartesian_data.y = sign * self._edo_jog_speed
            elif self._current_joint == 3:
                msg_mc.target.cartesian_data.e = sign * self._edo_jog_speed
            elif self._current_joint == 4:
                msg_mc.target.cartesian_data.z = sign * self._edo_jog_speed
            elif self._current_joint == 5:
                msg_mc.target.cartesian_data.r = sign * self._edo_jog_speed
            msg_mc.target.joints_mask = 127
        return msg_mc

    def calibration(self):
        # calibrating just first 6 joints!!!
        self._current_joint = 0  # always start calibration procedure with first joint

        rospy.loginfo("Entering jog/calibration loop")
        rospy.loginfo("    Use up/down arrows to increase/decrease jog speed")
        rospy.loginfo("    Use left/right arrows to jog joint")
        rospy.loginfo("    Use Enter to calibrate current joint and switch to next one")
        rospy.loginfo("Calibrating joint %d", self._current_joint + 1)

        while not rospy.is_shutdown():
            key = getkey()
            if key == keys.UP:
                if self._edo_jog_speed < self.JOG_SPEED_MAX:
                    self._edo_jog_speed += 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.DOWN:
                if self._edo_jog_speed > self.JOG_SPEED_MIN:
                    self._edo_jog_speed -= 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.RIGHT:
                self._jog_command_pub.publish(self.create_jog_joint_command_message(1))
            elif key == keys.LEFT:
                self._jog_command_pub.publish(self.create_jog_joint_command_message(-1))
            elif key == keys.PLUS:
                self._current_joint = (self._current_joint + 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Calibrating joint %d", self._current_joint + 1)
            elif key == keys.MINUS:
                self._current_joint = (self._current_joint - 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Calibrating joint %d", self._current_joint + 1)
            elif key == keys.ENTER:
                if self._current_joint < 0 or self._current_joint > self.NUMBER_OF_JOINTS-1:
                    rospy.logerr("Wrong number of joint %d", self._current_joint)
                    break
                msg_jc = JointCalibration()
                msg_jc.joints_mask = 1 << self._current_joint
                self._joint_calibration_command_pub.publish(msg_jc)

                # increase joint number or/and quit the calibration procedure
                self._current_joint += 1

                if self._current_joint >= self.NUMBER_OF_JOINTS-1:
                    self._current_joint = 0
                    rospy.loginfo("Calibration completed, exiting jog loop")
                    break
                else:
                    rospy.loginfo("Calibrating joint %d", self._current_joint + 1)
            elif key == keys.ESC:
                rospy.loginfo("Calibration NOT finished for all joints, exiting jog loop")
                break
            else:
                rospy.loginfo("Wrong button was presed")

    def jog(self):

        rospy.loginfo("Entering jog loop")
        rospy.loginfo("    Use up/down arrows to increase/decrease jog speed")
        rospy.loginfo("    Use left/right arrows to jog joint")
        rospy.loginfo("    Use +/- to increase/decrease joint number")
        rospy.loginfo("Jogging joint %d", self._current_joint + 1)

        while not rospy.is_shutdown():
            key = getkey()
            if key == keys.UP:
                if self._edo_jog_speed < self.JOG_SPEED_MAX:
                    self._edo_jog_speed += 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.DOWN:
                if self._edo_jog_speed > self.JOG_SPEED_MIN:
                    self._edo_jog_speed -= 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.RIGHT:
                self._jog_command_pub.publish(self.create_jog_joint_command_message(1))
            elif key == keys.LEFT:
                self._jog_command_pub.publish(self.create_jog_joint_command_message(-1))
            elif key == keys.PLUS:
                self._current_joint = (self._current_joint + 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Jogging joint %d", self._current_joint + 1)
            elif key == keys.MINUS:
                self._current_joint = (self._current_joint - 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Jogging joint %d", self._current_joint + 1)
            elif key == keys.ESC:
                rospy.loginfo("Exiting joint jog loop")
                break
            else:
                rospy.loginfo("Wrong button was presed")

    def jog_cartesian(self):

        array_of_cartesian = ['X-axis', 'Roll', 'Y-axis', 'Pitch', 'Z-axis', 'not exactly a Yaw', 'Gripper']

        rospy.loginfo("Entering jog Cartesian loop")
        rospy.loginfo("    Use up/down arrows to increase/decrease jog speed")
        rospy.loginfo("    Use left/right arrows to jog cartesian")
        rospy.loginfo("    Use +/- to select next/previus cartesian part")
        rospy.loginfo("    ['X-axis', 'Roll', 'Y-axis', 'Pitch', 'Z-axis', 'not exactly a Yaw', 'Gripper']")
        rospy.loginfo("Jogging cartesian: %s", array_of_cartesian[self._current_joint])

        while not rospy.is_shutdown():
            key = getkey()
            if key == keys.UP:
                if self._edo_jog_speed < self.JOG_SPEED_MAX:
                    self._edo_jog_speed += 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.DOWN:
                if self._edo_jog_speed > self.JOG_SPEED_MIN:
                    self._edo_jog_speed -= 0.1
                    rospy.loginfo("Jog speed: %.1f", self._edo_jog_speed)
            elif key == keys.RIGHT:
                pass
                self._jog_command_pub.publish(self.create_jog_cartesian_command_message(1))
            elif key == keys.LEFT:
                pass
                self._jog_command_pub.publish(self.create_jog_cartesian_command_message(-1))
            elif key == keys.PLUS:
                self._current_joint = (self._current_joint + 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Jogging cartesian: %s", array_of_cartesian[self._current_joint])
            elif key == keys.MINUS:
                self._current_joint = (self._current_joint - 1) % self.NUMBER_OF_JOINTS
                rospy.loginfo("Jogging cartesian: %s", array_of_cartesian[self._current_joint])
            elif key == keys.ESC:
                rospy.loginfo("Exiting cartesian jog loop")
                break
            else:
                rospy.loginfo("Wrong button was presed")

    def move_home(self):
        rospy.loginfo("Sending home position")

        # send reset command
        msg_mc = MovementCommand()
        msg_mc.move_command = 67
        self.send_movement_command_init(msg_mc)

        msg_mc = MovementCommand()
        msg_mc.move_command = 77
        msg_mc.move_type = 74
        msg_mc.ovr = 50
        msg_mc.target.data_type = 74
        msg_mc.target.joints_mask = (1 << self.NUMBER_OF_JOINTS) - 1
        msg_mc.target.joints_data = [0.0] * 7

        self.send_movement_command(msg_mc)
        rospy.loginfo("Home position should be soon reached")

    def move_joint(self, joint_values, joint_speed):

        msg_mc = MovementCommand()
        msg_mc.move_command = 77
        msg_mc.move_type = 74
        msg_mc.ovr = joint_speed
        msg_mc.delay = 255
        msg_mc.target.data_type = 74
        msg_mc.target.joints_mask = 127
        msg_mc.target.joints_data = [0.0] * 7

        # joint 7 are in degress
        msg_mc.target.joints_data[0] = joint_values[0]
        msg_mc.target.joints_data[1] = joint_values[1]
        msg_mc.target.joints_data[2] = joint_values[2]
        msg_mc.target.joints_data[3] = joint_values[3]
        msg_mc.target.joints_data[4] = joint_values[4]
        msg_mc.target.joints_data[5] = joint_values[5]
        # joint 7 is in milimeters
        msg_mc.target.joints_data[6] = joint_values[6]

        self.send_movement_command(msg_mc)

        while not self._sent_next_movement_command_bool and not rospy.is_shutdown():
            rospy.sleep(0.01)
        rospy.loginfo("Move joint should be completed")

    def execute_move_joint(self):
        rospy.loginfo("Setting up a joint movement:")
        try:
            repeat_number = int(raw_input('Write number of repetions cycles: '))
        except ValueError:
            rospy.loginfo("Not a number")
            rospy.loginfo("Exiting to a main loop.")
            return

        try:
            number_of_points = int(raw_input('Write number of different points: '))
        except ValueError:
            rospy.loginfo("Not a number")
            rospy.loginfo("Exiting to a main loop.")
            return

        list_of_poses = []
        rospy.loginfo("Input: j1 j2 j3 j4 j5 j6 gripper_span")
        rospy.loginfo("Joint data in degrees and gripper span in mm")

        for i in range(0, number_of_points):
            try:
                j1, j2, j3, j4, j5, j6, gripper_span = [int(x) for x in raw_input("Input: ").split()]
            except ValueError:
                rospy.loginfo("Not a valid angles or gripper span...")
                rospy.loginfo("Exiting to a main loop.")
                return
            list_of_poses.append([j1, j2, j3, j4, j5, j6, gripper_span])

        for item in list_of_poses:
            if len(item) != self.NUMBER_OF_JOINTS:
                rospy.logerr("To many or not enough joint values for a robot")
                rospy.loginfo("Exiting to a main loop.")
                return
        try:
            joint_speed = int(raw_input('Write joint speed [0,..,100]: '))
        except ValueError:
            rospy.loginfo("Not a number")
            rospy.loginfo("Exiting to a main loop.")
            return
        joint_speed %= 100

        # send reset message
        msg_mc = MovementCommand()
        msg_mc.move_command = 67
        self.send_movement_command_init(msg_mc)

        # send all the data
        for i in range(0, repeat_number):
            for j in range(0, number_of_points):
                if not rospy.is_shutdown():
                    self.move_joint(list_of_poses[j], joint_speed)
                    rospy.loginfo(list_of_poses[j])

        if rospy.is_shutdown():
            rospy.loginfo("Terminating an execution loop...")
            rospy.loginfo("Exiting to a main loop.")
            return

        rospy.loginfo("Joint movement was successful.")
        rospy.loginfo("Exiting to a main loop.")
        return


def main():
    rospy.loginfo("Main loop has started")

    rospy.init_node('python_configuration_node', anonymous=True)
    # True ensures that your node has a unique name by adding random numbers to the end of NAME

    states = EdoStates(-1, -1)

    rospy.Subscriber("/machine_state", MachineState, states.callback)
    # ros::spin() will enter a loop, pumping callbacks.
    # rospy.spin() # this is replaced with a loop

    joint_init_command_pub = rospy.Publisher('/bridge_init', JointInit, queue_size=10, latch=True)
    states._joint_init_command_pub = joint_init_command_pub

    joint_reset_command_pub = rospy.Publisher('/bridge_jnt_reset', JointReset, queue_size=10, latch=True)
    states._joint_reset_command_pub = joint_reset_command_pub

    jog_command_pub = rospy.Publisher('/bridge_jog', MovementCommand, queue_size=10, latch=True)
    states._jog_command_pub = jog_command_pub

    joint_calibration_command_pub = rospy.Publisher('/bridge_jnt_calib', JointCalibration, queue_size=10, latch=True)
    states._joint_calibration_command_pub = joint_calibration_command_pub

    states._movement_command_pub = rospy.Publisher('/bridge_move', MovementCommand, queue_size=10, latch=True)

    rospy.Subscriber('/machine_movement_ack', MovementFeedback, states.move_ack_callback)

    rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():

        # TODO move this if statements to callback function
        if states.edo_current_state == states.CS_INIT and states.edo_opcode == 0 and not states.send_first_step_bool:
            # I should not come back to this state
            states.send_first_step_bool = True
            states.select_6_axis_with_gripper_edo()

        if states.edo_current_state == states.CS_BREAKED and states.edo_opcode == 72 and not states.send_second_step_bool:
            # disengage brakes to uncalibrated robot
            states.send_second_step_bool = True
            states.disengage_brakes()

        if states.edo_current_state == states.CS_INIT and \
                (states.edo_opcode == 64 or states.edo_opcode == 72) and not states.reselect_joints_bool:
            # according to tablet, select 7 joints again and start from scratch...
            states.reselect_joints_bool = True
            states.select_6_axis_with_gripper_edo()

        if states.edo_current_state == states.CS_NOT_CALIBRATED and states.edo_opcode == 8 and not states.send_third_step_bool:
            rospy.loginfo("Starting calibration jog loop")
            states.calibration()
            states.send_third_step_bool = True
            states.read_input_bool = True

        if states.edo_current_state == states.CS_BREAKED and states.edo_opcode == 64:
            # disengage brakes to calibrated robot on every two seconds until emergency button is released
            states.read_input_bool = False
            if not states.disengage_brakes_bool:
                rospy.loginfo("Setting up release breakes timer")
                states.disengage_brakes_timer = rospy.Timer(rospy.Duration(2), states.disengage_brakes_callback)
                states.disengage_brakes_bool = True

        if states.edo_current_state == states.CS_CALIBRATED and states.edo_opcode == 0:
            if states.disengage_brakes_bool:
                rospy.loginfo("Robot is active again, disabling release breakes timer")
                states.disengage_brakes_timer.shutdown()
                states.disengage_brakes_bool = False
            states.read_input_bool = True

        # rospy.loginfo("Waiting a key before")
        if states.read_input_bool:
            rospy.loginfo("Press j to start jogging in joint space")
            rospy.loginfo("Press c to start jogging in cartesian space")
            rospy.loginfo("Press h to send robot to home position")
            rospy.loginfo("Press r to restart calibration")
            rospy.loginfo("Press f to start simple joint movement")
            rospy.loginfo("Press ESC to quit the program")
            key = getkey(blocking=True)
            if key == keys.J:
                states.jog()
            elif key == keys.C:
                states.jog_cartesian()
            elif key == keys.H:
                states.move_home()
            elif key == keys.R:
                states.calibration()
            elif key == keys.F:
                states.execute_move_joint()
            elif key == keys.ESC:
                rospy.loginfo("Exiting/Killing the main loop/program")
                rospy.signal_shutdown("setup_edo.py shutdown")

        rate.sleep()
        # rospy.loginfo("loop")


if __name__ == '__main__':

    # rospy.sleep() and rospy.Rate.sleep() can thrown exception
    try:
        main()
    except rospy.ROSInterruptException:
        pass

