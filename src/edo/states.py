#! /usr/bin/env python

import rospy
from edo_core_msgs.msg import MachineState
from edo_core_msgs.msg import JointInit
from edo_core_msgs.msg import JointReset
from edo_core_msgs.msg import JointControlArray
from edo_core_msgs.msg import JointControl
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointCalibration
from edo_core_msgs.msg import MovementFeedback
from edo.messages import errors

from getkey import getkey, keys


class EdoStates(object):
    NUMBER_OF_JOINTS = 7

    JOG_SPEED_MIN = 0.11
    JOG_SPEED_MAX = 0.99

    # Robot states in /machine_state
    CS_DISCONNECTED = -2
    CS_UNKNOWN = -1
    CS_INIT = 0             # Initial state
    CS_NOT_CALIBRATED = 1   # uncalibrated machine
    CS_CALIBRATED = 2       # calibrated machine
    CS_MOVE = 3             # machine in execution of a move
    CS_JOG = 4              # machine running a jog
    CS_MACHINE_ERROR = 5    # machine in error status and waiting for a restart
    CS_BRAKED = 6           # brake active, no motor power supply
    CS_INIT_DISCOVER = 254  # UI internal state if we are initializing joints
    CS_COMMAND = 255        # state machine busy keep previous state, temporary status when there is a command running

    # opcodes (bitwise) in /machine_state
    OP_NACK = 1                 # At least 1 joint didn't send an ACK
    OP_JOINT_ABSENT = 2         # A joint is not publishing its status. Hard stop
    OP_JOINT_OVERCURRENT = 4    # Joint in over current. Hard stop
    OP_JOINT_UNCALIBRATED = 8   # Joints not calibrated. Only jogs are accepted
    OP_POSITION_ERROR = 16      # Position error. Hard stop
    OP_ROSSERIAL_ERROR = 32     # Rosserial error. No state from joints. Hard stop
    OP_BRAKE_ACTIVE = 64        # Brake active. No power supply provided to motors
    OP_EMERGENCY_STOP = 128     # E-stop active [as documented in code; surprisingly actual E-STOP is not 128 but OP_BRAKE_ACTIVE]
    OP_FENCE = 256              # Fence active

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
        self._joint_calibration_command_pub = None
        self._movement_command_pub = None

        self.reselect_joints_bool = False

        self.disengage_brakes_timer = None
        self.disengage_brakes_bool = False
        self.read_input_bool = False

        self.msg_jca = JointControlArray()
        self.msg_mc = MovementCommand()

        # Joint Command topics
        self.jog_command_pub = None
        self.joint_control_pub = None

        rospy.Subscriber("/machine_state", MachineState, self.callback)

        joint_init_command_pub = rospy.Publisher('/bridge_init', JointInit, queue_size=10, latch=True)
        self._joint_init_command_pub = joint_init_command_pub

        joint_reset_command_pub = rospy.Publisher('/bridge_jnt_reset', JointReset, queue_size=10, latch=True)
        self._joint_reset_command_pub = joint_reset_command_pub

        jog_command_pub = rospy.Publisher('/bridge_jog', MovementCommand, queue_size=10, latch=True)
        self.jog_command_pub = jog_command_pub

        joint_control_pub = rospy.Publisher('/algo_jnt_ctrl', JointControlArray, queue_size=1)
        self.joint_control_pub = joint_control_pub

        joint_calibration_command_pub = rospy.Publisher('/bridge_jnt_calib', JointCalibration, queue_size=10, latch=True)
        self._joint_calibration_command_pub = joint_calibration_command_pub

        self._movement_command_pub = rospy.Publisher('/bridge_move', MovementCommand, queue_size=10, latch=True)

        rospy.Subscriber('/machine_movement_ack', MovementFeedback, self.move_ack_callback)        

    def callback(self, msg):
        self.edo_current_state = msg.current_state
        self.edo_opcode = msg.opcode
        if self.edo_current_state != self._edo_current_state_previous or self.edo_opcode != self._edo_opcode_previous:

            rospy.loginfo("Current machine state: %s (%d), opcode %d" % (self.get_current_code_string(),
                                                                         self.edo_current_state,
                                                                         self.edo_opcode))
            if self.edo_opcode != 0:
                rospy.logwarn(str(self.get_current_opcode_messages()))
            self._edo_current_state_previous = self.edo_current_state
            self._edo_opcode_previous = self.edo_opcode

            if self.edo_current_state == self.CS_MACHINE_ERROR:
                rospy.logerr("Robot is in error state, please reboot the robot!")

    def move_ack_callback(self, msg):
        if msg.type == 0:
            pass
        elif msg.type == 1:
            pass
        elif msg.type == 2:
            self._sent_next_movement_command_bool = True
        elif msg.type == -1:
            rospy.logerr("Feedback Error: {}".format(errors[msg.data] if msg.data in errors else msg.data))
        else:
            rospy.logerr("Feedback from robot is unknown: msg.type: %d, msg.data: %d" % msg.type, msg.data)

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
        elif self.edo_current_state == self.CS_BRAKED:
            return "BRAKED"
        elif self.edo_current_state == self.CS_INIT_DISCOVER:
            return "INIT_DISCOVER"
        elif self.edo_current_state == self.CS_COMMAND:
            return "ROBOT IS BUSY"

    def get_current_opcode_messages(self):
        messages = []
        if self.edo_opcode & 1:
            messages.append("NACK: At least 1 joint didn't send an ACK")
        if self.edo_opcode & 2:
            messages.append("JOINT_ABSENT: A joint is not publishing its status. Hard stop")
        if self.edo_opcode & 4:
            messages.append("JOINT_OVERCURRENT: Joint in over current. Hard stop")
        if self.edo_opcode & 8:
            messages.append("JOINT_UNCALIBRATED: Joints not calibrated. Only jogs are accepted")
        if self.edo_opcode & 16:
            messages.append("POSITION_ERROR: Position error. Hard stop")
        if self.edo_opcode & 32:
            messages.append("ROSSERIAL_ERROR: Rosserial error. No state from joints. Hard stop")
        if self.edo_opcode & 64:
            messages.append("BRAKE_ACTIVE: Brake active. No power supply provided to motors")
        if self.edo_opcode & 128:
            messages.append("EMERGENCY_STOP: E-stop active")
        if self.edo_opcode & 256:
            messages.append("FENCE: Fence active")
        return messages

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
        rospy.loginfo("Trying to disengage brakes...")
        msg_jr = JointReset()
        msg_jr.joints_mask = (1 << self.NUMBER_OF_JOINTS) - 1
        msg_jr.disengage_steps = 2000
        msg_jr.disengage_offset = 3.5
        self._joint_reset_command_pub.publish(msg_jr)  # /bridge_jnt_reset
        rospy.loginfo("Brakes should be disengaged")

    def disengage_brakes_callback(self, timer_event):
        # TODO check what I can do with parameter event
        self.disengage_brakes()

    def create_jog_joints_command_message(self, values):
        self.msg_mc.move_command = 74
        self.msg_mc.move_type = 74
        self.msg_mc.ovr = 100
        self.msg_mc.target.data_type = 74
        self.msg_mc.target.joints_mask = 127
        self.msg_mc.target.joints_data = [0.0] * 10
        for i, value in enumerate(values):
            self.msg_mc.target.joints_data[i] = value
        return self.msg_mc

    def create_move_commande_messages(self, joint_names, point):
        self.msg_mc.move_command = 77
        self.msg_mc.move_type = 74
        self.msg_mc.ovr = 50
        self.msg_mc.target.data_type = 74
        self.msg_mc.target.joints_mask = (1 << len(joint_names)) - 1
        self.msg_mc.target.joints_data = [JointControl(point.positions[i]/0.01745, point.velocities[i]/0.01745, 0, 0, 0) for i in range(self.msg_jca.size)]
        return self.msg_mc

    def create_jog_joint_command_message(self, sign):
        self.msg_mc.move_command = 74
        self.msg_mc.move_type = 74
        self.msg_mc.ovr = 100
        self.msg_mc.target.data_type = 74
        self.msg_mc.target.joints_mask = 127
        self.msg_mc.target.joints_data = [0.0] * 10
        self.msg_mc.target.joints_data[self._current_joint] = sign * self._edo_jog_speed
        return self.msg_mc

    def create_joint_command_message(self, joint_names, point):
        self.msg_jca.size = len(joint_names) + 1
        # Convert back command from radians to degrees
        # TODO: How can we exploit acceleration to provide ff_velocity and maybe current instead of 0, 0, 0?
        self.msg_jca.joints = [JointControl(point.positions[i]/0.01745, point.velocities[i]/0.01745, 0, 0, 0) for i in range(6)] + [JointControl(0, 0, 0, 0, 0)]
        return self.msg_jca

    def calibration(self):
        if self.edo_current_state == self.CS_CALIBRATED and self.edo_opcode == 0:
            rospy.logwarn("Robot was already calibrated, going for a new calibration...")
        else:
            while not (self.edo_current_state == self.CS_NOT_CALIBRATED and self.edo_opcode == self.OP_JOINT_UNCALIBRATED) and not rospy.is_shutdown():
                rospy.loginfo("Waiting machine state CS_NOT_CALIBRATED (currently {}) and opcode OP_CS_NOT_CALIBRATED (currently {})...".format(
                    self.get_current_code_string(), self.get_current_opcode_messages()))
                self.update()
                rospy.sleep(1)
        
        if rospy.is_shutdown(): return False

        # calibrating just first 6 joints!!!
        self._current_joint = 0  # always start calibration procedure with first joint

        def display_help():
            rospy.loginfo("Entering jog/calibration loop")
            rospy.loginfo("    Use up/down arrows to increase/decrease jog speed")
            rospy.loginfo("    Use left/right arrows to jog joint")
            rospy.loginfo("    Use Enter to calibrate current joint and switch to next one")
            rospy.loginfo("Calibrating joint %d", self._current_joint + 1)

        display_help()
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
                self.jog_command_pub.publish(self.create_jog_joint_command_message(1))
            elif key == keys.LEFT:
                self.jog_command_pub.publish(self.create_jog_joint_command_message(-1))
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
                    rospy.sleep(0.5)
                    return self.edo_current_state == self.CS_CALIBRATED and self.edo_opcode == 0
                else:
                    rospy.loginfo("Calibrating joint %d...", self._current_joint + 1)
            elif key == keys.ESC:
                rospy.loginfo("Calibration NOT finished for all joints, exiting jog loop")
                break
            else:
                rospy.logwarn("Wrong button was pressed")
                display_help()

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
                self.jog_command_pub.publish(self.create_jog_joint_command_message(1))
            elif key == keys.LEFT:
                self.jog_command_pub.publish(self.create_jog_joint_command_message(-1))
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
                rospy.loginfo("Wrong button was pressed")

    def update(self):
        if self.edo_current_state == self.CS_INIT and self.edo_opcode == 0 and not self.send_first_step_bool:
            # I should not come back to this state
            self.send_first_step_bool = True
            self.select_6_axis_with_gripper_edo()

        if self.edo_current_state == self.CS_BRAKED and self.edo_opcode == (self.OP_JOINT_UNCALIBRATED | self.OP_BRAKE_ACTIVE) \
                and not self.send_second_step_bool:
            # disengage brakes to uncalibrated robot
            self.send_second_step_bool = True
            self.disengage_brakes()

        if self.edo_current_state == self.CS_INIT and \
                (self.edo_opcode == self.OP_BRAKE_ACTIVE or self.edo_opcode == (self.OP_BRAKE_ACTIVE | self.OP_JOINT_UNCALIBRATED)) \
                and not self.reselect_joints_bool:
            # according to tablet, select 7 joints again and start from scratch...
            self.reselect_joints_bool = True
            self.select_6_axis_with_gripper_edo()

        if self.edo_current_state == self.CS_NOT_CALIBRATED and self.edo_opcode == self.OP_JOINT_UNCALIBRATED:
            rospy.logerr("Your robot is not calibrated, please calibrate it with calibrate.launch first")

        if self.edo_current_state == self.CS_BRAKED and self.edo_opcode == self.OP_BRAKE_ACTIVE:
            # disengage brakes to calibrated robot on every two seconds until emergency button is released
            self.read_input_bool = False
            if not self.disengage_brakes_bool:
                rospy.loginfo("Setting up release brakes timer")
                self.disengage_brakes_timer = rospy.Timer(rospy.Duration(2), self.disengage_brakes_callback)
                self.disengage_brakes_bool = True

        if self.edo_current_state == self.CS_CALIBRATED and self.edo_opcode == 0:
            if self.disengage_brakes_bool:
                rospy.loginfo("Robot is active again, disabling release brakes timer")
                # TODO This is blocking
                self.disengage_brakes_timer.shutdown()
                self.disengage_brakes_bool = False
            self.read_input_bool = True

ordered_joint_names = ["edo_joint_" + str(j+1) for j in range(EdoStates.NUMBER_OF_JOINTS)]
