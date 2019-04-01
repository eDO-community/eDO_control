# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
# JTAS adapted by Yoan Mollard for e.DO robot, meeting the license hereunder
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import errno
import actionlib
import bisect
from copy import deepcopy
import math
import operator
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16
from sensor_msgs.msg import JointState
from edo_core_msgs.msg import MachineState
from edo_core_msgs.msg import JointInit
from edo_core_msgs.msg import JointReset
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointCalibration
from edo_core_msgs.msg import MovementFeedback
from edo.states import EdoStates, ordered_joint_names
from edo.pid import PID
from edo.bezier import bezier_point, de_boor_control_pts, bezier_coefficients

def wait_for(test, timeout=1.0, raise_on_error=True, rate=100,
             timeout_msg="timeout expired", body=None):
    """
    Waits until some condition evaluates to true.
    @param test: zero param function to be evaluated
    @param timeout: max amount of time to wait. negative/inf for indefinitely
    @param raise_on_error: raise or just return False
    @param rate: the rate at which to check
    @param timout_msg: message to supply to the timeout exception
    @param body: optional function to execute while waiting
    """
    end_time = rospy.get_time() + timeout
    rate = rospy.Rate(rate)
    notimeout = (timeout < 0.0) or timeout == float("inf")
    while not test():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS Shutdown")
            return False
        elif (not notimeout) and (rospy.get_time() >= end_time):
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            return False
        if callable(body):
            body()
        rate.sleep()
    return True

class JointTrajectoryActionServer(object):
    def __init__(self, reconfig_server, rate=100.0, mode='position'):
        self._dyn = reconfig_server
        self._fjt = '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        # Verify joint control mode
        self._mode = mode
        if (self._mode != 'position' and self._mode != 'velocity'):
            rospy.logerr("%s: Action Server Creation Failed - "
                         "Provided Invalid Joint Control Mode '%s' (Options: "
                         "'position', 'velocity')" %
                    (self._action_name, self._mode,))
            return

        # Current joint states
        self._positions = {}
        self._velocities = {}

        # Actual robot control
        self.states = EdoStates(-1, -1)
        rospy.Subscriber("joint_states", JointState, self._js_callback)

        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments, messages, and dynamic reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()

        # Create our PID controllers
        self._pid = dict()
        for joint in ordered_joint_names:
            self._pid[joint] = PID()

        # Create our spline coefficients
        self._coeff = [None] * len(ordered_joint_names)

        # Start the action server
        rospy.sleep(0.5)
        if self.states.edo_current_state == self.states.CS_CALIBRATED:
            self._server.start()
            self._alive = True
        else:
            rospy.logerr("Joint Trajectory Action Server cannot be started when robot is in state %s" % self.states.get_current_code_string())
            rospy.logerr("Make sure your robot is started and calibrated properly with calibrate.launch")

    def robot_is_enabled(self):
        return True # TODO read e-stop

    def clean_shutdown(self):
        self._alive = False

    def _js_callback(self, js):
        self._positions.update(dict(zip(js.name, js.position)))
        self._velocities.update(dict(zip(js.name, js.velocity)))

    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = self._dyn.config['goal_time']
        # Stopped velocity tolerance - max velocity at end of execution
        self._stopped_velocity = self._dyn.config['stopped_velocity_tolerance']

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in ordered_joint_names:
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
            # Path execution tolerance
            path_error = self._dyn.config[jnt + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config[jnt + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error

            # PID gains if executing using the velocity (integral) controller
            if self._mode == 'velocity':
                self._pid[jnt].set_kp(self._dyn.config[jnt + '_kp'])
                self._pid[jnt].set_ki(self._dyn.config[jnt + '_ki'])
                self._pid[jnt].set_kd(self._dyn.config[jnt + '_kd'])
                self._pid[jnt].initialize()

    def _get_current_position(self, joint_names):
        return [self._positions[joint] for joint in joint_names]

    def _get_current_velocities(self, joint_names):
        return [self._velocities[joint] for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self, joint_names):
        values = [0.0] * len(joint_names)
        while (not self._server.is_new_goal_available() and self._alive and self.robot_is_enabled()):
            self.states._jog_command_pub.publish(self.states.create_jog_joints_command_message(values))
            rospy.sleep(1.0 / self._control_rate)

    def _command_joints(self, joint_names, point, start_time, dimensions_dict):
        if self._server.is_preempt_requested() or not self.robot_is_enabled():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop(joint_names)
            return False
        
        velocities = []
        deltas = self._get_current_error(joint_names, point.positions)
        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]] and self._path_thresh[delta[0]] >= 0.0)) or not self.robot_is_enabled():
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" % (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop(joint_names)
                return False
            if self._mode == 'velocity':
                velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        if ((self._mode == 'position') and self._alive):
            rospy.logerr(str(point.positions))
            #rospy.logerr("Uncomment but create_jog_joints_command_message expects 1/-1, should be normalized? what's in point.positions?")
            self.states._jog_command_pub.publish(self.states.create_jog_joints_command_message(point.positions))
            #self._server.set_aborted()
            return False
        elif self._alive:
            rospy.logerr("Not Implemented: joint velocity control. Is it possible in edo_core_msgs/MovementCommand with 'ovr'?")
            self._server.set_aborted()
            return False
        return True

    def _get_bezier_point(self, b_matrix, idx, t, cmd_time, dimensions_dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = b_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        if dimensions_dict['velocities']:
            pnt.velocities = [0.0] * num_joints
        if dimensions_dict['accelerations']:
            pnt.accelerations = [0.0] * num_joints
        for jnt in range(num_joints):
            b_point = bezier_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
            # Velocities at specified time
            if dimensions_dict['velocities']:
                pnt.velocities[jnt] = b_point[1]
            # Accelerations at specified time
            if dimensions_dict['accelerations']:
                pnt.accelerations[jnt] = b_point[-1]
        return pnt

    def _compute_bezier_coeff(self, joint_names, trajectory_points, dimensions_dict):
        # Compute Full Bezier Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_b_values = len(['b0', 'b1', 'b2', 'b3'])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_b_values))
        for jnt in xrange(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                if dimensions_dict['velocities']:
                    current_point.append(point.velocities[jnt])
                if dimensions_dict['accelerations']:
                    current_point.append(point.accelerations[jnt])
                traj_array[idx, :] = current_point
            d_pts = de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier_coefficients(traj_array, d_pts)
        return b_matrix

    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        velocity_flag = (len(trajectory_points[0].velocities) != 0 and len(trajectory_points[-1].velocities) != 0)
        acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and len(trajectory_points[-1].accelerations) != 0)
        return {'positions':position_flag,
                'velocities':velocity_flag,
                'accelerations':acceleration_flag}

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" % (self._action_name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))
        control_rate = rospy.Rate(self._control_rate)

        dimensions_dict = self._determine_dimensions(trajectory_points)

        if num_points == 1:
            # Add current position as trajectory point
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position(joint_names)
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            if dimensions_dict['velocities']:
                first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            if dimensions_dict['accelerations']:
                first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)

        # Force Velocites/Accelerations to zero at the final timestep if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories, and want continuous, non-zero velocities/accelerations between trajectories
        if dimensions_dict['velocities']:
            trajectory_points[-1].velocities = [0.0] * len(joint_names)
        if dimensions_dict['accelerations']:
            trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        try:
            b_matrix = self._compute_bezier_coeff(joint_names, trajectory_points, dimensions_dict)
        except Exception as ex:
            rospy.logerr(("{0}: Failed to compute a Bezier trajectory with error \"{2}: {3}\"").format(self._action_name, type(ex).__name__, ex))
            self._server.set_aborted()
            return
        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown() and self.robot_is_enabled()):
            #Acquire Mutex
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)
            #Calculate percentage of time passed in this interval
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0
                t = 0

            point = self._get_bezier_point(b_matrix, idx, t, cmd_time, dimensions_dict)

            # Command Joint Position, Velocity, Acceleration
            command_executed = self._command_joints(joint_names, point, start_time, dimensions_dict)
            self._update_feedback(deepcopy(point), joint_names, now_from_start)
            # Release the Mutex
            if not command_executed:
                return
            control_rate.sleep()
        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))

        def check_goal_state():
            for error in self._get_current_error(joint_names, last.positions):
                if (self._goal_error[error[0]] > 0
                        and self._goal_error[error[0]] < math.fabs(error[1])):
                    return error[0]
            if (self._stopped_velocity > 0.0 and
                max([abs(cur_vel) for cur_vel in self._get_current_velocities(joint_names)]) >
                    self._stopped_velocity):
                return False
            else:
                return True

        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown() and self.robot_is_enabled()):
            if not self._command_joints(joint_names, last, start_time, dimensions_dict):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        # Verify goal constraint
        result = check_goal_state()
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded" % (self._action_name,))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold" % (self._action_name,))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s" % (self._action_name, result,))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop(goal.trajectory.joint_names)


