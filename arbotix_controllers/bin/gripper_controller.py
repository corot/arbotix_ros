#!/usr/bin/env python

"""
  gripper_controller - action based controller for grippers.
  Copyright (c) 2011-2014 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy, actionlib
import collections
import threading

from control_msgs.msg import GripperCommandAction, GripperCommandResult, GripperCommandFeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import sin, asin

class TrapezoidGripperModel:
    """ A simple gripper with two opposing servos to open/close non-parallel jaws. """

    def __init__(self):
        # trapezoid model: base width connecting each gripper's rotation point
            #              + length of gripper fingers to computation point
            #              = compute angles based on a desired width at comp. point
        self.pad_width = rospy.get_param('~pad_width', 0.01)
        self.finger_length = rospy.get_param('~finger_length', 0.02)
        self.min_opening = rospy.get_param('~min_opening', 0.0)
        self.max_opening = rospy.get_param('~max_opening', 0.09)
        self.center_l = rospy.get_param('~center_left', 0.0)
        self.center_r = rospy.get_param('~center_right', 0.0)
        self.invert_l = rospy.get_param('~invert_left', False)
        self.invert_r = rospy.get_param('~invert_right', False)

        self.left_joint = rospy.get_param('~joint_left', 'l_gripper_joint')
        self.right_joint = rospy.get_param('~joint_right', 'r_gripper_joint')

        # publishers
        self.l_pub = rospy.Publisher(self.left_joint+'/command', Float64, queue_size=5)
        self.r_pub = rospy.Publisher(self.right_joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        # check limits
        if command.position > self.max_opening or command.position < self.min_opening:
            rospy.logerr("Command (%f) exceeds opening limits (%f, %f)",
                          command.position, self.max_opening, self.min_opening)
            return False
        # compute angles
        angle = asin((command.position - self.pad_width)/(2*self.finger_length))
        if self.invert_l:
            l = -angle + self.center_l
        else:
            l = angle + self.center_l
        if self.invert_r:
            r = angle + self.center_r
        else:
            r = -angle + self.center_r
        # publish msgs
        lmsg = Float64(l)
        rmsg = Float64(r)
        self.l_pub.publish(lmsg)
        self.r_pub.publish(rmsg)
        return True

    def getPosition(self, js):
        left = right = 0
        for i in range(len(js.name)):
            if js.name[i] == self.left_joint:
                left = js.position[i]
            elif js.name[i] == self.right_joint:
                right = js.position[i]
        # TODO

        return 0.0

    def getEffort(self, joint_states):
        return 1.0

class ParallelGripperModel:
    """ One servo to open/close parallel jaws, typically via linkage. """

    def __init__(self):
        self.center = rospy.get_param('~center', 0.0)
        self.scale = rospy.get_param('~scale', 1.0)
        self.joint = rospy.get_param('~joint', 'gripper_joint')

        # publishers
        self.pub = rospy.Publisher(self.joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        self.pub.publish((command.position * self.scale) + self.center)

    def getPosition(self, joint_states):
        return 0.0

    def getEffort(self, joint_states):
        return 1.0


class OneSideGripperModel:
    """ Simplest of grippers, one servo opens or closes to achieve a particular size opening. """

    def __init__(self):
        self.pad_width = rospy.get_param('~pad_width', 0.01)
        self.finger_length = rospy.get_param('~finger_length', 0.02)
        self.min_opening = rospy.get_param('~min_opening', 0.0)
        self.max_opening = rospy.get_param('~max_opening', 0.09)
        self.center = rospy.get_param('~center', 0.0)
        self.invert = rospy.get_param('~invert', False)
        self.joint = rospy.get_param('~joint', 'gripper_joint')

        # publishers
        self.pub = rospy.Publisher(self.joint+'/command', Float64, queue_size=5)

    def setCommand(self, command):
        """ Take an input command of width to open gripper. """
        # check opening limits and bound if necessary
        if command.position > self.max_opening:
            rospy.logwarn("Command (%f) exceeds opening limits; we will use max opening instead (%f)",
                           command.position, self.max_opening)
            command.position = self.max_opening
        elif command.position < self.min_opening:
            rospy.logwarn("Command (%f) exceeds opening limits; we will use min opening instead (%f)",
                           command.position, self.min_opening)
            command.position = self.min_opening

        # compute angle
        angle = asin((command.position - self.pad_width)/(2*self.finger_length))
        # publish message
        if self.invert:
            self.pub.publish(-angle + self.center)
        else:
            self.pub.publish(angle + self.center)

        return True

    def getPosition(self, joint_angle):
        # revert angle calculation done in setCommand
        if self.invert:
            angle = self.center - joint_angle 
        else:
            angle = joint_angle - self.center

        return sin(angle)*2*self.finger_length + self.pad_width

    def getEffort(self, joint_states):
        # TODO
        return 1.0


class GripperActionController:
    """ The actual action callbacks. """
    def __init__(self):
        rospy.init_node('gripper_controller')

        # auxiliary constants; make parameters if not generic enough
        self._EPSILON_OPENING_DIFF_ = 0.001  # one millimeter; close enough to goal position
        
        # buffer used to estimate gripper joint state topic frequency 
        self._STATE_HZ_BUFFER_SIZE_ = 10 
        
        self.state_cb_event = threading.Event()
        self.state_cb_times = collections.deque(maxlen=self._STATE_HZ_BUFFER_SIZE_)
        
        # time the controller will wait before deciding that the gripper is stalled
        # WARN: if too long, and the commanded pose is smaller than the grasped object,
        # the servo can get jammed (it will stop working ant its led will start blinking)
        self.stalled_time = rospy.get_param('~stalled_time', 0.2)

        # setup model
        try:
            model = rospy.get_param('~model')
        except:
            rospy.logerr('no model specified, exiting')
            exit()
        if model == 'dualservo':
            self.model = TrapezoidGripperModel()
        elif model == 'parallel':
            self.model = ParallelGripperModel()
        elif model == 'singlesided':
            self.model = OneSideGripperModel()
        else:
            rospy.logerr('Gripper Controller: unknown model specified, exiting')
            exit()

        # subscribe to joint_states
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        # subscribe to command and then spin
        self.server = actionlib.SimpleActionServer('~gripper_action', GripperCommandAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.spin()

    def actionCb(self, goal):
        result = GripperCommandResult()
        feedback = GripperCommandFeedback()
        
        """ Take an input command of width to open gripper. """
        rospy.loginfo('Gripper Controller action goal received: %f' % goal.command.position)
        
        # send command to gripper
        if not self.model.setCommand(goal.command):
            self.server.set_aborted()
            rospy.loginfo('Gripper Controller: Aborted.')
            return
        
        # register progress so we can guess if the gripper is stalled; our buffer 
        # must contain up to: stalled_time / joint_states period position values
        if len(self.state_cb_times) == self.state_cb_times.maxlen:
            T = sum(self.state_cb_times[i] - self.state_cb_times[i-1] \
                    for i in xrange(1, len(self.state_cb_times) - 1)) \
                 /(len(self.state_cb_times) - 1) 
            progress = collections.deque(maxlen=round(self.stalled_time/T))
        else:
            self.server.set_aborted()
            rospy.logerr('Gripper Controller: no messages from joint_states topic received')
            return

        diff_at_start = round(abs(goal.command.position - self.current_position), 3)

        # keep watching for gripper position...
        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.loginfo('Gripper Controller: Preempted.')
                return

            # synchronize with the joints state callbacks;
            self.state_cb_event.wait()

            # break when we have reached the goal position...
            diff = abs(goal.command.position - self.current_position)
            if diff < self._EPSILON_OPENING_DIFF_:
                result.reached_goal = True
                break
            
            # ...or when progress stagnates, probably signaling that the gripper is exerting max effort and not moving
            progress.append(round(diff, 3))  # round to millimeter to neglect tiny motions of the stalled gripper
            if len(progress) == progress.maxlen and progress.count(progress[0]) == len(progress):
                if progress[0] == diff_at_start:
                    # we didn't move at all! is the gripper connected?
                    self.server.set_aborted()
                    rospy.logerr('Gripper Controller: gripper not moving; is the servo connected?')
                    return

                # buffer full with all-equal positions -> gripper stalled
                result.stalled = True
                break
            
            # publish feedback
            feedback.position = self.current_position
            self.server.publish_feedback(feedback)

        result.position = self.current_position
        self.server.set_succeeded(result)
        rospy.loginfo('Gripper Controller: Succeeded.')


    def stateCb(self, joint_states):
        try:
            index = joint_states.name.index(self.model.joint)
            angle = joint_states.position[index]
        except ValueError:
            # no problem; probably a joint states message unrelated to the gripper
            return
        
        self.current_position = self.model.getPosition(angle)
        
        # notice the action server goal callback that new data is available
        self.state_cb_event.set()
        self.state_cb_event.clear()
        
        self.state_cb_times.append(rospy.get_rostime().to_sec())


if __name__=='__main__':
    try:
        rospy.logwarn("Please use gripper_controller (no .py)")
        GripperActionController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Hasta la Vista...')
