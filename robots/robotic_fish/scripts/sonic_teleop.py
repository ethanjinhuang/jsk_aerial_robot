#!/usr/bin/env python

from __future__ import print_function # for print function in python2

import rospy
import math

from spinal.msg import ServoControlCmd
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.servo_center = rospy.get_param('~servo_center', 2048.0)
        self.servo_range = rospy.get_param('~servo_range', 1024.0)
        self.servo_cmd = rospy.get_param('~servo_cmd', [self.servo_center])
        self.servo_axis = 0.0
        self.control_mode = 'manual'
        self.auto_frequency = rospy.get_param('~auto_frequency', 2.0)
        self.auto_default_amplitude = rospy.get_param('~auto_default_amplitude', 512.0)
        self.auto_amplitude = rospy.get_param('~auto_amplitude', self.auto_default_amplitude)
        self.auto_amplitude_rate = rospy.get_param('~auto_amplitude_rate', 64.0)
        self.auto_start_time = rospy.Time.now()
        self.last_joy_time = rospy.Time.now()
        self.auto_button_prev = 0

        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self._controlLoop)

    def set_servo_target(self, target):
        self.servo_cmd[0] = self.clamp_servo_target(target)

    def clamp_servo_target(self, target):
        upper_limit = self.servo_center + self.servo_range
        lower_limit = self.servo_center - self.servo_range
        if target > upper_limit:
            target = upper_limit
        if target < lower_limit:
            target = lower_limit
        return target

    def get_center_target(self):
        return self.clamp_servo_target(self.servo_center - self.servo_axis * self.servo_range)

    def manual_control(self):
        self.set_servo_target(self.get_center_target())

    def auto_control(self):
        center_target = self.get_center_target()
        upper_margin = self.servo_center + self.servo_range - center_target
        lower_margin = center_target - (self.servo_center - self.servo_range)
        amplitude = min(self.auto_amplitude, upper_margin, lower_margin)
        t = (rospy.Time.now() - self.auto_start_time).to_sec()
        offset = amplitude * math.sin(2.0 * math.pi * self.auto_frequency * t)
        self.set_servo_target(center_target + offset)

    def start_auto(self):
        if self.auto_amplitude <= 0:
            self.auto_amplitude = self.auto_default_amplitude
        if self.control_mode != 'auto':
            rospy.loginfo("start sonic auto mode, amplitude = %s", self.auto_amplitude)
        self.control_mode = 'auto'
        self.auto_start_time = rospy.Time.now()

    def stop_control(self):
        self.control_mode = 'manual'
        self.set_servo_target(self.servo_center)

    def publish_cmd(self):
        servo_msg = ServoControlCmd()
        servo_msg.index = [0]
        servo_msg.cmd.append(int(self.servo_cmd[0]))
        self.servo_cmd_pub.publish(servo_msg)

    def _controlLoop(self, event):
        if self.control_mode == 'auto':
            self.auto_control()
            self.publish_cmd()

    def _joyCallback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_joy_time).to_sec()
        self.last_joy_time = now

        if msg.buttons[1] == 1:
            self.stop_control()
            self.publish_cmd()
            return

        self.servo_axis = msg.axes[2]
        if abs(self.servo_axis) < self.joy_dead_zone:
            self.servo_axis = 0

        if msg.buttons[3] == 1 and self.auto_button_prev == 0:
            self.start_auto()
        self.auto_button_prev = msg.buttons[3]

        if msg.buttons[6] == 1 or msg.buttons[7] == 1:
            amp_delta = 0.5 * (1.0 - msg.axes[4]) - 0.5 * (1.0 - msg.axes[3])
            if abs(amp_delta) < self.joy_dead_zone:
                amp_delta = 0
            self.auto_amplitude += amp_delta * self.auto_amplitude_rate * dt
            if self.auto_amplitude < 0:
                self.auto_amplitude = 0
            if self.auto_amplitude > self.servo_range:
                self.auto_amplitude = self.servo_range

        if self.control_mode == 'manual':
            self.manual_control()
            self.publish_cmd()



if __name__=="__main__":

    rospy.init_node("teleop")

    teleop_node = Teleop()

    rospy.spin()
