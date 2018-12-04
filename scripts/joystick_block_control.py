#!/usr/bin/python

import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *

class JoystickBlockControl(object):

    def __init__(self):
        rospy.init_node("joystick_block_control")
        self.last_joystick_state = None
        self.last_control_mode = None
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystick_cb, queue_size=1)
        self.thruster_pub = rospy.Publisher("/flying_box/thruster/command", WrenchStamped, queue_size=1)
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            if self.last_joystick_state is not None:
                x_axis_val = self.last_joystick_state.axes[1]
                y_axis_val = self.last_joystick_state.axes[0]
                l_trigger = (self.last_joystick_state.axes[2] - 1.0) * -0.5
                r_trigger = (self.last_joystick_state.axes[5] - 1.0) * -0.5
                z_axis_val = l_trigger - r_trigger
                x_button = bool(self.last_joystick_state.buttons[2])
                y_button = bool(self.last_joystick_state.buttons[3])
                wrench_command = WrenchStamped();
                wrench_command.header.frame_id = "flying_box"
                if x_button and y_button:
                    rospy.logerr("Do not press X and Y at the same time")
                    wrench_command.wrench.force.x = 0.0
                    wrench_command.wrench.force.y = 0.0
                    wrench_command.wrench.force.z = 0.0
                    wrench_command.wrench.torque.x = 0.0
                    wrench_command.wrench.torque.y = 0.0
                    wrench_command.wrench.torque.z = 0.0
                elif x_button:
                    if self.last_control_mode is not "TRANSLATE":
                        rospy.loginfo("Switched to translation")
                        self.last_control_mode = "TRANSLATE"
                    wrench_command.wrench.force.x = x_axis_val * 10.0
                    wrench_command.wrench.force.y = y_axis_val * 10.0
                    wrench_command.wrench.force.z = z_axis_val * 10.0
                    wrench_command.wrench.torque.x = 0.0
                    wrench_command.wrench.torque.y = 0.0
                    wrench_command.wrench.torque.z = 0.0
                elif y_button:
                    if self.last_control_mode is not "ROTATE":
                        rospy.loginfo("Switched to rotation")
                        self.last_control_mode = "ROTATE"
                    wrench_command.wrench.force.x = 0.0
                    wrench_command.wrench.force.y = 0.0
                    wrench_command.wrench.force.z = 0.0
                    wrench_command.wrench.torque.x = x_axis_val
                    wrench_command.wrench.torque.y = y_axis_val
                    wrench_command.wrench.torque.z = z_axis_val
                else:
                    if self.last_control_mode is not None:
                        rospy.loginfo("Switched to idle")
                        self.last_control_mode = None
                    wrench_command.wrench.force.x = 0.0
                    wrench_command.wrench.force.y = 0.0
                    wrench_command.wrench.force.z = 0.0
                    wrench_command.wrench.torque.x = 0.0
                    wrench_command.wrench.torque.y = 0.0
                    wrench_command.wrench.torque.z = 0.0
                self.thruster_pub.publish(wrench_command)
            rate.sleep()

    def joystick_cb(self, msg):
        self.last_joystick_state = msg

if __name__ == '__main__':
    JoystickBlockControl()
