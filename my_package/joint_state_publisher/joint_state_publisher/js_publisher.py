#!/usr/bin/env python
# Lucas Walter
# make a joint exactly what the command wants it to be- this only works
# for position control.

import rclpy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class CommandToJointState:
    def __init__(self):
        self.joint_name = rclpy.get_param("~joint_name")
        self.joint_state = JointState()
        self.joint_state.name.append(self.joint_name)
        self.joint_state.position.append(0.0)
        self.joint_state.velocity.append(0.0)
        self.joint_pub = rclpy.Publisher("joint_states", JointState, queue_size=1)
        self.command_sub = rclpy.Subscriber(
            "command", Float64, self.command_callback, queue_size=1
        )

    def command_callback(self, msg):
        self.joint_state.position[0] = msg.data
        self.joint_state.header.stamp = rclpy.Time.now()
        self.joint_pub.publish(self.joint_state)


if __name__ == "__main__":
    rclpy.init_node("command_to_joint_state")
    command_to_joint_state = CommandToJointState()
    rclpy.spin()
