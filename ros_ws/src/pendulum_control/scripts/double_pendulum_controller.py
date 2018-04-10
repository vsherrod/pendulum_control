#!/usr/bin/env python
from abc import ABCMeta, abstractmethod
import gazebo_torque_commander as gtc
import joint_state_subscriber as jss
import numpy as np
import rospy

class DoublePendulumController():
    __metaclass__ = ABCMeta

    def __init__(self):

        self.joints = ["joint1", "joint2"]

        self.cmd_topic = "/joint_angle_cmds"
        self.torque_service_name = "/gazebo/apply_joint_effort"

        self.rate = 50.0
        self.timestep = 1.0/self.rate

        self.torque_1_cmder = gtc.GazeboTorqueCommander(self.torque_service_name, "joint1", rospy.Duration.from_sec(self.timestep))
        self.torque_2_cmder = gtc.GazeboTorqueCommander(self.torque_service_name, "joint2", rospy.Duration.from_sec(self.timestep))

        self.joint_sub = jss.JointStateSubscriber()
        self.joint_cmd_sub = jss.JointStateSubscriber("/joint_angle_cmds")

        self.x_cmd = [np.pi/4.0, 0.0, 0.0, 0.0]

    def run(self):

        x = []
        i = 0

        for joint in self.joints:

            self.joint_sub.getDataJoint(joint)
            self.joint_cmd_sub.getDataJoint(joint)

            x.append(self.joint_sub.position)
            x.append(self.joint_sub.velocity)
            self.x_cmd[2*i] = self.joint_cmd_sub.position
            self.x_cmd[2*i+1] = self.joint_cmd_sub.velocity

            i += 1

        #zeroing out the estimates
        x[0] = x[0] + np.pi
        x[2] = x[2] - 2.0*np.pi

        torques = self.find_torques(x, self.x_cmd)

        self.torque_1_cmder.apply_torque_cmd(torques[0])
        self.torque_2_cmder.apply_torque_cmd(torques[1])

    @abstractmethod
    def find_torques(self, x, x_cmd):
        pass









