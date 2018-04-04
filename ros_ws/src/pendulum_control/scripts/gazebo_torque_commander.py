#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import ApplyJointEffort


class GazeboTorqueCommander:
    def __init__(self, service_name, joint_name, duration):

        self.apply_joint_effort = rospy.ServiceProxy(service_name, ApplyJointEffort)

        self.joint_name = joint_name
        self.duration = duration

    def apply_torque_cmd(self, torque_cmd):

        self.apply_joint_effort(self.joint_name, torque_cmd, rospy.Time.now(), self.duration)

if __name__ == "__main__":

    rospy.init_node('cmder', anonymous=True)

    duration = rospy.Duration.from_sec(1/100.0)

    cmder = GazeboTorqueCommander("/gazebo/apply_joint_effort", "joint1", duration)

    cmder.apply_torque_cmd(100.0)
