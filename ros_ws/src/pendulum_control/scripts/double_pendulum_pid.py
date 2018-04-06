#!/usr/bin/env python
import double_pendulum_controller as dpc
import numpy as np
import rospy

class DoublePendulumPID(dpc.DoublePendulumController):

    def __init__(self):

        dpc.DoublePendulumController.__init__(self)

        self.kp = 10.0
        self.kd = 5.0
        self.ki = 7.0

        self.integrator1 = 0.0
        self.integrator2 = 0.0


    def find_torques(self, x, x_cmd):

        error1 = x_cmd[0]-x[0]
        error2 = x_cmd[2]-x[2]

        self.integrator1 += self.timestep*error1
        self.integrator2 += self.timestep*error2

        torque1 = self.kp*error1 - self.kd*x[1] + self.ki*self.integrator1
        torque2 = self.kp*error2 - self.kd*x[3] + self.ki*self.integrator2

        torques = [torque1, torque2]


        print torques

        return torques

if __name__ == "__main__":

    controller = DoublePendulumPID()

    rospy.init_node('joint_controller', anonymous=True)

    rate = rospy.Rate(controller.rate)

    while not rospy.is_shutdown():

        # controller.just_do_it()
        controller.run()

        rate.sleep()










