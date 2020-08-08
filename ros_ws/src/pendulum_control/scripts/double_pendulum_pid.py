#!/usr/bin/env python
import double_pendulum_controller as dpc
import numpy as np
import rospy

class DoublePendulumPID(dpc.DoublePendulumController):

    def __init__(self, kp, ki, kd):

        dpc.DoublePendulumController.__init__(self)

        self.kp = kp
        self.kd = kd
        self.ki = ki

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


        if torque1 > 100.0 or torque2 > 100.0:
            print torques

        return torques

if __name__ == "__main__":

    # kp = 10.0
    # kd = 5.0
    # ki = 7.0

    kp = 10.0
    kd = 3.0
    ki = 0.0

    controller = DoublePendulumPID(kp, ki, kd)

    rospy.init_node('joint_controller', anonymous=True)

    rate = rospy.Rate(controller.rate)

    while not rospy.is_shutdown():

        # controller.just_do_it()
        controller.run()

        rate.sleep()










