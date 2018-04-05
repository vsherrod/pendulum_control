#!/usr/bin/env python
import double_pendulum_controller as dpc
import double_pendulum_mpc_solver as dpms
import numpy as np
import rospy
import scipy.signal as ss

class DoublePendulumMPC(dpc.DoublePendulumController):

    def __init__(self):

        dpc.DoublePendulumController.__init__(self)

        self.g = 9.81

        self.l1 = 1.0
        self.l2 = 1.0

        self.w1 = 0.1
        self.w2 = 0.1

        self.m1 = 1.0
        self.m2 = 1.0

        self.b1 = 0.7
        self.b2 = 0.7

        self.I1 = 1.0/12.0*self.m1*(5.0/4.0*self.l1**2 + self.w1**2)
        self.I2 = 1.0/12.0*self.m2*(5.0/4.0*self.l2**2 + self.w2**2)

        self.Q1 = [10.0]
        self.Q2 = [10.0]
        self.Q3 = [0.06, 0.0, 0.0, 0.06]

        self.R1 = [1000.0]
        self.R2 = [1000.0]

        self.tau_prev = [0.0, 0.0]

        self.C = np.matrix([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])

        self.D = np.matrix([[0.0, 0.0],
                            [0.0, 0.0],
                            [0.0, 0.0],
                            [0.0, 0.0]])


    def just_do_it(self):

        x = [0.0, 0.0, 0.0, 0.0]

        x_cmd = [np.pi/4.0, 0.0, -np.pi/4.0, 0.0]

        A = np.matrix([[0.0, 1.0, 0.0, 0.0],
                      [-self.m1*self.g*self.l1/(2*self.I1)*np.cos(x[0]) - self.m2*self.g*self.l1/self.I1*np.cos(x[0]) - self.m2*self.g*self.l2/(2.0*self.I1)*np.cos(x[0] + x[2]), -self.b1/self.I1, -self.m2*self.g*self.l2/(2.0*self.I1)*np.cos(x[0] + x[2]), 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [-self.m2*self.g*self.l2/(2.0*self.I2)*np.cos(x[0] + x[2]), 0.0, -self.m2*self.g*self.l2/(2.0*self.I2)*np.cos(x[0]+x[2]), -self.b2/self.I2]])

        B = np.matrix([[0.0, 0.0],
                      [1.0/self.I1, 0.0],
                      [0.0, 0.0],
                      [0.0, 1.0/self.I2]])

        sysd = ss.cont2discrete((A,B,self.C,self.D), self.timestep)

        Ad = sysd[0].T.flatten('F').tolist()
        Bd = sysd[1].T.flatten('F').tolist()

        print self.tau_prev

        torques = dpms.runController(Ad, Bd, self.Q1, self.Q2, self.Q3, self.R1, self.R2, x, x_cmd, self.tau_prev)

        print torques



    def find_torques(self, x, x_cmd):

        A = np.matrix([[0.0, 1.0, 0.0, 0.0],
                      [-self.m1*self.g*self.l1/(2*self.I1)*np.cos(x[0]) - self.m2*self.g*self.l1/self.I1*np.cos(x[0]) - self.m2*self.g*self.l2/(2.0*self.I1)*np.cos(x[0] + x[2]), -self.b1/self.I1, -self.m2*self.g*self.l2/(2.0*self.I1)*np.cos(x[0] + x[2]), 0.0],
                      [0.0, 0.0, 0.0, 1.0],
                      [-self.m2*self.g*self.l2/(2.0*self.I2)*np.cos(x[0] + x[2]), 0.0, -self.m2*self.g*self.l2/(2.0*self.I2)*np.cos(x[0]+x[2]), -self.b2/self.I2]])

        B = np.matrix([[0.0, 0.0],
                      [1.0/self.I1, 0.0],
                      [0.0, 0.0],
                      [0.0, 1.0/self.I2]])

        sysd = ss.cont2discrete((A,B,self.C,self.D), self.timestep)

        Ad = sysd[0].flatten('F').tolist()
        Bd = sysd[1].flatten('F').tolist()

        torques = dpms.runController(Ad, Bd, self.Q1, self.Q2, self.Q3, self.R1, self.R2, x, x_cmd, self.tau_prev)

        # self.torques_prev = [torques[0][0], torques[1][0]]

        kp = 10.0
        kd = 5.0

        self.torques_prev = [kp*(x_cmd[0]-x[0]) - kd*x[1], kp*(x_cmd[2]-x[2]) - kd*x[3]]

        print torques

        return self.torques_prev

if __name__ == "__main__":

    controller = DoublePendulumMPC()

    rospy.init_node('joint_controller', anonymous=True)

    rate = rospy.Rate(controller.rate)

    while not rospy.is_shutdown():

        # controller.just_do_it()
        controller.run()

        rate.sleep()










