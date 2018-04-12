#!/usr/bin/env python
import double_pendulum_controller as dpc
import double_pendulum_mpc_solver as dpms
import double_pendulum_pid as dpp
import numpy as np
import rospy
import scipy.signal as ss
import pdb

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

        self.b = np.array([[self.b1, 0.0],
                           [0.0, self.b2]])

        self.I1 = 1.0/12.0*self.m1*(self.l1**2 + self.w1**2)
        self.I2 = 1.0/12.0*self.m2*(self.l2**2 + self.w2**2)

        self.Q1 = [0.002]
        self.Q2 = [0.0002] #[0.0002]
        self.Q3 = [0.001, 0.0, 0.0, 0.001]

        self.R1 = [0.0]
        self.R2 = [0.0]

        self.q_des_prev = [0.0, 0.0]

        self.C = np.array([[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])

        self.D = np.array([[0.0, 0.0],
                            [0.0, 0.0],
                            [0.0, 0.0],
                            [0.0, 0.0]])

        self.kp = np.array([[10.0, 0.0],
                             [0.0, 10.0]])

        self.kd = np.array([[5.0, 0.0],
                             [0.0, 5.0]])

        self.pid_controller = dpp.DoublePendulumPID(self.kp[0][0], 0.0, self.kd[0][0])


    def just_do_it(self):

        x = [0.0, 0.0, 0.0, 0.0]

        x_cmd = [np.pi/2.0, 0.0, np.pi/4.0, 0.0]

        Cor = np.array([[-0.5*self.m2*self.l1*self.l2*x[3]*np.sin(x[2]), -0.5*self.m2*self.l1*self.l2*np.cos(x[2])*(x[3] + x[1])],
                        [0.5*self.m2*self.l1*self.l2*x[1]*np.sin(x[2]), 0.0]])

        Tau_grav = np.array([[-1.0*(0.5*self.m1*self.l1 + self.m2*self.l1)*self.g*np.sin(x[0]) - 0.5*self.m2*self.l2*self.g*np.sin(x[0] + x[2])],
                             [-0.5*self.m2*self.l2*self.g*np.sin(x[0]+x[2])]])

        a = np.dot(self.M_inv, -1.0*Cor-self.b-self.kd)

        z = np.dot(self.M_inv, self.kp)

        print "z: ", z[0]

        A = np.array([[0.0, 1.0, 0.0, 0.0],
                      [-z[0][0], a[0][0], -z[0][1], a[0][1]],
                      [0.0, 0.0, 0.0, 1.0],
                      [-z[1][0], a[1][0], -z[1][1], a[1][1]]])

        B = np.array([[0.0, 0.0],
                      [z[0][0], z[0][1]],
                      [0.0, 0.0],
                      [z[1][0], z[1][1]]])

        sysd = ss.cont2discrete((A,B,self.C,self.D), self.timestep)

        print "a: ", a
        print "cor: ", Cor
        print "tau_grav: ", Tau_grav
        print "Ac: ", A
        print "Bc: ", B
        print "Ad: ", sysd[0]
        print "Bd: ", sysd[1]

        Ad = sysd[0].flatten('F').tolist()
        Bd = sysd[1].flatten('F').tolist()

        q_des = dpms.runController(Ad, Bd, self.Q1, self.Q2, self.Q3, self.R1, self.R2, x, x_cmd, self.q_des_prev)

        # print self.tau_prev

        # q_goal = [1.0]*4
        # q_0 = [1.0]*4
        # Q1= [1.0]
        # Q2= [1.0]
        # R1= [1.0]
        # R2= [1.0]
        # A = [1.0]*16
        # B = [1.0]*8
        # q_goal[0] = 0.20319161029830202;
        # q_goal[1] = 0.8325912904724193;
        # q_goal[2] = -0.8363810443482227;
        # q_goal[3] = 0.04331042079065206;
        # q_0[0] = 1.5717878173906188;
        # q_0[1] = 1.5851723557337523;
        # q_0[2] = -1.497658758144655;
        # q_0[3] = -1.171028487447253;
        # Q1[0] = 1.0514672033008299;
        # Q2[0] = 1.4408098436506365;
        # R1[0] = 1.0298762108785668;
        # R2[0] = 1.456833224394711;
        # A[0] = 0.596576190459043;
        # A[1] = -0.8860508694080989;
        # A[2] = 0.7050196079205251;
        # A[3] = 0.3634512696654033;
        # A[4] = -1.9040724704913385;
        # A[5] = 0.23541635196352795;
        # A[6] = -0.9629902123701384;
        # A[7] = -0.3395952119597214;
        # A[8] = -0.865899672914725;
        # A[9] = 0.7725516732519853;
        # A[10] = -0.23818512931704205;
        # A[11] = -1.372529046100147;
        # A[12] = 0.17859607212737894;
        # A[13] = 1.1212590580454682;
        # A[14] = -0.774545870495281;
        # A[15] = -1.1121684642712744;
        # B[0] = -0.44811496977740495;
        # B[1] = 1.7455345994417217;
        # B[2] = 1.9039816898917352;
        # B[3] = 0.6895347036512547;
        # B[4] = 1.6113364341535923;
        # B[5] = 1.383003485172717;
        # B[6] = -0.48802383468444344;
        # B[7] = -1.631131964513103;

        # torques = dpms.runController(A, B, Q1, Q2, R1, R2, q_0, q_goal)

        print "Cowman"
        # pdb.set_trace()

        print torques

        # pdb.set_trace()


    def find_torques(self, x, x_cmd):

        # x_cmd = [0.0, 0.0, 0.0, 0.0]

        # error1 = x_cmd[0] - x[0]
        # error2 = x_cmd[2] - x[2]

        # k_error = [1.0*error1, 0.01*error2]
        k_error = [0.0, 0.0]

        m11 = 0.25*self.m1*self.l1**2 + self.m2*(self.l1**2 + 0.25*self.l2**2 + self.l1*self.l2*np.cos(x[2])) + self.I1 + self.I2
        m12 = self.m2*(0.25*self.l2**2 + 0.5*self.l1*self.l2*np.cos(x[2])) + self.I2
        m22 = 0.25*self.m2*self.l2**2 + self.I2

        self.M = np.array([[m11, m12],
                           [m12, m22]])

        self.M_inv = np.linalg.inv(self.M)

        h = -0.5*self.m2*self.l1*self.l2*np.sin(x[2])

        Cor = np.array([[h*x[3], h*(x[3] + x[1])],
                        [-h*x[1], 0.0]])

        Tau_grav = np.array([-1.0*(0.5*self.m1*self.l1 + self.m2*self.l1)*self.g*np.sin(x[0]) - 0.5*self.m2*self.l2*self.g*np.sin(x[0] + x[2]),
                             -0.5*self.m2*self.l2*self.g*np.sin(x[0]+x[2])])

        a = np.dot(self.M_inv, -1.0*Cor-self.b-self.kd)

        z = np.dot(self.M_inv, self.kp)

        A = np.array([[0.0, 1.0, 0.0, 0.0],
                      [-z[0][0], a[0][0], -z[0][1], a[0][1]],
                      [0.0, 0.0, 0.0, 1.0],
                      [-z[1][0], a[1][0], -z[1][1], a[1][1]]])

        B = np.array([[0.0, 0.0],
                      [z[0][0], z[0][1]],
                      [0.0, 0.0],
                      [z[1][0], z[1][1]]])

        # print "x: ", x
        # print "Cor: ", Cor
        # print "Tau_grav: ", Tau_grav
        # print "A: ", A
        # print "B: ", B
        # print "C: ", self.C
        # print "D: ", self.D

        sysd = ss.cont2discrete((A,B,self.C,self.D), self.timestep)

        Ad = sysd[0].flatten('F').tolist()
        Bd = sysd[1].flatten('F').tolist()

        # print self.tau_prev

        self.q_des_prev = dpms.runController(Ad, Bd, self.Q1, self.Q2, self.Q3, self.R1, self.R2, x, x_cmd, self.q_des_prev, k_error)

        print "q_des: ", self.q_des_prev

        x_cmd_pid = [self.q_des_prev[0], 0.0, self.q_des_prev[1], 0.0]

        tau_tilde = np.array(self.pid_controller.find_torques(x, x_cmd_pid))

        torques = tau_tilde #+ Tau_grav

        return torques

if __name__ == "__main__":

    controller = DoublePendulumMPC()

    # controller.just_do_it()

    rospy.init_node('joint_controller', anonymous=True)

    rate = rospy.Rate(controller.rate)

    first_time = 1

    time_prev = rospy.Time.now()

    while not rospy.is_shutdown():

        if not first_time:
            time_now = rospy.Time.now()
            step = time_now -time_prev
            print "Hz: ", 1.0/step.to_sec()
            time_prev = time_now

        first_time = 0




        # controller.just_do_it()
        controller.run()

        rate.sleep()










