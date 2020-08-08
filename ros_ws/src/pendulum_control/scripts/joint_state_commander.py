#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState

def populate_joint_state_msg(joint_state_msg, get_joint_data, name):
        joint_data = get_joint_data(name)

        if joint_data.success == True:
            joint_state_msg.name.append(name)
            joint_state_msg.position.append(joint_data.position[0])
            joint_state_msg.velocity.append(joint_data.rate[0])
        else:
            print "No data for " + name

if __name__ == "__main__":

    joint_state_pub = rospy.Publisher('joint_angle_cmds', JointState, queue_size = 1)

    rospy.init_node('commander', anonymous=True)

    rate = rospy.Rate(100)

    joint1_cmds = np.linspace(-np.pi, np.pi, 5)
    joint2_cmds = np.linspace(-3.0*np.pi/2.0, -np.pi/2.0, 5)


    i = 0

    ascending = True

    start_time = rospy.Time.now()
    first_time = True

    while not rospy.is_shutdown():

        if first_time:
            first_time = False
            start_time = rospy.Time.now()

        cur_time = rospy.Time.now()

        print "start time: ", start_time.to_sec()
        print "cur time: ", cur_time.to_sec()

        change_time = cur_time.to_sec() - start_time.to_sec()

        print change_time

        if change_time > 4.0:
            start_time = cur_time
            if ascending:
                if i == len(joint1_cmds)-1:
                    i -= 1
                    ascending = False
                else:
                    i += 1
            else:
                if i == 0:
                    i += 1
                    ascending = True
                else:
                    i -= 1


        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        joint_state_msg.name.append("joint1")
        joint_state_msg.name.append("joint2")

        joint_state_msg.position.append(joint1_cmds[i])
        joint_state_msg.position.append(joint2_cmds[i])


        joint_state_pub.publish(joint_state_msg)

        rate.sleep()

