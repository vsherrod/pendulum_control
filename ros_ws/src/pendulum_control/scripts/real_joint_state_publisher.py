#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetJointProperties

def populate_joint_state_msg(joint_state_msg, get_joint_data, name):
        joint_data = get_joint_data(name)

        if joint_data.success == True:
            joint_state_msg.name.append(name)
            joint_state_msg.position.append(joint_data.position[0])
            joint_state_msg.velocity.append(joint_data.rate[0])
        else:
            print "No data for " + name

if __name__ == "__main__":

    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)

    rospy.init_node('real_joint_state_publisher', anonymous=True)

    rate = rospy.Rate(100)

    rospy.wait_for_service('/gazebo/get_joint_properties')


    while not rospy.is_shutdown():

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        try:
            get_joint_data = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)

            populate_joint_state_msg(joint_state_msg, get_joint_data, "joint1")
            populate_joint_state_msg(joint_state_msg, get_joint_data, "joint2")

            print "Publishing data for both joints"

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        #zeroing out the estimates
        if len(joint_state_msg.position) >= 2:
            # joint_state_msg.position[0] += np.pi
            joint_state_msg.position[1] -= 2.0*np.pi

        joint_state_pub.publish(joint_state_msg)

        rate.sleep()

