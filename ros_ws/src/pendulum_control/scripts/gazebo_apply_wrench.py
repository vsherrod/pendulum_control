#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point
from geometry_msgs.msg import Wrench


class GazeboApplyWrench:
    def __init__(self, service_name, body_name, reference_frame):

        self.apply_wrench = rospy.ServiceProxy(service_name, ApplyBodyWrench)

        self.body_name = body_name
        self.reference_frame = reference_frame

    def wrench_time(self, point, wrench, duration):

        self.apply_wrench(self.body_name, self.reference_frame, point, wrench, rospy.Time.now(), duration)

if __name__ == "__main__":

    rospy.init_node('disturber', anonymous=True)

    duration = rospy.Duration.from_sec(1/100.0)

    smacker = GazeboApplyWrench("/gazebo/apply_body_wrench", "link3", "link3")

    point = Point()
    point.x = 0.0
    point.y = 0.0
    point.z = 1.0

    wrench = Wrench()
    wrench.force.x = 500.0
    wrench.force.y = 0.0
    wrench.force.z = 0.0
    wrench.torque.x = 0.0
    wrench.torque.y = 0.0
    wrench.torque.z = 0.0

    smacker.wrench_time(point, wrench, duration)

    print "potatoes"
