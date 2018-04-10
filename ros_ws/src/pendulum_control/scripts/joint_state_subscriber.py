from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import rospy
import threading
import pdb
import copy

class JointStateSubscriber:
    def __init__(self, topic_name = "/joint_states"):

        rospy.Subscriber(topic_name, JointState, self.callback)
        self.header_ = Header()
        self.header = Header()
        self.names_ = []
        self.names = []
        self.positions_ = []
        self.positions = []
        self.velocities_ = []
        self.velocities = []
        self.efforts_ = []
        self.efforts = []
        self.position = 0.0
        self.velocity = 0.0
        self.effort = 0.0

        self.lock = threading.Lock()

    def callback(self, msg):

        self.lock.acquire()
        try:
            self.header_ = msg.header
            self.names_ = msg.name
            self.positions_ = msg.position
            self.velocities_ = msg.velocity
            self.efforts_ = msg.effort
        finally:
            self.lock.release()


    def getData(self):
        self.lock.acquire()
        try:
            self.header = copy.deepcopy(self.header_)
            self.names = copy.deepcopy(self.names_)
            self.positions = copy.deepcopy(self.positions_)
            self.velocities = copy.deepcopy(self.velocities_)
            self.efforts = copy.deepcopy(self.efforts_)

        finally:
            self.lock.release()

        return self.header, self.names, self.positions, self.velocities, self.efforts

    def getDataJoint(self, joint):
        self.lock.acquire()
        if joint in self.names_:
            idx = self.names_.index(joint)
            try:
                self.header = copy.deepcopy(self.header_)
                if len(self.positions_) > idx:
                    self.position = self.positions_[idx]
                if len(self.velocities_) > idx:
                    self.velocity = self.velocities_[idx]
                if len(self.efforts_) > idx:
                    self.effort = self.efforts_[idx]

            finally:
                self.lock.release()
        else:
            print "Joint not in joint states"
            self.lock.release()

if __name__ == "__main__":

    rospy.init_node('real_joint_state_subscriber', anonymous=True)

    subscriber = JointStateSubscriber()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        header, names, positions, velocities, efforts = subscriber.getData()

        # print names
        # print positions
        # print velocities
        # print efforts

        subscriber.getDataJoint("joint1")

        print subscriber.position
        print subscriber.velocity
        print subscriber.effort

        rate.sleep()










