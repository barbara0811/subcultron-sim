#!/usr/bin/python
"""
"""

__author__ = "barbanas"
from geometry_msgs.msg import Point
from auv_msgs.msg import NavSts
from misc_msgs.srv import GetTrustInfo
from misc_msgs.msg import ConnMatrix
import numpy as np
from math import sqrt, pow
import rospy

class UWSimLocationHolder(object):

    def __init__(self):

        self.communicationRange = 5

        self.connMatrix = np.zeros([8,8])
        self.positions = []
        for i in range(8):
            self.positions.append(Point())

        self.afish1locSub = rospy.Subscriber('/afish1/position', NavSts, self.afish1position_cb)
        self.afish2locSub = rospy.Subscriber('/afish2/position', NavSts, self.afish2position_cb)
        self.afish3locSub = rospy.Subscriber('/afish3/position', NavSts, self.afish3position_cb)

        self.amussel1locSub = rospy.Subscriber('/amussel1/position', NavSts, self.amussel1position_cb)
        self.amussel2locSub = rospy.Subscriber('/amussel2/position', NavSts, self.amussel2position_cb)
        self.amussel3locSub = rospy.Subscriber('/amussel3/position', NavSts, self.amussel3position_cb)
        self.amussel4locSub = rospy.Subscriber('/amussel4/position', NavSts, self.amussel4position_cb)
        self.amussel5locSub = rospy.Subscriber('/amussel5/position', NavSts, self.amussel5position_cb)

        self.APub = rospy.Publisher('/conn_matrix', ConnMatrix, queue_size = 1)
        rospy.Timer(rospy.Duration(0.1), self.calculate_connectivity_matrix)

        rospy.Service('get_connectivity_vectors', GetTrustInfo, self.get_conn_vectors_srv)

        rospy.spin()

    def afish1position_cb(self, msg):
        self.positions[0].x = msg.position.north
        self.positions[0].y = msg.position.east
        self.positions[0].z = msg.position.depth

    def afish2position_cb(self, msg):
        self.positions[1].x = msg.position.north
        self.positions[1].y = msg.position.east
        self.positions[1].z = msg.position.depth

    def afish3position_cb(self, msg):
        self.positions[2].x = msg.position.north
        self.positions[2].y = msg.position.east
        self.positions[2].z = msg.position.depth


    def amussel1position_cb(self, msg):
        self.positions[3].x = msg.position.north
        self.positions[3].y = msg.position.east
        self.positions[3].z = msg.position.depth

    def amussel2position_cb(self, msg):
        self.positions[4].x = msg.position.north
        self.positions[4].y = msg.position.east
        self.positions[4].z = msg.position.depth

    def amussel3position_cb(self, msg):
        self.positions[5].x = msg.position.north
        self.positions[5].y = msg.position.east
        self.positions[5].z = msg.position.depth

    def amussel4position_cb(self, msg):
        self.positions[6].x = msg.position.north
        self.positions[6].y = msg.position.east
        self.positions[6].z = msg.position.depth

    def amussel5position_cb(self, msg):
        self.positions[7].x = msg.position.north
        self.positions[7].y = msg.position.east
        self.positions[7].z = msg.position.depth


    def calculate_connectivity_matrix(self, event):
        for i in range(8):
            for j in range(i + 1, 8):
                if self.distance(self.positions[i], self.positions[j]) <= self.communicationRange:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 1
                else:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 0
        T = []
        for i in range(3):
            T.extend(self.connMatrix[i][:3])
        self.APub.publish(T)

    def distance(self, p1, p2):
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))

# service functions
    def get_conn_vectors_srv(self, req):
        if req.index >= 3:
            return {}
        return {'A': self.connMatrix[req.index][:3], 'b': self.connMatrix[req.index][3:] }

if __name__ == "__main__":
    rospy.init_node("uwsim_location_holder")
    try:
        controller = UWSimLocationHolder()
    except rospy.ROSInterruptException:
        pass