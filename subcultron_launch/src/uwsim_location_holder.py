#!/usr/bin/python
"""
"""

__author__ = "barbanas"
from geometry_msgs.msg import Point
from auv_msgs.msg import NavSts
from misc_msgs.srv import GetTrustInfo
import numpy as np
from math import sqrt, pow
import rospy

class UWSimLocationHolder(object):

    def __init__(self):

        self.communicationRange = 5

        self.connMatrix = np.zeros([2,2])
        self.positions = []
        for i in range(2):
            self.positions.append(Point())

        self.afish1locSub = rospy.Subscriber('/afish1/position', NavSts, self.afish1position_cb)

        self.amussel1locSub = rospy.Subscriber('/amussel1/position', NavSts, self.amussel1position_cb)

        rospy.Timer(rospy.Duration(0.2), self.calculate_connectivity_matrix)

        rospy.Service('get_connectivity_vectors', GetTrustInfo, self.get_conn_vectors_srv)

        rospy.spin()

    def afish1position_cb(self, msg):
        self.positions[0].x = msg.position.north
        self.positions[0].y = msg.position.east
        self.positions[0].z = msg.position.depth


    def amussel1position_cb(self, msg):
        self.positions[1].x = msg.position.north
        self.positions[1].y = msg.position.east
        self.positions[1].z = msg.position.depth


    def calculate_connectivity_matrix(self, event):
        for i in range(2):
            for j in range(i + 1, 2):
                if self.distance(self.positions[i], self.positions[j]) <= self.communicationRange:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 1
                else:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 0

    def distance(self, p1, p2):
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))

# service functions
    def get_conn_vectors_srv(self, req):
        if req.index >= 1:
            return {}
        return {'A': self.connMatrix[req.index][:1], 'b': self.connMatrix[req.index][1:] }

if __name__ == "__main__":
    rospy.init_node("uwsim_location_holder")
    try:
        controller = UWSimLocationHolder()
    except rospy.ROSInterruptException:
        pass