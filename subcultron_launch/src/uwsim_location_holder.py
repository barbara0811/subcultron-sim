#!/usr/bin/python
"""
"""

__author__ = "barbanas"
from geometry_msgs.msg import Point
from auv_msgs.msg import NavSts
from misc_msgs.srv import GetTrustInfo
<<<<<<< HEAD
from misc_msgs.msg import ConnMatrix
=======
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
import numpy as np
from math import sqrt, pow
import rospy

class UWSimLocationHolder(object):

    def __init__(self):

        self.communicationRange = 5

<<<<<<< HEAD
        self.connMatrix = np.zeros([8,8])
        self.positions = []
        for i in range(8):
=======
        self.connMatrix = np.zeros([15,15])
        self.positions = []
        for i in range(15):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
            self.positions.append(Point())

        self.afish1locSub = rospy.Subscriber('/afish1/position', NavSts, self.afish1position_cb)
        self.afish2locSub = rospy.Subscriber('/afish2/position', NavSts, self.afish2position_cb)
        self.afish3locSub = rospy.Subscriber('/afish3/position', NavSts, self.afish3position_cb)
<<<<<<< HEAD
=======
        self.afish4locSub = rospy.Subscriber('/afish4/position', NavSts, self.afish4position_cb)
        self.afish5locSub = rospy.Subscriber('/afish5/position', NavSts, self.afish5position_cb)
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace

        self.amussel1locSub = rospy.Subscriber('/amussel1/position', NavSts, self.amussel1position_cb)
        self.amussel2locSub = rospy.Subscriber('/amussel2/position', NavSts, self.amussel2position_cb)
        self.amussel3locSub = rospy.Subscriber('/amussel3/position', NavSts, self.amussel3position_cb)
        self.amussel4locSub = rospy.Subscriber('/amussel4/position', NavSts, self.amussel4position_cb)
        self.amussel5locSub = rospy.Subscriber('/amussel5/position', NavSts, self.amussel5position_cb)
<<<<<<< HEAD

        self.APub = rospy.Publisher('/conn_matrix', ConnMatrix, queue_size = 1)
        rospy.Timer(rospy.Duration(0.1), self.calculate_connectivity_matrix)
=======
        self.amussel6locSub = rospy.Subscriber('/amussel6/position', NavSts, self.amussel6position_cb)
        self.amussel7locSub = rospy.Subscriber('/amussel7/position', NavSts, self.amussel7position_cb)
        self.amussel8locSub = rospy.Subscriber('/amussel8/position', NavSts, self.amussel8position_cb)
        self.amussel9locSub = rospy.Subscriber('/amussel9/position', NavSts, self.amussel9position_cb)
        self.amussel10locSub = rospy.Subscriber('/amussel10/position', NavSts, self.amussel10position_cb)

        rospy.Timer(rospy.Duration(0.2), self.calculate_connectivity_matrix)
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace

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

<<<<<<< HEAD

    def amussel1position_cb(self, msg):
=======
    def afish4position_cb(self, msg):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
        self.positions[3].x = msg.position.north
        self.positions[3].y = msg.position.east
        self.positions[3].z = msg.position.depth

<<<<<<< HEAD
    def amussel2position_cb(self, msg):
=======
    def afish5position_cb(self, msg):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
        self.positions[4].x = msg.position.north
        self.positions[4].y = msg.position.east
        self.positions[4].z = msg.position.depth

<<<<<<< HEAD
    def amussel3position_cb(self, msg):
=======

    def amussel1position_cb(self, msg):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
        self.positions[5].x = msg.position.north
        self.positions[5].y = msg.position.east
        self.positions[5].z = msg.position.depth

<<<<<<< HEAD
    def amussel4position_cb(self, msg):
=======
    def amussel2position_cb(self, msg):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
        self.positions[6].x = msg.position.north
        self.positions[6].y = msg.position.east
        self.positions[6].z = msg.position.depth

<<<<<<< HEAD
    def amussel5position_cb(self, msg):
=======
    def amussel3position_cb(self, msg):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
        self.positions[7].x = msg.position.north
        self.positions[7].y = msg.position.east
        self.positions[7].z = msg.position.depth

<<<<<<< HEAD

    def calculate_connectivity_matrix(self, event):
        for i in range(8):
            for j in range(i + 1, 8):
=======
    def amussel4position_cb(self, msg):
        self.positions[8].x = msg.position.north
        self.positions[8].y = msg.position.east
        self.positions[8].z = msg.position.depth

    def amussel5position_cb(self, msg):
        self.positions[9].x = msg.position.north
        self.positions[9].y = msg.position.east
        self.positions[9].z = msg.position.depth

    def amussel6position_cb(self, msg):
        self.positions[10].x = msg.position.north
        self.positions[10].y = msg.position.east
        self.positions[10].z = msg.position.depth

    def amussel7position_cb(self, msg):
        self.positions[11].x = msg.position.north
        self.positions[11].y = msg.position.east
        self.positions[11].z = msg.position.depth

    def amussel8position_cb(self, msg):
        self.positions[12].x = msg.position.north
        self.positions[12].y = msg.position.east
        self.positions[12].z = msg.position.depth

    def amussel9position_cb(self, msg):
        self.positions[13].x = msg.position.north
        self.positions[13].y = msg.position.east
        self.positions[13].z = msg.position.depth

    def amussel10position_cb(self, msg):
        self.positions[14].x = msg.position.north
        self.positions[14].y = msg.position.east
        self.positions[14].z = msg.position.depth


    def calculate_connectivity_matrix(self, event):
        for i in range(15):
            for j in range(i + 1, 15):
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace
                if self.distance(self.positions[i], self.positions[j]) <= self.communicationRange:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 1
                else:
                    self.connMatrix[i][j] = self.connMatrix[j][i] = 0
<<<<<<< HEAD
        T = []
        for i in range(3):
            T.extend(self.connMatrix[i][:3])
        self.APub.publish(T)
=======
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace

    def distance(self, p1, p2):
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2))

# service functions
    def get_conn_vectors_srv(self, req):
<<<<<<< HEAD
        if req.index >= 3:
            return {}
        return {'A': self.connMatrix[req.index][:3], 'b': self.connMatrix[req.index][3:] }
=======
        if req.index >= 5:
            return {}
        return {'A': self.connMatrix[req.index][:5], 'b': self.connMatrix[req.index][5:] }
>>>>>>> 55f23a50914aac6ad187d99d9422c03d1fc2eace

if __name__ == "__main__":
    rospy.init_node("uwsim_location_holder")
    try:
        controller = UWSimLocationHolder()
    except rospy.ROSInterruptException:
        pass