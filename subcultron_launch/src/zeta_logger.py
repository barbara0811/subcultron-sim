#!/usr/bin/python
"""
"""

__author__ = "barbanas"
from misc_msgs.msg import zeta
from std_msgs.msg import Bool
import numpy as np
from math import sqrt, pow
import rospy

### PARAMETERS ###

user = ""
noiseActivationRate = 0.0

area = [30, 30]

n_fish = 5
n_mussel = 10

##########################

class ZetaLogger(object):

    def __init__(self):

        self.zeta = [[] for i in range(5)]
        self.start = False
        self.zetas = set()

        self.afish1zetaSub = rospy.Subscriber('/afish1/zeta', zeta, self.afish1zeta_cb)
        self.afish2zetaSub = rospy.Subscriber('/afish2/zeta', zeta, self.afish2zeta_cb)
        self.afish3zetaSub = rospy.Subscriber('/afish3/zeta', zeta, self.afish3zeta_cb)
        self.afish4zetaSub = rospy.Subscriber('/afish4/zeta', zeta, self.afish4zeta_cb)
        self.afish5zetaSub = rospy.Subscriber('/afish5/zeta', zeta, self.afish5zeta_cb)

        self.logs_folder = '/home/' + user + '/Desktop/logs_trust/' + str(n_fish) + 'fsh_' + str(n_mussel) + 'mss_' + str(noiseActivationRate) + 'ns_' + str(area[0]) + 'x' + str(area[1]) + '/'

        file = open(self.logs_folder + 'zeta.txt','w')

        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Timer(rospy.Duration(0.1), self.save_zeta)

        rospy.spin()

    def afish1zeta_cb(self, msg):
        self.zeta[0] = msg.zeta
        self.zetas.add(0)

    def afish2zeta_cb(self, msg):
        self.zeta[1] = msg.zeta
        self.zetas.add(1)

    def afish3zeta_cb(self, msg):
        self.zeta[2] = msg.zeta
        self.zetas.add(2)

    def afish4zeta_cb(self, msg):
        self.zeta[3] = msg.zeta
        self.zetas.add(3)

    def afish5zeta_cb(self, msg):
        self.zeta[4] = msg.zeta
        self.zetas.add(4)


    def start_cb(self, msg):
        self.start = True

    def save_zeta(self, event):
        if not self.start or len(self.zetas) < 5:
            return
        file = open(self.logs_folder + 'zeta.txt','a')
        output = ""
        for i in range(5):
             output += " ".join(str(x) for x in self.zeta[i])
             output += " " 
        file.write(output + "\n")

        Zeta_ = np.zeros(len(self.zeta[0]))
        for i in range(5):
            Zeta_ += self.zeta[i]
        Zeta_ /= 5

        stdDev = np.zeros(len(self.zeta[0]))
        for i in range(5):
            stdDev += np.power(Zeta_ - self.zeta[i], 2)
        stdDev /= (5*4)
        stdDev = np.sqrt(stdDev)
        print stdDev

if __name__ == "__main__":
    rospy.init_node("zeta_logger")
    try:
        logger = ZetaLogger()
    except rospy.ROSInterruptException:
        pass