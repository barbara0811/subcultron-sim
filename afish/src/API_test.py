#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario -- 
"""

__author__ = "barbanas"

import rospy
import action_library

from misc_msgs.srv import GetPosition, GetSensoryReading, GetTrustInfo
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64



class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False

        self.position = None
    
        # publishers
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        

        rospy.spin()
      
    # callback functions
      
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
           
    def start_cb(self, msg):
        
        self.startPub.publish(msg)
        self.start = True

        rospy.sleep(4)
        
        # test battery module
        battery = action_library.Battery()
        print battery.get_level()
        print "################"
        
        induction = action_library.Induction()
        induction.send_energy("/amussel1/", 1)

        
                        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
