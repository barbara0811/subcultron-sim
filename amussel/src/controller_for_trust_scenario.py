#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario -- aMussels only monitor currents and send measurement
data to aFishes in the communication range.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sqrt, pow, fabs
import action_library

class ScenarioController(object):
    
    def __init__(self):

        self.current = NED(0, 0, 0)
        self.position = None

        rospy.Subscriber("current_sensor", TwistStamped, self.current_sensor_cb)
        
        rospy.spin()
    
    def current_sensor_cb(self, msg):

        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
