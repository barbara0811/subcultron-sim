#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped, Point
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sqrt, pow, fabs
from action_client import actionClient

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.current = NED(0, 0, 0)
        self.position = None
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('position', NavSts, self.position_cb)
        rospy.Subscriber("ping_sensor", NED, self.ping_sensor_cb)
        rospy.Subscriber("current_sensor", TwistStamped, self.current_sensor_cb)
        
        self.cl = actionClient(rospy.get_namespace())
        
        rospy.spin()

    def start_cb(self, msg):
        
        self.startPub.publish(msg)
        
        # sending position goal --> testing purposes
        p = Point(0, 0, 0)
        self.cl.send_position_goal(p)
        
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
    def ping_sensor_cb(self, msg):
        return
        if self.position is None or self.start is None:
            return
        
        if self.pingCount == 300:
            self.reset_ping_structures()
            
        self.pingCount += 1
        self.pingSum.north += (msg.north - self.position.north)
        self.pingSum.east += (msg.east - self.position.east)
        self.pingAvgHeading.north = self.pingSum.north / self.pingCount
        self.pingAvgHeading.east = self.pingSum.east / self.pingCount
        
        self.check_current_suitability()
    
    def current_sensor_cb(self, msg):
        return
        if self.start is None:
            return
        
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
        self.check_current_suitability()
        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
