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
        self.start = False

        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('current_sensor', TwistStamped, self.current_sensor_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        
        rospy.spin()
    
    def current_sensor_cb(self, msg):

        if not self.start:
            return
        
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
    def position_cb(self, msg):
        
        if not self.start:
            return
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
            
    def start_cb(self, msg):
        
        self.start = True
        while self.position is None:
            rospy.sleep(0.1)
        self.send_depth_goal(12.5)
        
    def send_depth_goal(self, depth):

        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = NED(pos_old.north, pos_old.east, depth)
        print goal
        if action_library.send_depth_goal(self.stateRefPub, goal) == -1:
            return

        pos_err = []
        while not rospy.is_shutdown():
            dL = abs(goal.depth - pos_old.depth)
            dl = abs(goal.depth - self.position.depth)
            if len(pos_err) < 10:
                pos_err.append(dl)
            else:
                pos_err.pop(0)
                pos_err.append(dl)

            if (len(pos_err) == 10) and (fabs(sum(pos_err) / len(pos_err)) < 0.05):  # mission is successfully finished
                return
            else:  #mission is still ongoing
                rospy.sleep(0.1)
                
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
