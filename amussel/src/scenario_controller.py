#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
"""

__author__ = "barbanas"

import rospy
import actionlib
import amussel.msg
from auv_msgs.msg import NED
from geometry_msgs.msg import TwistStamped
from math import atan2, radians

class ScenarioController(object):
    
    def __init__(self):
        
        self.client = actionlib.SimpleActionClient('action_server', amussel.msg.aMusselAction)
        self.send_depth_goal(5)
        
        self.reset_ping_structures()
        
        self.current = NED(0, 0, 0)
        
        rospy.Subscriber("ping_sensor", NED, self.ping_sensor_cb)
        rospy.Subscriber("current_sensor", TwistStamped, self.current_sensor_cb)
        
        rospy.spin()
        
        
    def check_current_suitability(self): 
        
        if self.pingCount < 50:
            return  
        
        currentAngle = atan2(self.current.north, self.current.east) 
        swarmCenterAngle = atan2(self.pingAvgHeading.north, self.pingAvgHeading.east)
        
        if abs(currentAngle - swarmCenterAngle) < radians(45):
            self.reset_ping_structures()
            self.start_drifting(1)
            
    def start_drifting(self, duration):
        
        self.send_depth_goal(0.3)
        rospy.sleep(duration)
        self.send_depth_goal(5)
        
    def reset_ping_structures(self):
        
        self.pingCount = 0
        self.pingAvgHeading = NED(0, 0, 0) 
        self.pingSum = NED(0, 0, 0)
         
    def ping_sensor_cb(self, msg):
        
        self.pingCount += 1
        self.pingSum.north += msg.north
        self.pingSum.east += msg.east
        self.pingAvgHeading.north = self.pingSum.north / self.pingCount
        self.pingAvgHeading.east = self.pingSum.east / self.pingCount
        
        self.check_current_suitability()
    
    def current_sensor_cb(self, msg):
        
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
        
        self.check_current_suitability()
        
    def send_depth_goal(self, depth):
        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()
    
        rospy.loginfo('Connected to server')
    
        # Creates a goal to send to the action server.
        goalPosition = NED(0, 0, depth)
        goal = amussel.msg.aMusselGoal(id=0, position=goalPosition)
        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        self.client.get_result()

        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           