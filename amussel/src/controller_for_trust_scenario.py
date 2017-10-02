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
from misc_msgs.srv import GetPosition, GetSensoryReading
from std_msgs.msg import Bool
from math import atan2, radians, degrees, sqrt, pow, fabs
import random
import os
import action_library
import math

class ScenarioController(object):
    
    def __init__(self):

        self.gotCurrent = False
        self.current = NED(0, 0, 0)
        self.position = None
        self.start = False

        # position service
        rospy.Service('get_position', GetPosition, self.get_position_srv)
        # get sensory reading service
        rospy.Service('get_sensory_reading', GetSensoryReading, self.get_sensory_reading_srv)

        self.logs_folder = os.path.expanduser('~') + '/Desktop/logs_trust/last_simulation'
        if not os.path.exists(self.logs_folder):
            os.makedirs(self.logs_folder)
        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'path.txt', 'w'): pass
        with open(self.logs_folder + '/current.txt', 'w'): pass
        
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)


        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('current_sensor', TwistStamped, self.current_sensor_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)

        print rospy.get_namespace()
        
        rospy.spin()
        
    # callback functions
    
    def start_cb(self, msg):

        print 'start: ', rospy.get_namespace()
        self.start_time = rospy.get_time()
        self.start = True
        while self.position is None:
            rospy.sleep(0.1)
        try:
            self.send_depth_goal(5.0)
        except:
            pass
        self.startPub.publish(msg)

    def position_cb(self, msg):
        
        if not self.start:
            return
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)

        time_ = self.time_from_start()

        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'path.txt', 'a') as my_file:
            my_file.write('time:{0}--north:{1}--east:{2}--depth:{3}\n'.format(
                time_, self.position.north, self.position.east, self.position.depth)
            )

    def time_from_start(self):
        return (rospy.get_time() - self.start_time)
    
    def current_sensor_cb(self, msg):

        if not self.start:
            return
        
        self.gotCurrent = True
        self.current.north = msg.twist.linear.x
        self.current.east = msg.twist.linear.y
    
    # service functions
    
    def get_position_srv(self, req):
        
        while self.position is None:
            rospy.sleep(0.1)
            
        return [self.position.north, self.position.east, self.position.depth]
        
    def get_sensory_reading_srv(self, req):
        cur_north = self.current.north
        cur_east = self.current.east

        #cur_north = 1.0
        #cur_east = 0.5

        #cur_north = math.sin(self.time_from_start() * 2 * math.pi / 2000)
        #cur_east = math.cos(self.time_from_start() * 2 * math.pi / 2000)

        #if self.time_from_start() >= 600 and self.time_from_start() <= 800:
        cur_north = 1.0# - self.time_from_start() * 0.0005
        cur_east = 1.0 #- self.time_from_start() * 0.0005
        #elif self.time_from_start() > 800:
        #    cur_north = -1
        #    cur_east = -1.5


        if rospy.get_namespace() == '/amussel1/':
            with open(self.logs_folder + '/current.txt', 'a') as my_file:
                my_file.write('time:{0}--current:[{1},{2}]\n'.format(self.time_from_start(), cur_north, cur_east)
                          )



        if rospy.get_namespace() == '/amussel1/':
            k = [random.gauss(0.5 , 0.001), random.gauss(0.5 , 0.001)]
        else:
            k = [random.gauss(0, 0.0001), random.gauss(0, 0.0001)]
        #print k
        return {'current': [cur_north + k[0], cur_east + k[1]]}

        #if not self.gotCurrent:
        #    return {'current': [k[0], k[1]]}
        #else:
        #    pass
        
    def send_depth_goal(self, depth):

        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = NED(pos_old.north, pos_old.east, depth)
        
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
    
           
