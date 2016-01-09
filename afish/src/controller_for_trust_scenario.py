#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario -- 
"""

__author__ = "barbanas"

import rospy
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import TwistStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool, Float64
from math import atan2, degrees, radians, sqrt, pow, fabs, sin, cos, pi, log, tan
import action_library
from random import random, choice
import matplotlib.pyplot as plt
import numpy as np

area = [[-40, 40], [-40, 40]]

class ScenarioController(object):
    
    def __init__(self):
        
        self.yaw = 0
        self.position = None
        self.orientation = Point()
        self.start = False
        
        # constrained random walk
        '''
        self.targetPos = NED(30, 30, 0)
        self.p = None
        self.goalPos = None
        self.theta = [0, 0, 0] # probability of decreasing the value i for each dimension (NED) 
        self.area = [[-100, 100], [-100, 100], [0, 15]]
        self.N = 30 # reach the target position in N steps
        self.i = 0
        self.pointList = []
        self.visited = []
        '''
                
        # bacterial chemotaxis levy random walk
        self.tumble = False
        self.tumbleValue = 0
        self.signalHistory = [0, 0]
        
        self.levyTimeout = None
        self.levyA = 0
        self.levyTumble = False
        
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        
        rospy.Subscriber('noise_sensor', Float64, self.noise_cb)
        
        rospy.Timer(rospy.Duration(0.1), self.bacterial_chemotaxis_levy_walk)
        
        # open to overwrite file contents -- used for path visualization
        f = open('/home/barbara/Desktop' + rospy.get_namespace()[:-1] + 'path.txt','w')

        rospy.spin()
        
    def position_cb(self, msg):
        
        if not self.start:
            return
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
        f = open('/home/barbara/Desktop' + rospy.get_namespace()[:-1] + 'path.txt','a')
        f.write(str(self.position.north) + " " + str(self.position.east) + "\n")
            
    def start_cb(self, msg):
        
        self.startTime = rospy.get_time()
        self.start = True
        while self.position is None:
            rospy.sleep(0.1)
            
        # enable and configure controllers
        self.velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
        self.velcon_enable(enable=True)

        self.fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
        self.fadp_enable(enable=True)
     
        self.config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
        self.config_vel_controller(ControllerName="FADP", desired_mode=[2, 2, 2, 0, 0, 0])
        
        self.startPub.publish(msg)
        
    ''' 
    def constrained_random_walk_cb(self, event):
        
        if not self.start:
            return
        
        while self.p is None:
            rospy.sleep(0.1)
        
        if self.goalPos is None:
            self.i = 0
            self.N = int(sqrt(pow(self.p.north - self.targetPos.north, 2) + \
                              pow(self.p.east - self.targetPos.east, 2))) * 10
                              #+ \
                              #pow(self.p.depth - self.targetPos.depth, 2)))
            print "### " + str(self.N)
        else:
            self.i += 1
        
        if self.i == self.N:
            self.i = 0
            #print self.pointList
            plt.figure(figsize=(20,10))
            
            axes = plt.gca()
            axes.set_xlim([-50,50])
            axes.set_ylim([-50,50])
            
            for i in reversed(range(1, len(self.pointList))):
                n1 = self.pointList[i - 1]
                n2 = self.pointList[i]
                
                if '1' in rospy.get_namespace():
                    plt.plot([n1.north, n2.north], [n1.east, n2.east], "bo", linestyle='-')
                else:
                    plt.plot([n1.north, n2.north], [n1.east, n2.east], "ro", linestyle='-')
            
            plt.show(block=False)
            self.start = False
            
        roam = True
        if self.i < self.N / 2:
            pass #roam = True
        
        self.goalPos = self.p
        
        if roam:
            self.theta = [0.5, 0.5, 0.5]
        else:
            self.theta[0] = (1 - float(self.targetPos.north - self.goalPos.north) / (self.N - self.i)) / 2
            self.theta[1] = (1 - float(self.targetPos.east - self.goalPos.east) / (self.N - self.i)) / 2
            self.theta[2] = (1 - float(self.targetPos.depth - self.goalPos.depth) / (self.N - self.i)) / 2
        
        prob = random()
        if prob > self.theta[0] and str(int(self.goalPos.north + 2)) + str(int(self.goalPos.east)) not in self.visited:
            self.goalPos.north += 2
        else:
            self.goalPos.north -= 2
            
        if self.goalPos.north < self.area[0][0]:
            self.goalPos.north = self.area[0][0]
        if self.goalPos.north > self.area[0][1]:
            self.goalPos.north = self.area[0][1]
        
        prob = random()    
        if prob > self.theta[1]:
            self.goalPos.east += 2
        else:
            self.goalPos.east -= 2
            
        if self.goalPos.east < self.area[1][0]:
            self.goalPos.east = self.area[1][0]
        if self.goalPos.east > self.area[1][1]:
            self.goalPos.east = self.area[1][1]
             
         
        if prob > self.theta[2]:
            self.goalPos.depth += 1
        else:
            self.goalPos.depth -= 1
            
        if self.goalPos.depth < self.area[2][0]:
            self.goalPos.depth = self.area[2][0]
        if self.goalPos.depth > self.area[2][1]:
            self.goalPos.depth = self.area[2][1]
         
        print "..."
        print self.i
        print self.theta    
        print self.goalPos
        print self.p
        print ".."
        
        self.p = NED(self.goalPos.north, self.goalPos.east, self.goalPos.depth)
        self.pointList.append(NED(self.goalPos.north, self.goalPos.east, self.goalPos.depth))
        self.visited.append(str(int(self.goalPos.north)) + str(int(self.goalPos.east)))
        if len(self.visited) > 10:
            del self.visited[0]
        #self.send_position_goal(self.goalPos, False)
    '''
                
    def bacterial_chemotaxis_levy_walk(self, event):
        '''
        '''
        
        while self.position is None:
            rospy.sleep(0.1)

        if '1' in rospy.get_namespace():
            print "$$$$$$$$$$$$$$$$$  " + str(rospy.get_time() - self.startTime)
        C = 0
        
        # get sensory signal amplitude
        amp = self.signalHistory[-2:]
        newAmplitude = amp[1]
        # if in tumbling process, use the amplitude value at the start of tumbling
        if self.tumble:
            oldAmplitude = self.tumbleValue
        else:
            oldAmplitude = amp[0]
        
        # no sensory data -- perform Levy walk
        if newAmplitude < 0:
            # levy walk
            # start levy walk
            
            if self.levyTumble:
                self.levyA = 0
                # constant time period for tumbling activity
                self.levyTimeout = rospy.get_time() + 0.2
                #print "outside the area, tumbling"
            elif self.levyTimeout is None:
                t = self.levy_random()
                #print "## keeping direction for " + str(t)
                self.levyTimeout = rospy.get_time() + t
                self.levyA = 1
            elif rospy.get_time() >= self.levyTimeout:
                # toggle attractor variable
                self.levyA = 1 - self.levyA
                if self.levyA == 0:
                    # constant time period for tumbling activity
                    self.levyTimeout = rospy.get_time() + 0.2
                else:
                    # levy random variable
                    t = self.levy_random()
                    #print "## keeping direction for " + str(t)
                    self.levyTimeout = rospy.get_time() + t
                    
            A = self.levyA
        else:
            # bacterial chemotaxis
            # if signal strength has lowered, change the angle until the signal improves again
            if newAmplitude - oldAmplitude <= C:
                A = 0
                # if tumbling was already occurring, discard the new signal readings until improvement
                if not self.tumble:
                    self.tumbleValue = newAmplitude
                self.tumble = True
            else:
                A = 1
                self.tumble = False
            
        linVelocity = A
        if newAmplitude < 0:
            angVelocity = choice([-1, 1]) * (1 - A) * (60 + np.random.normal(0, 10))
        else:
            angVelocity = (1 - A) * (30 + np.random.normal(0, 10))
        angVelocityRad = radians(angVelocity)
        
        deltax = linVelocity * cos(angVelocityRad + radians(self.yaw))
        deltay = linVelocity * sin(angVelocityRad + radians(self.yaw))
        deltatheta = angVelocity
        #print ".."
        #print [deltax, deltay, deltatheta]
        stateRef = NavSts()
        
        self.yaw += deltatheta
        self.yaw %= 360
        
        stateRef.position.north = self.position.north + deltax
        stateRef.position.east = self.position.east + deltay
        stateRef.position.depth = self.position.depth
        stateRef.orientation.yaw = self.yaw 
        
        if stateRef.position.north < area[0][0] or stateRef.position.north > area[0][1] or \
            stateRef.position.east < area[1][0] or stateRef.position.east > area[1][1]:
            self.levyTumble = True
            return
        else:
            self.levyTumble = False
        
        self.orientation.x = cos(radians(self.yaw))
        self.orientation.y = sin(radians(self.yaw))
        self.orientation.z = 0
        
        action_library.send_state_ref(self.stateRefPub, stateRef)
        
    def levy_random(self):
        m = 1
        n = 1
        # generates m x n matrix
        alpha = 0.5     # characteristic exponent
        beta = 1        # skewness parameter
        c = 0.5         # scale
        delta = 0       # location parameter
        
        w = -log(random())
        phi = (random() - 0.5) * pi
        
        cosphi = cos(phi)
        if abs(alpha - 1) > 1.0e-8:
            zeta = beta * tan(pi * alpha / 2)
            aphi = alpha * phi
            a1phi = (1 - alpha) * phi
            x = ((sin(aphi) + zeta * cos(aphi)) / cosphi)\
                * (cos(a1phi) + zeta * sin(a1phi)) \
                / pow(w * cosphi, (1 - alpha) / alpha)
        else:
            bphi = (pi / 2) + beta * phi
            x = (2 / pi) * (bphi * tan(phi) - beta * log((pi / 2) * w * cosphi / bphi))
            if abs(alpha - 1) < 0.00001:
                x = x + beta * tan(pi * alpha / 2) 
            
        x = delta + c * x
        print x
        return x
        
    def noise_cb(self, msg):
        
        if len(self.signalHistory) >= 30:
            del self.signalHistory[0]
            
        self.signalHistory.append(msg.data)
        
    def send_position_goal(self, position, wait):

        pos_old = NED(self.position.north, self.position.east, self.position.depth)
        goal = position
        if action_library.send_position_goal(self.stateRefPub, goal) == -1:
            return

        if wait:
            pos_err = []
            while not rospy.is_shutdown():
                dL = sqrt(pow(goal.north - pos_old.north, 2) + pow(goal.east - pos_old.east, 2) \
                          + pow(goal.depth - pos_old.depth, 2))
                dl = sqrt(pow(goal.north - self.position.north, 2) + pow(goal.east - self.position.east, 2) \
                          + pow(goal.depth - self.position.depth, 2))
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
    
           
