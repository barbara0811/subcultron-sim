#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario -- 
"""

__author__ = "barbanas"

import rospy
import action_library
import math

from misc_msgs.srv import GetPosition, GetSensoryReading, GetTrustInfo
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64
from math import radians, pow, sin, cos, pi, log, tan, sqrt, fabs
from random import random, choice
from copy import deepcopy
from scipy.integrate import odeint
import itertools
import numpy as np

area = [[-5, 5], [-5, 5]]

aFishList = []

for i in range(5):
    aFishList.append("/afish" + str(i + 1) + "/")

aMusselList = []
for i in range(5):
    aMusselList.append("/amussel" + str(i + 1) + "/")

class ScenarioController(object):
    
    def __init__(self):
        
        self.start = False
        
        self.yaw = 0
        self.position = None
        self.orientation = Point()
                
        # bacterial chemotaxis + levy random walk
        self.tumble = False
        self.tumbleValue = 0
        self.signalHistory = [0, 0]
        
        self.levyTimeout = None
        self.levyA = 0
        self.levyTumble = False
        
        # trust
        self.communicationRange = 5
        self.trust_sample = 0.001
        
        self.A = np.zeros([len(aFishList)])                # graph connectivity matrix
        self.A_previous = np.zeros([len(aFishList)])                # graph connectivity matrix
        self.b = np.zeros([len(aMusselList)])              # visited aMussels
        self.b_previous = np.zeros([len(aMusselList)])              # visited aMussels
        self.current =  np.zeros([len(aMusselList), 2])    # the latest aMussel sensory reading, vector -- [north, east]
        
        self.tau = np.zeros([len(aMusselList)])      # observation function about agent's trustworthiness
        self.delta =  np.zeros([len(aMusselList)])   # performance
        self.delta_previous =  np.zeros([len(aMusselList)])   # performance
        self.sigma =  np.zeros([len(aMusselList)])   # confidence
        self.sigma_previous =  np.zeros([len(aMusselList)])   # confidence
        self.sigma_init =  np.zeros([len(aMusselList)])   # initial values of confidence
        self.sigma_init_previous =  np.zeros([len(aMusselList)])   # initial values of confidence
        self.zeta_init =  np.zeros([len(aMusselList)])   # initial values of trust
        self.zeta_init_previous =  np.zeros([len(aMusselList)])   # initial values of trust
        self.adapt = 1
        self.K = 0.02

        self.diff_zeta_previous = np.zeros([len(aMusselList)])
        self.diff_zeta = np.zeros([len(aMusselList)])
        self.diff_sigma_previous = np.zeros([len(aMusselList)])
        self.diff_sigma = np.zeros([len(aMusselList)])

        self.flag_no_comm = np.zeros([len(aMusselList)])

        self.zeta =  np.zeros([len(aFishList), len(aMusselList)])    # trust matrix --> rows: aFish trust vectors
        self.zeta_previous =  np.zeros([len(aFishList), len(aMusselList)])    # trust matrix --> rows: aFish trust vectors
        self.index = aFishList.index(rospy.get_namespace())          # agent's index
        
        
        # publishers
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)
        
        # subscribers
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)
        
        rospy.Subscriber('noise_sensor', Float64, self.noise_cb)
        
        # position service
        rospy.Service('get_position', GetPosition, self.get_position_srv)
        
        # get trust vector service
        rospy.Service('get_trust_info', GetTrustInfo, self.get_trust_info_srv)
        
        # open to overwrite file content -- used for path visualization
        f = open('/home/petra/Desktop/logs_trust' + rospy.get_namespace()[:-1] + 'path.txt','w')
        f = open('/home/petra/Desktop/logs_trust' + rospy.get_namespace()[:-1] + 'A.txt','w')
        f = open('/home/petra/Desktop/logs_trust/conns_A.txt','w')
        
        # periodic function call
            
        # random / biased random walk
        rospy.Timer(rospy.Duration(0.1), self.bacterial_chemotaxis_levy_walk)
        
        # trust scenario
        rospy.Timer(rospy.Duration(0.2), self.update_communication_structures)
        rospy.Timer(rospy.Duration(self.trust_sample), self.trust)
        
        rospy.spin()
      
    # callback functions
      
    def position_cb(self, msg):
        
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
        
        if not self.start:
            return

        f = open('/home/petra/Desktop/logs_trust' + rospy.get_namespace()[:-1] + 'path.txt','a')
        f.write(str(self.position.north) + " " + str(self.position.east) + "\n")
           
    def start_cb(self, msg):
        
        self.startTime = rospy.get_time()
        
        # enable and configure controllers
        self.velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
        self.velcon_enable(enable=True)

        self.fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
        self.fadp_enable(enable=True)
     
        self.config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
        self.config_vel_controller(ControllerName="FADP", desired_mode=[2, 2, 2, 0, 0, 0])
        
        # submerge
        while self.position is None:
            rospy.sleep(0.1)
        
        action_library.send_position_goal(self.stateRefPub, NED(self.position.north, self.position.east, 5))
        depthOld = self.position.depth
        posErr = []
        
        while True:
            dL = abs(self.position.depth - depthOld)
            dl = abs(5 - self.position.depth)
    
            if len(posErr) < 20:
                posErr.append(dl)
            else:
                posErr.pop(0)
                posErr.append(dl)
    
            if (len(posErr) == 20) and (fabs(sum(posErr) / len(posErr)) < 0.1):  # mission is successfully finished
                break
        
        self.start = True
        self.startPub.publish(msg)
        
    def noise_cb(self, msg):
        
        if len(self.signalHistory) >= 30:
            del self.signalHistory[0]
            
        self.signalHistory.append(msg.data)
        
    # service functions
    
    def get_position_srv(self, req):
        
        while self.position is None:
            rospy.sleep(0.1)
            
        return [self.position.north, self.position.east, self.position.depth]
    
    def get_trust_info_srv(self, req):
        
        return {"zeta": self.zeta[self.index]}
    
    # scenario functions

    def bacterial_chemotaxis_levy_walk(self, event):
        '''
        TODO -- description
        '''
        while self.position is None or not self.start:
            rospy.sleep(0.1)

        # uncomment for simulation time tracking
        #if '1' in rospy.get_namespace():
        #    print "$$$$$$$$$$$$$$$$$  " + str(rospy.get_time() - self.startTime)
        C = 0
        
        # get sensory signal amplitude
        amp = self.signalHistory[-2:]
        newAmplitude = amp[1]
        # if in tumbling process, use the amplitude value at the start of tumbling
        if self.tumble:
            oldAmplitude = self.tumbleValue
        else:
            oldAmplitude = amp[0]
        
        if newAmplitude < 0:    # no sensory data -- perform Levy walk
            # clean up leftover chemotaxis flag (otherwise it can get stuck in tumbling)
            if self.tumble:
                self.tumble = False
            # levy walk
            if self.levyTumble:
                self.levyA = 0
                # constant time period for tumbling activity
                self.levyTimeout = rospy.get_time() + 0.1
                ##print "outside the area, tumbling"
            elif self.levyTimeout is None:
                t = self.levy_random()
                ##print "## keeping direction for " + str(t)
                self.levyTimeout = rospy.get_time() + t
                self.levyA = 1
            elif rospy.get_time() >= self.levyTimeout:
                # toggle attractor variable
                self.levyA = 1 - self.levyA
                if self.levyA == 0:
                    # constant time period for tumbling activity
                    self.levyTimeout = rospy.get_time() + 0.1
                else:
                    # levy random variable
                    t = self.levy_random()
                    ##print "## keeping direction for " + str(t)
                    self.levyTimeout = rospy.get_time() + t
                    
            A = self.levyA
        else:   # bacterial chemotaxis
            # if signal strength has lowered, change the angle until the signal improves again
            if newAmplitude - oldAmplitude <= C:
                A = 0
                # if tumbling was already occurring, discard the new signal readings until improvement
                if not self.tumble: # keep amplitude value at the start of tumbling process
                    self.tumbleValue = newAmplitude
                self.tumble = True
            else:
                A = 1
                self.tumble = False
            
        linVelocity = A
        if newAmplitude < 0: # Levy walk
            angVelocity = choice([-1, 1]) * (1 - A) * (60 + np.random.normal(0, 10))
        else: # bacterial chemotaxis
            angVelocity = (1 - A) * (30 + np.random.normal(0, 5))
        angVelocityRad = radians(angVelocity)
        
        deltax = linVelocity * cos(angVelocityRad + radians(self.yaw))
        deltay = linVelocity * sin(angVelocityRad + radians(self.yaw))
        deltatheta = angVelocity
        
        # prepare state reference
        stateRef = NavSts()
        
        self.yaw += deltatheta
        self.yaw %= 360
        
        stateRef.position.north = self.position.north + deltax
        stateRef.position.east = self.position.east + deltay
        stateRef.position.depth = self.position.depth
        stateRef.orientation.yaw = self.yaw 
        
        if stateRef.position.north < area[0][0] or stateRef.position.north > area[0][1] or \
            stateRef.position.east < area[1][0] or stateRef.position.east > area[1][1]: 
            # random walk is at the edge of the allowed area -- force tumbling
            self.levyTumble = True
            return
        else:
            self.levyTumble = False
        
        # calc orientation (only yaw rotation is allowed)
        self.orientation.x = cos(radians(self.yaw))
        self.orientation.y = sin(radians(self.yaw))
        self.orientation.z = 0
        
        # initialize movement
        action_library.send_state_ref(self.stateRefPub, stateRef)
        
    def levy_random(self):
        '''
        Levy alpha stable distribution random number generator. 
        Based on http://economics.sbs.ohio-state.edu/jhm/jhm.html (STABRND.M generator)
        '''
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
        return x

    def get_max_norm(self):
        '''
        A method to calculate maximum norm of all possible permutations in a matrix of currents.

        '''
        
        permutations_indices = list(itertools.permutations(np.linspace(0,len(aMusselList) - 1,len(aMusselList)),2))
        permutations_norms = np.zeros([len(permutations_indices)])
        for i in range(len(permutations_indices)):
            permutations_norms[i] = np.linalg.norm(self.current[int(permutations_indices[i][0])] - self.current[int(permutations_indices[i][1])])

        return max(permutations_norms)


    def init_trust(self):
        '''
        Initialization function for trust variables.
        '''

        self.sigma_init_previous = self.sigma_init
        self.zeta_init_previous = self.zeta_init
        self.delta_previous = self.delta
        self.delta =  np.zeros([len(aMusselList)])   # performance
        self.sigma_init =  np.zeros([len(aMusselList)]) 
        self.zeta_init =  np.zeros([len(aMusselList)]) 
        for i in range(len(aMusselList)):
            if self.b[i] == 1:
                self.sigma_init[i] = round(random(),1)
                self.zeta_init[i] = 0.1
                nb_of_subs = 0
                for j in range(len(aMusselList)):
                    if self.b[j] == 1: 
                        max_norm = self.get_max_norm()   
                        if max_norm != 0:
                            self.delta[i] = self.delta[i] + ((np.linalg.norm(self.current[j] - self.current[i]))/max_norm)**2
                            nb_of_subs += 1
                if nb_of_subs > 0:
                    self.delta[i] = sqrt(self.delta[i])/nb_of_subs
                nb_of_subs = 0
        # print "delta: " + str(self.delta) + "\n"

           
    def check_connectivity_matrix(self, matrix_A):


        connected_A = 0
        pr = [1]
        #check if tree is a spanning tree
        deg = np.zeros([len(aFishList),len(aFishList)])
        row_sum = np.zeros([len(aFishList)])

        for i in range(len(aFishList)):
            deg[i,i] = sum(matrix_A[i])
        L = deg - matrix_A
        row_sum = sum(np.transpose(L))

        # V - full matrix whose columns are the corresponding eigenvectors
        # D - diagonal matrix of eigenvalues

        D, V = np.linalg.eig(L)
        D = np.floor(D)
        # number of eigenvalues = 0
        number = len(np.where(D == 0)[0])  

        if number == 1:
            V_values = np.transpose(V)
            pr = np.round(np.dot(L,V_values[np.where(np.absolute(D) == 0)[0][0]]))

        if sum(row_sum) == 0 and sum(pr) == 0 and number == 1:
            connected_A = 1


        return connected_A



    def update_communication_structures(self, event):  
        '''
        A function that gets called periodically. It updates agent's communication structures
        and sensory reading data.
        '''
        
        while self.position is None or not self.start:
            rospy.sleep(0.1)
            
        #print "zeta: " + str(self.zeta[self.index])
        
        #if "1" in rospy.get_namespace():
        #    print ".. " + str(rospy.get_time())
        
        '''
        ---- OLD ----
        # update aFish connectivity matrix based on new position information
        newConnection = False # new connection flag TODO --> check if this is the correct behavior
        
        aFishesInRange = []
        for i in range(len(aFishList)):
            try:
                rospy.wait_for_service(aFishList[i] + 'get_position', 0.05)
                get_pos = rospy.ServiceProxy(aFishList[i] + 'get_position', GetPosition)
                p = get_pos()
                if sqrt(pow(self.position.north - p.x, 2) + \
                        pow(self.position.east - p.y, 2) + \
                        pow(self.position.depth - p.z, 2)) < self.communicationRange:
                    if self.A[i] == 0:
                        newConnection = True
                    self.A_previous[i] = self.A[i]
                    self.A[i] = 1
                    aFishesInRange.append(i)
                else:
                    self.A_previous[i] = self.A[i]
                    self.A[i] = 0
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
                
        # update aMussel connectivity matrix based on new position information
        aMusselsInRange = []
        for i in range(len(aMusselList)):
            try:
                rospy.wait_for_service(aMusselList[i] + 'get_position', 0.05)
                get_pos = rospy.ServiceProxy(aMusselList[i] + 'get_position', GetPosition)
            
                p = get_pos()
                if sqrt(pow(self.position.north - p.x, 2) + \
                        pow(self.position.east - p.y, 2) + \
                        pow(self.position.depth - p.z, 2)) < self.communicationRange:
                    aMusselsInRange.append(i)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
        
        '''
            
        rospy.wait_for_service('/get_connectivity_vectors', 0.1)
        get_conn = rospy.ServiceProxy('/get_connectivity_vectors', GetTrustInfo)
        
        result = get_conn(self.index)
        self.A = np.array(result.A)
        mussels = np.array(result.b)
        
        aFishesInRange = np.nonzero(self.A)[0]
        aMusselsInRange = np.nonzero(mussels)[0]
        
        f = open('/home/petra/Desktop/logs_trust' + rospy.get_namespace()[:-1] + 'A.txt','a')
        matrix_A = np.zeros([len(aFishList),len(aFishList)])

        for i in range(len(self.A)):
            matrix_A[self.index,i] = self.A[i]
            matrix_A[i,self.index] = self.A[i]

        aStr = str(rospy.get_time()) + "\n"
        for i in range(len(aFishList)):
            for item in matrix_A[i,:]:
                aStr += str(int(item)) + " "
            aStr += "\n"
        f.write(aStr[:-1] + "\n")

        f = open('/home/petra/Desktop/logs_trust/conns_A.txt','a')
        connected_A = self.check_connectivity_matrix(matrix_A)
        aStr = str(rospy.get_time()) + "\n" + str(int(connected_A)) + "\n"
        f.write(aStr[:-1] + "\n")
        
        # exchange trust information with aFishes in range
        for i in aFishesInRange:
            if i == self.index:
                continue
            try:
                rospy.wait_for_service(aFishList[i] + 'get_trust_info', 0.05)
                get_trust = rospy.ServiceProxy(aFishList[i] + 'get_trust_info', GetTrustInfo)
            
                result = get_trust(0)
                self.zeta[i] = result.zeta
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e     
         
        
        # get sensory reading data from aMussels in range
        for i in aMusselsInRange:
            try:
                rospy.wait_for_service(aMusselList[i] + 'get_sensory_reading', 0.1)
                get_sen = rospy.ServiceProxy(aMusselList[i] + 'get_sensory_reading', GetSensoryReading)
            
                result = get_sen()
                
                if len(result.current) == 2:
                    self.b_previous[i] = self.b[i]
                    self.current[i] = result.current
                    self.b[i] = 1
                    
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            except rospy.ROSException, e:
                print "Service call failed: %s"%e
    

    def deriv(self,y,t, alpha): # return derivatives of the array y #edit: put an extra arg
        return np.array([ self.diff_zeta[alpha[0]], 0])
    

    def deriv_sigma(self,y,t, alpha): # return derivatives of the array y #edit: put an extra arg
        return np.array([ self.diff_sigma[alpha[0]], 0])



    def sign(self,x): 
        '''
        signum function
        '''
        
        if x < 0: 
            return -1 
        elif x > 0: 
            return 1 
        else: 
            return 0 


    def trust(self, event):
        '''
        Trust implementation. Gets called upon information change.
        TODO -- implement + test behavior (is it better if it is called periodically?)
        '''
        
        while self.position is None or not self.start:
            rospy.sleep(0.1)
        
        # initialize trust variables
        self.init_trust()
        self.diff_zeta = np.zeros([len(aMusselList)])
        self.diff_sigma = np.zeros([len(aMusselList)])
        time_step = np.linspace(0.0,self.trust_sample,2)
        self.flag_no_comm = np.zeros([len(aMusselList)])

        for i in range(len(aMusselList)):

            if self.sigma_previous[i] == 0:
                self.tau[i] = 0
            else:                  
                self.tau[i] = math.exp(-(self.delta_previous[i] ** 2)/(self.sigma_previous[i] ** 2))

            for j in range(len(aFishList)):
                self.diff_zeta[i] = self.diff_zeta[i] + self.A_previous[j]*self.sign(self.zeta_previous[j,i] - self.zeta_previous[self.index,i]) + self.b_previous[i]*self.sign(self.tau[i] - self.zeta_previous[self.index,i])

            
            if self.adapt > 0:
                self.diff_sigma[i] = - self.K*self.b_previous[i]*self.sign(self.tau[i] - self.zeta_previous[self.index,i])
            else:
                self.diff_sigma[i] = 0
                     
    
            if self.b_previous[i] == 0:
                self.flag_no_comm[i] = 1
                self.diff_zeta[i] = 0
                self.diff_sigma[i] = 0

        # print "diff previous: " + str(self.diff_zeta_previous)

        self.diff_zeta = self.diff_zeta_previous + self.diff_zeta 
        # print "diff: " + str(self.diff_zeta)
        self.diff_sigma = self.diff_sigma_previous + self.diff_sigma

        self.diff_zeta_previous = self.diff_zeta
        self.diff_sigma_previous = self.diff_sigma

        for i in range(len(aMusselList)):       
            if self.flag_no_comm[i] == 0:

                alpha = [i]
                yinit = [self.zeta_init_previous[i], 0]
                yINT = odeint(self.deriv,yinit,time_step,args=(alpha, ))
                self.zeta[self.index,i] = yINT[:,0][1]           
                
                sigmainit = [self.sigma_init_previous[i], 0]
                sigmaINT = odeint(self.deriv_sigma,sigmainit,time_step,args=(alpha, ))
                self.sigma[i] = sigmaINT[:,0][1]
            else:
                self.zeta[self.index,i] = self.zeta_previous[self.index,i]
                self.sigma[i] = self.sigma_previous[i]          
                self.flag_no_comm[i] = 0

        # print "zeta previous: " + str(self.zeta_previous[self.index])
        
        self.zeta_previous = self.zeta
        self.sigma_previous = self.sigma
            

                        
if __name__ == "__main__":
    
    rospy.init_node("scenario_controller")
    
    try:
        controller = ScenarioController()
            
    except rospy.ROSInterruptException:
        pass
    
           
