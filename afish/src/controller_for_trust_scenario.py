#!/usr/bin/env python

"""
Initializes ROS node for high level mission control.
Trust scenario --
"""

__author__ = "barbanas"

import rospy
import time
import action_library
import math
import os
import xml
import re
import json

from misc_msgs.srv import GetPosition, GetSensoryReading, GetTrustInfo
from misc_msgs.msg import ConnMatrix, zeta
from navcon_msgs.srv import EnableControl, ConfigureVelocityController
from auv_msgs.msg import NED, NavSts
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float64
from math import radians, pow, sin, cos, pi, log, tan, sqrt, fabs
from random import random, choice
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=1000)

from types import *

### PARAMETERS ###

user = "barbara"

logs_dir = os.path.expanduser('~') + '/Desktop/logs_trust/'


class AgentData(object):
    # default simulation preferences

    # fish movement parameters
    area = [[-15, 15], [-15, 15]]
    noiseActivationRate = 5000  # seconds (noise activation rate)
    meet_period = 0.05
    linear_vel = 1

    # time samples
    trust_sample = 0.1
    comm_sample = 0.1
    walk_sample = 0.1

    quantize = False

    k_a = float('inf')
    k_a = 0.1
    a_init = float('inf')
    k_b = 0.1
    b_init = float('inf')

    time_limit = 2000
    communicationRange = 5


    def __init__(self, config_file_name=None):
        self.get_vehicles()
        self.index = self.afish_list.index(rospy.get_namespace())  # agent's index

        self.fishes_in_range = np.zeros([self.afish_number])
        self.mussels_in_range = np.zeros([self.amussel_number])

        if not config_file_name:
            config_file_name = logs_dir + 'config.txt'

        if os.path.isdir(config_file_name):
            with open(config_file_name, 'r') as my_file:
                data = json.load(my_file)

            for key in data:
                setattr(self, key, data['key'])



    def get_vehicles(self):
        """
        Read aFish amd aMussel vehicles from swarm_test.xml
        """

        self.amussel_list = []
        self.afish_list = []

        # parsing full swarm_test.xml path
        dir_path = os.path.dirname(os.path.realpath(__file__))
        project_folder_name = 'subcultron-sim'
        path = 'subcultron_launch/data/simulation/swarm_test.xml'
        full_path = dir_path[:dir_path.find(project_folder_name) + len(project_folder_name) + 1] + path

        swarm_test_root = xml.etree.ElementTree.parse(full_path).getroot()

        # parsing swarm_test.xml file to get info about amussels and afishes
        vehicles = []
        for child in swarm_test_root:
            if child.tag == 'vehicle':
                vehicles.append(child)

        for vehicle in vehicles:
            for child in vehicle:
                if child.tag == 'name':
                    if child.text.find('amussel') != -1:
                        self.amussel_list.append('/{0}/'.format(child.text))
                    elif child.text.find('afish') != -1:
                        self.afish_list.append('/{0}/'.format(child.text))

        self.afish_list.sort(key=self.natural_keys)
        self.amussel_list.sort(key=self.natural_keys)
        self.afish_number = len(self.afish_list)
        self.amussel_number = len(self.amussel_list)

    @staticmethod
    def atoi(text):
        return int(text) if text.isdigit() else text

    def natural_keys(self, text):
        '''
        alist.sort(key=natural_keys) sorts in human order
        http://nedbatchelder.com/blog/200712/human_sorting.html
        (See Toothy's implementation in the comments)
        '''
        return [self.atoi(c) for c in re.split('(\d+)', text)]

    def start(self):
        self.start_time = rospy.get_time()

    def time_from_start(self):
        return (rospy.get_time() - self.start_time)


class ScenarioController(object):
    def __init__(self):
        self.data = AgentData()
        self.index = self.data.index

        # the latest aMussel sensory reading, vector [north, east]
        self.current = np.zeros([len(self.data.amussel_list), 2])
        #self.current.fill(np.nan)
        self.current_trust = Trust(self.data, self.current)
        if self.data.k_b == float('inf'):
            self.current_trust.value_weight = False


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

        # publishers
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        self.startPub = rospy.Publisher('start_sim', Bool, queue_size=1)

        self.zetaPub = rospy.Publisher('zeta', zeta, queue_size=1)

        self.noiseIntensityPub = rospy.Publisher("/noise_intensity", Float64, queue_size=1)
        self.noiseTimeout = None
        self.noiseRate = self.data.noiseActivationRate  # seconds (noise activation rate)
        self.noiseActivated = False

        # subscribers
        rospy.Subscriber('/scenario_start', Bool, self.start_cb)
        rospy.Subscriber('position', NavSts, self.position_cb)

        rospy.Subscriber('noise_sensor', Float64, self.noise_cb)

        # position service
        rospy.Service('get_position', GetPosition, self.get_position_srv)

        # get trust vector service
        rospy.Service('get_trust_info', GetTrustInfo, self.get_trust_info_srv)

        # open to overwrite file content -- used for path visualization
        self.logs_folder = logs_dir + 'last_simulation/'

        if not os.path.exists(self.logs_folder):
            os.makedirs(self.logs_folder)


        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'path.txt', 'w'): pass
        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'current_zeta.txt', 'w'): pass


        # periodic function call

        # random / biased random walk
        rospy.Timer(rospy.Duration(self.data.walk_sample), self.bacterial_chemotaxis_levy_walk)

        # trust scenario
        rospy.Timer(rospy.Duration(self.data.comm_sample), self.update_communication_structures)
        rospy.Timer(rospy.Duration(self.data.trust_sample), self.trust)

        rospy.spin()

    # callback functions

    def position_cb(self, msg):

        self.position = NED(msg.position.north, msg.position.east, msg.position.depth)

        if not self.start:
            return

        #if not hasattr(self, 'pos_step'):
        #    self.pos_step = 0
        #else:
        #    self.pos_step += 1

        #if self.pos_step % self.time_factor:
        #    return

        time_ = self.data.time_from_start()

        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'path.txt', 'a') as my_file:
            my_file.write('time:{0}--north:{1}--east:{2}--depth:{3}\n'.format(
                time_, self.position.north, self.position.east, self.position.depth)
            )

    def start_cb(self, msg):

        # enable and configure controllers
        print 'start: ', self.index
        self.data.start()
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
        if "1" in rospy.get_namespace() and self.noiseRate > 0:
            #t = expovariate(1.0 / float(self.noiseRate))
            self.noiseTimeout = rospy.get_time() + self.noiseRate

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
        if self.data.quantize:
            rtn = []
            for zeta in self.current_trust.zeta[self.index]:
                rtn.append(self.quantize(zeta))
            return {"zeta": np.array(rtn)}

        return {"zeta": self.current_trust.zeta[self.index]}

    # scenario functions

    def bacterial_chemotaxis_levy_walk(self, event):
        '''
        TODO -- description
        '''
        if not self.start:
            return

        while self.position is None or not self.start:
            rospy.sleep(0.1)

        if self.noiseTimeout is not None and not self.noiseActivated and rospy.get_time() >= self.noiseTimeout and "1" in rospy.get_namespace():
            self.noiseIntensityPub.publish(Float64(10.0))
            self.noiseActivated = True
            return

        # uncomment for simulation time tracking
        # if '1' in rospy.get_namespace():
        #    print "$$$$$$$$$$$$$$$$$  " + str(rospy.get_time() - self.startTime)
        C = 0

        # get sensory signal amplitude
        amp = self.signalHistory[-2:]
        newAmplitude = amp[1]
        # if in tumbling process, use the amplitude value at the start of tumbling
        # if self.tumble:
        #    oldAmplitude = self.tumbleValue
        # else:
        oldAmplitude = amp[0]

        if newAmplitude < 0:  # no sensory data -- perform Levy walk
            # clean up leftover chemotaxis flag (otherwise it can get stuck in tumbling)
            if self.tumble:
                self.tumble = False
            # levy walk
            if self.levyTumble:
                self.levyA = 0
                # constant time period for tumbling activity
                self.levyTimeout = rospy.get_time() + 0.2# * self.time_factor
                # print "ouertside the area, tumbling"
            elif self.levyTimeout is None:
                t = self.levy_random()
                # print "## keeping direction for " + str(t)
                self.levyTimeout = rospy.get_time() + t# * self.time_factor
                self.levyA = 1
            elif rospy.get_time() >= self.levyTimeout:
                # toggle attractor variable
                self.levyA = 1 - self.levyA
                if self.levyA == 0:
                    # constant time period for tumbling activity
                    self.levyTimeout = rospy.get_time() + 0.2
                    # print "tumbling"
                else:
                    # levy random variable
                    t = self.levy_random()
                    # print "## keeping direction for " + str(t)
                    self.levyTimeout = rospy.get_time() + t# * self.time_factor

            A = self.levyA
        else:  # bacterial chemotaxis
            # if signal strength has lowered, change the angle until the signal improves again
            if newAmplitude - oldAmplitude <= C:
                A = 0
                # if tumbling was already occurring, discard the new signal readings until improvement
                if not self.tumble:  # keep amplitude value at the start of tumbling process
                    self.tumbleValue = newAmplitude
                self.tumble = True
            else:
                A = 1
                self.tumble = False

        linVelocity = A
        if newAmplitude < 0:  # Levy walk
            angVelocity = choice([-1, 1]) * (1 - A) * (60 + np.random.normal(0, 10))
        else:  # bacterial chemotaxis
            angVelocity = (1 - A) * (30 + np.random.normal(0, 5))
        angVelocityRad = radians(angVelocity)

        linVelocity *= self.data.linear_vel

        deltax = linVelocity * cos(angVelocityRad + radians(self.yaw))
        deltay = linVelocity * sin(angVelocityRad + radians(self.yaw))
        deltatheta = angVelocity
        stateRef = NavSts()

        # if self.index == 0:
        #    print deltax, deltay, deltatheta

        # prepare state reference
        self.yaw += deltatheta
        self.yaw %= 360

        stateRef.position.north = self.position.north + deltax
        stateRef.position.east = self.position.east + deltay
        stateRef.position.depth = self.position.depth
        stateRef.orientation.yaw = self.yaw

        if stateRef.position.north < self.data.area[0][0] or stateRef.position.north > self.data.area[0][1] or \
                        stateRef.position.east < self.data.area[1][0] or stateRef.position.east > self.data.area[1][1]:
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
        alpha = 0.5  # characteristic exponent
        beta = 1  # skewness parameter
        c = 0.5  # scale
        delta = 0  # location parameter

        w = -log(random())
        phi = (random() - 0.5) * pi

        cosphi = cos(phi)
        if abs(alpha - 1) > 1.0e-8:
            zeta = beta * tan(pi * alpha / 2)
            aphi = alpha * phi
            a1phi = (1 - alpha) * phi
            x = ((sin(aphi) + zeta * cos(aphi)) / cosphi) \
                * (cos(a1phi) + zeta * sin(a1phi)) \
                / pow(w * cosphi, (1 - alpha) / alpha)
        else:
            bphi = (pi / 2) + beta * phi
            x = (2 / pi) * (bphi * tan(phi) - beta * log((pi / 2) * w * cosphi / bphi))
            if abs(alpha - 1) < 0.00001:
                x = x + beta * tan(pi * alpha / 2)

        x = delta + c * x
        return x

    def update_communication_structures(self, event):
        '''
        A function that gets called periodically. It updates agent's communication structures
        and sensory reading data.
        '''
        if not self.start:
            return

        while self.position is None or not self.start:
            rospy.sleep(0.1)

        try:
            rospy.wait_for_service('/get_connectivity_vectors', 0.5)
            get_conn = rospy.ServiceProxy('/get_connectivity_vectors', GetTrustInfo)
        except rospy.ServiceException, e:
            # print "Service call failed: %s"%e
            return
        except rospy.ROSException, e:
            # print "Service call failed: %s"%e
            return

        result = get_conn(self.index)

        _result_A = np.array(result.A)
        _result_B = np.array(result.b)

        aFishesInRange = np.nonzero(_result_A)[0]
        aMusselsInRange = np.nonzero(_result_B)[0]

        if "1" in rospy.get_namespace():
            # if all fished are in range, turn off bacterial chemotaxis and start Levy walk (deactivate noise)
            if len(aFishesInRange) == self.data.afish_number - 1 and self.noiseActivated:
                if not hasattr(self, 'meeting_timeout'):
                    self.meeting_active = True
                    self.meeting_timeout = rospy.get_time() + self.data.meet_period
                if self.meeting_active:
                    if self.meeting_timeout <= rospy.get_time():
                        self.noiseIntensityPub.publish(Float64(-1.0))
                        self.noiseActivated = False
                        #t = expovariate(1.0 / float(self.noiseRate))
                        self.noiseTimeout = rospy.get_time() + self.noiseRate
                        self.meeting_active = False
                else:
                    self.meeting_active = True
                    self.meeting_timeout = rospy.get_time() + self.data.meet_period

        fishes_in_range = np.zeros(self.data.afish_number)
        for i in aFishesInRange:
            if i == self.index:
                continue

            try:
                rospy.wait_for_service(self.data.afish_list[i] + 'get_trust_info', 0.05)
                get_trust = rospy.ServiceProxy(self.data.afish_list[i] + 'get_trust_info', GetTrustInfo)

                result = get_trust(0)

                self.current_trust.zeta[i] = result.zeta
                fishes_in_range[i] = 1
            except rospy.ServiceException, e:
                # print "Service call failed: %s"%e
                pass
            except rospy.ROSException, e:
                # print "Service call failed: %s"%e
                pass


        mussels_in_range = np.zeros(self.data.amussel_number)
        for i in aMusselsInRange:
            try:
                rospy.wait_for_service(self.data.amussel_list[i] + 'get_sensory_reading', 0.1)
                get_sen = rospy.ServiceProxy(self.data.amussel_list[i] + 'get_sensory_reading', GetSensoryReading)

                result = get_sen()

                if len(result.current) == 2:
                    self.current[i] = result.current
                    mussels_in_range[i] = 1

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            except rospy.ROSException, e:
                print "Service call failed: %s" % e

        self.current_trust.set_a(fishes_in_range)
        self.current_trust.set_b(mussels_in_range)
        self.data.fishes_in_range = np.array(fishes_in_range)
        self.data.mussels_in_range= np.array(mussels_in_range)

    def trust(self, event):
        """

        :param event:
        :return:
        """

        if not self.start:
            return

        while self.position is None or not self.start:
            rospy.sleep(0.1)


        self.current_trust.calculate()
        with open(self.logs_folder + rospy.get_namespace()[:-1] + 'current_zeta.txt', 'a') as my_file:
            my_file.write(
                'time:{0}--zeta:{1}--A:{2}--conn_A:{3}--B:{4}--conn_B:{5}--current:{6}--estimated_current:{7}--sigma:{8}--delta:{9}\n'.format(
                    self.data.time_from_start(),
                    np.array2string(self.current_trust.zeta[self.index], separator=','),
                    np.array2string(self.current_trust.A, separator=','),
                    np.array2string(self.data.fishes_in_range, separator=','),
                    np.array2string(self.current_trust.B, separator=','),
                    np.array2string(self.data.mussels_in_range, separator=','),
                    np.array2string(self.current, separator=',').replace('\n', ''),
                    np.array2string(self.current_trust.estimated_value, separator=','),
                    np.array2string(self.current_trust.sigma, separator=','),
                    np.array2string(self.current_trust.delta, separator=',')
                )
            )

        self.zetaPub.publish(self.current_trust.zeta[self.index])

        if not hasattr(self, 'trust_step_counter'):
            self.trust_step_counter = 0
        self.trust_step_counter += 1
        if not self.trust_step_counter % ((1 / self.data.trust_sample) * 50):
            # just printing current simulation state
            time_from_start = self.data.time_from_start()
            print 'Fish {0}, iteration {1}, time {2}, calculated time {3}, delay {4}'.format(
                self.index,
                self.trust_step_counter,
                time_from_start,
                self.trust_step_counter * self.data.trust_sample,
                time_from_start - self.trust_step_counter * self.data.trust_sample
            )

            if time_from_start >= self.data.time_limit:
                working_dir = '/home/ja/Desktop/logs_trust/finished.txt'
                with open(working_dir, 'w'): pass

    @staticmethod
    def quantize(input_, step=0.1, range_=(0, 1)):
        factor = ((range_[1] - range_[0]) / step)
        return float(round(input_ * factor)) / factor


class Trust(object):
    default_values = {
        'zeta_init': 0.1,
        'sigma_init': 0.1,
        'k': 0.000002,
        'median_filter_tolerance': 0.5,
        'value_weight': True
    }

    def __init__(self, data, value, **kwargs):
        self.data = data  # general agent data
        self.value = value  # value to calculate trust

        self.estimated_value = np.zeros(len(value[0]))

        self.zeta = np.zeros([data.afish_number, data.amussel_number])
        self.delta = np.zeros([data.amussel_number])

        for key, value in kwargs.iteritems():
            if key not in self.default_values:
                print("Can not set attribute not defined in default_values")
            else:
                setattr(self, key, value)

        for key in self.default_values:
            if not hasattr(self, key):
                setattr(self, key, self.default_values[key])

        if type(self.zeta_init) in (IntType, FloatType):
                self.zeta[data.index].fill(self.zeta_init)
        else:
            if len(self.zeta_init) != data.amussel_number:
                raise ValueError('Length of zeta_init argument ({0}) not matching to number of mussels ({1})'.format(
                    len(self.zeta_init), data.amussel_number
                ))
            self.zeta[data.index] = self.zeta_init

        if type(self.sigma_init) in (IntType, FloatType):
            self.sigma = np.zeros([data.amussel_number])
            self.sigma.fill(self.sigma_init)
        else:
            if len(self.sigma_init) != data.amussel_number:
                raise ValueError('Length of sigma_init argument ({0}) not matching to number of mussels ({1})'.format(
                    len(self.sigma_init), data.amussel_number
                ))
            self.sigma = self.sigma_init

        self._A = np.zeros([self.data.afish_number])
        self._A.fill(self.data.a_init)
        self._A_rtn = np.zeros([self.data.afish_number])
        self._B = np.zeros([self.data.amussel_number])
        self._B.fill(self.data.b_init)
        self._B_rtn = np.zeros([self.data.amussel_number])

    @property
    def A(self):
        return self._A_rtn

    def set_a(self, a_matrix):
        if not hasattr(self, 'last_a_update'):
            period_from_last_update = self.data.comm_sample
        else:
            period_from_last_update = rospy.get_time() - self.last_a_update

        self.last_a_update = rospy.get_time()
        self._A += (1 - a_matrix) * period_from_last_update
        n = np.ma.masked_where(a_matrix == 1, self._A)
        self._A = np.ma.filled(n, 0)
        self._A_rtn = np.array([self.get_a(i) for i in range(len(self._A))])

    def get_a(self, index):
        if self._A[index] == 0:
            return 1
        if self._A[index] == float('inf'):
            return 0
        else:
            return math.exp(- self.data.k_a * self._A[index])

    @property
    def B(self):
        return self._B_rtn

    def set_b(self, b_matrix):
        if not hasattr(self, 'last_b_update'):
            period_from_last_update = self.data.comm_sample
        else:
            period_from_last_update = rospy.get_time() - self.last_b_update

        self.last_b_update = rospy.get_time()
        self._B += (1 - b_matrix) * period_from_last_update
        n = np.ma.masked_where(b_matrix == 1, self._B)
        self._B = np.ma.filled(n, 0)
        self._B_rtn = np.array([self.get_b(i) for i in range(len(self._B))])

    def get_b(self, index):
        if self._B[index] == 0:
            return 1
        elif self._B[index] == float('inf'):
            return 0
        else:
            return math.exp(- self.data.k_b * self._B[index])

    def calculate(self):
        """
        Calculate trust value (zeta)
        """

        self.calculate_delta()
        self.discrete()

    def discrete(self):
        i = self.data.index

        d_zeta = np.zeros([self.data.amussel_number])
        d_sigma = np.zeros([self.data.amussel_number])

        # get current values
        a = np.array(self.A)
        b = np.array(self.B)
        delta = np.array(self.delta)
        zeta = np.array(self.zeta)
        dii = a.sum()

        for j in range(self.data.amussel_number):
            observation = self.tau(self.sigma[j], delta[j]) - zeta[i][j]
            for k in range(self.data.afish_number):
                d_zeta[j] += a[k] * (zeta[k][j] - zeta[i][j])

            d_zeta[j] += b[j] * observation
            d_zeta[j] /= (dii + 1 + b[j])
            #if np.isnan(np.sum(d_zeta[j])):
            #    print observation, dii

            d_sigma[j] += -self.k * observation * b[j]

        self.sigma += d_sigma

        self.zeta[i] += d_zeta

    def calculate_delta(self):
        average = self.average_with_median()
        if average is None or np.isnan(np.sum(average)):
            if average is not None:
                print average
            return
        self.estimated_value = average
        for i, value in enumerate(self.value):
            if np.isnan(np.sum(value)):
                continue
            error_vector = average - value
            self.delta[i] = np.linalg.norm(error_vector)

    def average_with_median(self):
        """
        Calculates average with median filter
        """
        value_vector_ = []
        indices = []

        for i, x in enumerate(self.value):
            # get all measured data
            if not np.isnan(np.sum(x)):
                # if x measured ->  x != None
                value_vector_.append(x)
                indices.append(i)

        if value_vector_:
            median = np.median(value_vector_, axis=0)
        else:
            return None

        sum_value = np.zeros(len(self.value[0]))
        sum_weight = np.zeros(len(self.value[0]))
        for i, x in enumerate(value_vector_):
            # filter values
            if np.linalg.norm(x - median) < self.median_filter_tolerance:
                weight = self.zeta[self.data.index][indices[i]]
                if self.value_weight:
                    # if value_weight enabled -> include B matrix element weight in calculation of weighted average
                    weight *= self.B[indices[i]]
                sum_value += np.array([(weight * x[i]) for i in range(len(x))])
                sum_weight += weight

        return (sum_value / sum_weight) if sum_weight.all() else median

    @staticmethod
    def tau(sigma, delta):
        """
        Calculate tau
        """
        if sigma:
            return math.exp(-(delta ** 2) / (sigma ** 2))
        else:
            return 0

    @staticmethod
    def sign_threshold(self, x, threshold=0.0001):
        if abs(x) <= threshold:
            return 0
        else:
            return np.sign(x)

    def standard(self):
        # old - not using!
        # derived from trust algorithm equation for continuous time domain
        i = self.data.index
        d_zeta = np.zeros([self.data.amussel_number])
        d_sigma = np.zeros([self.data.amussel_number])

        a = np.array(self.A)
        b = np.array(self.B)
        delta = np.array(self.delta)
        zeta = np.array(self.zeta)
        for j in range(self.data.amussel_number):
            for k in range(self.data.afish_number):
                d_zeta[j] += a[k] * (self.zeta[k][j] - self.zeta[i][j])
            observation = Trust.tau(self.sigma[j], self.delta[j]) - self.zeta[i][j]
            d_zeta[j] = (d_zeta[j] + b[j] * observation) * self.data.trust_sample
            d_sigma[j] = - self.k * b[j] * observation * self.data.trust_sample

        self.sigma += d_sigma
        self.zeta[i] += d_zeta

    @staticmethod
    def weighted_average(value_vector, weight):
        """
        Calculates weighted average

        Args:
            value_vector - vector with values (value can be n dimension vector)
            weight       - vector with value weights
        Returns:
            weighted average
        """
        sum_value = np.zeros(len(value_vector[0]))
        for index, vector in enumerate(value_vector):
            sum_value += np.array([(weight[index] * vector[i]) for i in range(len(vector))])
        weight_sum = sum(weight)
        return (sum_value / weight_sum) if weight_sum != 0 else None


if __name__ == "__main__":

    rospy.init_node("scenario_controller")

    try:
        controller = ScenarioController()

    except rospy.ROSInterruptException:
        pass