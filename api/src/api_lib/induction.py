#!/usr/bin/env python

"""
Induction API class.
"""

__author__ = "barbanas"


from misc_msgs.srv import GetChargeInfo
from std_msgs.msg import Bool
import rospy

class Induction(object):
    
    def __init__(self):
        pass
    
    def enable_coil(self):
        pass    
    
    def disable_coil(self):
        pass   
    
    def send_energy(self):
        pass
    
    def get_charging_power(self):
        pass     