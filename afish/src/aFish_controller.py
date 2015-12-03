#!/usr/bin/env python

__author__= "Anja Zrnec"

import random
import action_library # depth goal & new position
import main_controller
from auv_msgs.msg import NED, NavSts

class FishController(object):

	#topics to subscribe
	#rospy.Subscriber('ping_sensor', NED, #koju funkciju pozvat?)
	self.position = None
	self.newPosition = None
	self.depth = 5 #na randum napraviti depth isto
	
	rospy.Subscriber('position', NavSts, swim)

	#topics to publish (?)
	self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
	## closest position for the mussel !?
	
	#scenario functions

	def swim(self, msg):
		
		self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
		self.newPosition = main_controller.generate_available_position(self.position) #collision avoidance (?) # can be same position if depth isn't equal!
		action_library.send_depth_goal( self.stateRefPub, self.newPosition)
		
		
	

	rospy.spin()
	
import rospy
if __name__ == '__main__':
	rospy.init_node("fish_controller")
	try:
		controller = FishController()
	except rospy.ROSInterruptException:
		pass
