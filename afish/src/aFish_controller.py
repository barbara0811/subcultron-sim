#!/usr/bin/env python

__author__= "Anja Zrnec"

import rospy
import random
import action_library # depth goal & new position
from auv_msgs.msg import NED, NavSts
from afish.msg  import positions

class FishController(object):
	
	def __init__(self):
	#topics to subscribe
		self.position = None
		self.newPosition = None
		self.fishPositions = None
		self.depth = 0
	
		rospy.Subscriber('position', NavSts, self.swim)
		rospy.Subscriber('positions', positions, self.setNewPositions)

		#topics to publish
		self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)

		rospy.spin()
	
	def swim(self, msg):
		
		self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
		self.newPosition = NED(msg.position.north + random.uniform(-1.0, 1.0), msg.position.east + random.uniform(-1.0, 1.0), msg.position.depth + random.uniform(-1.0, 1.0))
		safeToMove = positionAvailable(self.newPosition)
		if saveToMove:
			action_library.send_depth_goal( self.stateRefPub, self.newPosition)
		
	
	def positionAvailable(self, positionToCheck):

		if positionCheck in self.fishPositions:
			return true
		else: 
			return false
	
	def setNewPositions(self, msg):
		
		self.fishPositions = msg.positions
		print self.fishPositions[0]

if __name__ == '__main__':
	rospy.init_node("fish_controller")
	try:
		controller = FishController()
	except rospy.ROSInterruptException:
		pass
