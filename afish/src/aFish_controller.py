#!/usr/bin/env python

__author__= "Anja Zrnec"

import rospy
import random
import action_library
from auv_msgs.msg import NED, NavSts
from afish.msg  import positions

class FishController(object):
	
	def __init__(self):
	#topics to subscribe
		self.position = None
		self.newPosition = None
		self.fishPositions = [NED(0,0,0)]*2
		self.depth = 0
	
		rospy.Subscriber('position', NavSts, self.swim)
		rospy.Subscriber('positions', positions, self.setNewPositions)

		#topics to publish
		self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)

		rospy.spin()
	
	def swim(self, msg):
		
		self.position = NED(msg.position.north, msg.position.east, msg.position.depth)
		self.newPosition = NED(msg.position.north + random.uniform(-1.0, 1.0), msg.position.east + random.uniform(-1.0, 1.0), msg.position.depth + random.uniform(-1.0, 1.0))
		self.safeToMove = self.positionAvailable(self.newPosition)
		if self.safeToMove:
			action_library.send_depth_goal(self.stateRefPub, self.newPosition)
			print self.newPosition
		
	
	def positionAvailable(self, positionToCheck):
		
		if positionToCheck in self.fishPositions:
			return False
		else: 
			return True
	
	def setNewPositions(self, msg):
		
		self.fishPositions = msg.positions
#		print self.fishPositions

if __name__ == '__main__':
	rospy.init_node("fish_controller")
	try:
		controller = FishController()
	except rospy.ROSInterruptException:
		pass
