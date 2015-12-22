#!/usr/bin/env python

__author__ = "Anja Zrnec"

import rospy
from auv_msgs.msg import NED, NavSts
from afish.msg import positions
import time

class PositionManager(object):

	def __init__(self):

		#add
		self.numberOfRobots = 2
		self.positionList = [NED(0,0,0)]*self.numberOfRobots
		print self.positionList
		# robot position subscribers
		rospy.Subscriber('aFish1/position', NavSts, self.updatePosition1)
		rospy.Subscriber('aFish2/position', NavSts, self.updatePosition2)


		# publishes list of positions
		self.list = rospy.Publisher('positions', positions, queue_size = 1)
		
		#test
#		while True:
#			self.publishNewPositions(NED( 1.0, 2.0, 3.0) , 0)
#			time.sleep(5)

	# dinamical functions
	def updatePosition1(self, msg):
		index = 1
		self.publishNewPosition(msg, index)

	def updatePosition2(self, msg):
		index = 2
		self.publishNewPosition(msg, index)

	def updatePosition1(self, msg):
		index = 1
		self.publishNewPosition(msg, index)

	def publishNewPositions(self, msg, index):

		position = NED(msg.position.north, msg.position.east, msg.position.depth)
#		position = NED(msg.north, msg.east, msg.depth)
#		print position
		self.positionList[index] = position
#		print self.positionList
		self.list.publish(self.positionList)

if __name__ == '__main__' :
	
	rospy.init_node('communication')

	try:
		communication = PositionManager()

	except rospy.ROSInterruptException:
		pass
