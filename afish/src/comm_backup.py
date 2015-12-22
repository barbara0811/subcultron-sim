#!/usr/bin/env python

__author__ = "Anja Zrnec"

import rospy
from auv_msgs.msg import NED, NavSts

class PositionManager(object):

	def __init__(self):

		#add
		self.numberOfRobots = 0
		self.positionList = [None]*numberOfRobots

		# robot position subscribers

		rospy.Subscriber('aFish1/position', NavSts, self.updatePosition1)

		# publishes list of positions
		# TODO new type of msg for list of positions
		self.list = rospy.Publish('/positions', Positions, queue_size = 1)

	# dinamical functions
	def updatePosition1(self, msg):
		index = 1
		self.publishNewPosition(msg, index)

	def publishNewPositions(self, msg, index):

		position = NED(msg.postion.north, msg.position.east, msg.position.depth)
		self.positionList.insert(index, position)
		self.list.publish(positionList)

if __name__ == '__main__' :
	
	rospy.init_node('communication')

	try:
		communication = PositionManager()

	except rospy.ROSInterruptException:
		pass
