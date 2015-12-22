#!/usr/bin/env python

__author__ = "Anja Zrnec"

import rospy
from auv_msgs.msg import NED, NavSts
from afish.msg import positions
from afish.msg import positions
class PositionManager(object):

	def __init__(self):

		#add

		self.positionList = [NED(0, 0, 0)]*numberOfRobots

		# robot position subscribers


		# publishes list of positionsns
		self.list = rospy.Publish('positions', positions, queue_size = 1)

	# dinamical functions

	def publishNewPositions(self, msg, index):

		position = NED(msg.postion.north, msg.position.east, msg.position.depth)
		self.positionList[index] = position
		self.list.publish(self.positionList)

if __name__ == '__main__' :
	
	rospy.init_node('communication')

	try:
		communication = PositionManager()

	except rospy.ROSInterruptException:
