#!/usr/bin/env python

__author__ = "Anja Zrnec"

import sys
import rospy
import rospkg

functionsLine = '# robot position subscribers'
add = '#add'
dinamicalFunctions = '# dinamical functions'

def appendFunctionsToFile(f, numberOfRobots):

	lineCounter = 0
	for line in f:
		lineCounter += 1
		line = line.rstrip()
		if line == functionsLine:
			appendSubscriberText(lineCounter, numberOfRobots)
		if line == add:
			numberOfFish(lineCounter, numberOfRobots)
		if line == dinamicalFunctions:
			appendFunctions(lineCounter, numberOfRobots)
	f.close()


def appendSubscriberText(index, numberOfRobots):

	f.seek(index)
	for i in numberOfRobots:
		f.write("rospy.Subscriber(aFish" + i + "'/position', NavSts, self.updatePosition" + i + ")\n")

def numberOfFish(index, numberOfRobots):

	f.seek(index)
	f.write('self.numberOfRobots = ' + numberOfRobots)

def appendFunctions(index, numberOfRobots):

	f.seek(index)
	for i in numberOfRobots:
		f.write( 'def updatePosition' + i + '(self, msg):')
		f.write ('index = ' + i)
		f.write ('self.publishNewPosition(msg, index)')

if __name__ == '__main__':

	if len(sys.argv) < 1:
		print "missing an argument"
		sys.exit(0)

	rospack = rospkg.RosPack()
	numberOfRobots = int(sys.argv[1])
	print numberOfRobots
	f = open(rospack.get_path('afish') + '/src/communication.py', 'a+r')
	appendFunctionsToFile(f, numberOfRobots)
