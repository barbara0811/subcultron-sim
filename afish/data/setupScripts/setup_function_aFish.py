#!/usr/bin/env python

__author__ = "Anja Zrnec"

import sys
import rospy
import rospkg

functionsLine = '# robot position subscribers'
add = '#add'
dinamicalFunctions = '# dinamical functions'

def appendFunctionsToFile(numberOfRobots):

	f = open(rospack.get_path('afish') + '/src/communication_raw.py', 'r')
	fNew = open(rospack.get_path('afish') + '/src/communication.py', 'a')
	fNew.seek(0)
	fNew.truncate()
	for line in f:
		fNew.write(line)
		line = line.strip()
		if line == functionsLine:
			appendSubscriberText(fNew, numberOfRobots)
		if line == add:
			numberOfFish(fNew, numberOfRobots)
		if line == dinamicalFunctions:
			appendFunctions(fNew, numberOfRobots)
	f.close()


def appendSubscriberText(f, numberOfRobots):

	for i in range( 0, numberOfRobots):
		f.write("		rospy.Subscriber('aFish" + str(i+1) + "/position', NavSts, self.updatePosition" + str(i+1) + ")\n")


def numberOfFish(f, numberOfRobots):

	f.write('		self.numberOfRobots = ' + str(numberOfRobots))

def appendFunctions(f, numberOfRobots):

	for i in range( 0, numberOfRobots):
		f.write( '	def updatePosition' + str(i+1) + '(self, msg):\n')
		f.write ('		index = ' + str(i+1) + '\n')
		f.write ('		self.publishNewPosition(msg, index)\n\n')

if __name__ == '__main__':

	if len(sys.argv) < 1:
		print "missing an argument"
		sys.exit(0)

	rospack = rospkg.RosPack()
	numberOfRobots = int(sys.argv[1])
	print numberOfRobots
	appendFunctionsToFile(numberOfRobots)
