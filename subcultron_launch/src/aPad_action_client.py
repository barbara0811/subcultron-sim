#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
from apad.msg import aPadAction, aPadGoal
from geometry_msgs.msg import Pose, Point

def action_client():

    print 'Starting client'
    client = actionlib.SimpleActionClient('/apad1/action_server', aPadAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    print 'Connected to server'

    # Creates a goal to send to the action server.
    goal_pose = Pose()
    goal_pose.position = Point(0,0,0)
    goal = aPadGoal(id=0, pose=goal_pose)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    
    client.wait_for_result()
    # Prints out the result of executing the action
    print client.get_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('apad_action_client')
        result = action_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
