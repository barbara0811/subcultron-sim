        
import rospy
import actionlib
from apad.msg import aPadAction, aPadGoal
from geometry_msgs.msg import Pose, Point

class actionClient(object):

    def __init__(self, namespace):
        self.client = actionlib.SimpleActionClient(namespace + 'action_server', aPadAction)
        self.feedback = 0

    def send_position_goal(self, position):

        self.client.wait_for_server()

        print 'Connected to server'

        # Creates a goal to send to the action server.
        goal_pose = Pose()
        goal_pose.position = position
        goal = aPadGoal(id=0, pose=goal_pose)
        # Sends the goal to the action server.
        self.client.send_goal(goal, None, None, self.feedback_cb)
        # Waits for the server to finish performing the action.
        
        self.client.wait_for_result()
        # Prints out the result of executing the action
        print self.client.get_result()
        return self.client.get_result()

    def feedback_cb(self, feedback):
        if feedback.status - self.feedback >= 5:
            #print " " + str(feedback.status) + "%"
            self.feedback = feedback.status
