#!/usr/bin/python

import rospy
import math
from apad.msg import aPadAction, aPadGoal, aPadResult, aPadFeedback
from geometry_msgs.msg import Point, Twist, Vector3, TwistStamped, PoseStamped, Pose, PoseWithCovarianceStamped
from auv_msgs.msg import NavSts, NED

from navcon_msgs.srv import EnableControl, EnableControlRequest, ConfigureVelocityController

import actionlib

class aPadActionServer:

    def __init__(self):
        
        # position
        rospy.Subscriber('position', NavSts, self.position_cb)

        # state reference publisher
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        self.action_rec_flag = 0  # 0 - waiting action, 1 - received action, 2 - executing action
        
        self.as_goal = aPadGoal()
        self.as_res = aPadResult()
        self.as_feed = aPadFeedback()

        self.position = Point(0, 0, 0)
        self.position_offset = Point(-0.5, -0.5, 0)  # for docking -- object offset 

        self.pos_old = Point(self.position.x, self.position.y, self.position.z)
        
        self.pos_err = []
        self.perched_flag = 0  # flag for dummy action perching onto aMussel/aFish action
        
        self.action_server = actionlib.SimpleActionServer("action_server", aPadAction, auto_start=False)
        self.action_server.register_goal_callback(self.action_execute_cb)
        self.action_server.register_preempt_callback(self.action_preempt_cb)
        self.action_server.start()

        while not rospy.is_shutdown():
            # goal position for aMussel/aFish docked onto aPad
            goalPosition = NED()
            goalPosition.north = self.position.x + self.position_offset.x
            goalPosition.east = self.position.y + self.position_offset.y
            #goalPosition.z = self.position.z + self.position_offset.z

            if self.perched_flag == 1:
                self.set_model_state(goalPosition)

            if self.action_server.is_active():

                ###################
                # received action #
                ###################

                if self.action_rec_flag == 1:
                
                    if self.as_goal.id == 0:
                        print 'Go to position action'
                        self.pos_old = Point(self.position.x, self.position.y, 0)
                        start = Vector3(self.position.x, self.position.y, 0)
                        end = Vector3(self.as_goal.pose.position.x, self.as_goal.pose.position.y, 0)

                        dl = math.sqrt(math.pow(self.as_goal.pose.position.x - self.position.x, 2) +
                                        math.pow(self.as_goal.pose.position.y - self.position.y, 2))

                        if dl < 0.1:
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            print 'finished executing go_to_position action'
                            print "###"
                            print self.position
                            print "###"
                            self.action_server.set_succeeded(self.as_res)
                        else:
                            # send goal message
                            try:
                                # send goal
                                fadp_enable = rospy.ServiceProxy('FADP_enable', EnableControl)
                                fadp_enable(enable=True)

                                config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
                                config_vel_controller(ControllerName="FADP", desired_mode=[2, 2, 0, 0, 0, 0])

                                velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
                                velcon_enable(enable=True)
        
                                stateRef = NavSts()
                                stateRef.header.seq = 0
                                stateRef.header.stamp.secs = 0
                                stateRef.header.stamp.nsecs = 0
                                stateRef.global_position.latitude = 0.0
                                stateRef.global_position.longitude = 0.0
                                stateRef.origin.latitude = 0.0
                                stateRef.origin.longitude = 0.0
                                stateRef.position.north = self.as_goal.pose.position.x
                                stateRef.position.east = self.as_goal.pose.position.y
                                stateRef.position.depth = 0
                                stateRef.altitude = 0.0
                                stateRef.body_velocity.x = 0
                                stateRef.body_velocity.y = 0
                                stateRef.body_velocity.z = 0
                                stateRef.gbody_velocity.x = 0
                                stateRef.gbody_velocity.y = 0
                                stateRef.gbody_velocity.z = 0
                                stateRef.orientation.roll = 0
                                stateRef.orientation.pitch = 0
                                stateRef.orientation.yaw = 0
                                stateRef.orientation_rate.roll = 0
                                stateRef.orientation_rate.pitch = 0
                                stateRef.orientation_rate.yaw = 0
                                stateRef.position_variance.north = 0
                                stateRef.position_variance.east = 0
                                stateRef.position_variance.depth = 0
                                stateRef.orientation_variance.roll = 0
                                stateRef.orientation_variance.pitch = 0
                                stateRef.orientation_variance.yaw = 0
                                stateRef.status = 0
                                self.stateRefPub.publish(stateRef)
                                
                                self.action_rec_flag = 2  #executing trajectory

                            except rospy.ServiceException, e:
                                print "Service gen_and_exe_traj call failed: %s" % e
                                self.as_res.status = -1
                                self.action_server.set_aborted(self.as_res)
                                self.action_rec_flag = 0  # waiting for new actions

                    elif self.as_goal.id == 1:
                        print 'perch action'
                        self.action_rec_flag = 2  # start executing action

                    elif self.as_goal.id == 2:
                        print 'release action'
                        self.action_rec_flag = 2  # start executing action

                ####################
                # executing action #
                ####################

                elif self.action_rec_flag == 2:

                    self.as_res.id = self.as_goal.id
                    self.as_feed.id = self.as_goal.id

                    if self.as_goal.id == 0:
                        # Go to position action
                        dL = math.sqrt(math.pow(self.as_goal.pose.position.x - self.pos_old.x, 2) +
                                       math.pow(self.as_goal.pose.position.y - self.pos_old.y, 2))
                        dl = math.sqrt(math.pow(self.as_goal.pose.position.x - self.position.x, 2) +
                                       math.pow(self.as_goal.pose.position.y - self.position.y, 2))

                        if len(self.pos_err) < 20:
                            self.pos_err.append(dl)
                        else:
                            self.pos_err.pop(0)
                            self.pos_err.append(dl)

                        if (len(self.pos_err) == 20) and (math.fabs(sum(self.pos_err) / len(self.pos_err)) < 0.5):  # mission is successfully finished
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            print 'finished executing go_to_position action'
                            self.action_server.set_succeeded(self.as_res)
                        else:  # mission is still ongoing
                            self.as_feed.status = (1 - dl / dL) * 100  # mission completeness
                            self.as_feed.pose.position = self.position
                            #print [math.fabs(sum(self.pos_err) / len(self.pos_err)), str(self.position), str(self.as_goal.pose.position)]
                            self.action_server.publish_feedback(self.as_feed)

                    elif self.as_goal.id == 1:
                        print 'executing perch action'
                        self.perched_flag = 1
                        # static position publisher
                        self.staticPosPub = rospy.Publisher('/' + self.as_goal.object + '/position_static', NavSts, queue_size=1)
                        print 'finished executing perch action ' + self.as_goal.object
                        self.action_rec_flag = 0  # waiting for new action
                        self.as_res.status = 0
                        self.action_server.set_succeeded(self.as_res)

                    elif self.as_goal.id == 2:
                        print 'executing release action'
                        self.perched_flag = 0
                        msg = NavSts()
                        msg.position = NED(self.position.x, self.position.y, 0)
                        msg.status = 1
                        self.staticPosPub.publish(msg) # signal the end of static position to attached object
                        print 'finished executing release action'
                        self.action_rec_flag = 0  # waiting for new action
                        self.as_res.status = 0
                        self.action_server.set_succeeded(self.as_res)
            else:
                pass

            rospy.sleep(rospy.Duration(0.05))

    def set_model_state(self, position):

        msg = NavSts()
        msg.position = position
        self.staticPosPub.publish(msg)

    def position_cb(self, msg):    
        self.position.x = msg.position.north
        self.position.y = msg.position.east
        self.position.z = msg.position.depth

    def action_execute_cb(self):
        print 'Received action = '
        self.action_rec_flag = 1
        self.as_goal = self.action_server.accept_new_goal()

    def action_preempt_cb(self):
        print 'Cancelling action'
        # send cancel request to mission planner by sending start = 0, end = 0
        start = Vector3(0, 0, 0)
        end = Vector3(0, 0, 0)

        try:
            traj_srv = rospy.ServiceProxy('gen_and_exe_traj', PlanPath)
            traj_resp = traj_srv(start, end, 0)

        except rospy.ServiceException, e:
            print "Failed to revoke trajectory" % e

        # hold current position
        twist = Twist()
        twist.linear.x = self.position.x
        twist.linear.y = self.position.y
        twist.linear.z = self.position.z
        self.traj_pub.publish(twist)

        self.action_rec_flag = 0  # wait for new action
        self.action_server.set_preempted()


if __name__ == '__main__':
    # Initialize action server node
    rospy.init_node('action_server', anonymous=True)
    try:
        aPadActionServer()
    except rospy.ROSInterruptException:
        pass
        