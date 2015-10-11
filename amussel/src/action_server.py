#!/usr/bin/python

import rospy
import actionlib
import math
from amussel.msg import aMusselAction, aMusselGoal, aMusselResult, aMusselFeedback
from auv_msgs.msg import NavSts, NED
from navcon_msgs.srv import EnableControl, ConfigureVelocityController

class aMusselActionServer:

    def __init__(self): 
        
        rospy.Subscriber('position', NavSts, self.position_cb)
        
        self.action_rec_flag = 0  # 0 - waiting action, 1 - received action, 2 - executing action
        self.as_goal = aMusselGoal()
        self.as_res = aMusselResult()
        self.as_feed = aMusselFeedback()
        self.position = None
        self.pos_err = []
        
        self.stateRefPub = rospy.Publisher('stateRef', NavSts, queue_size=1)
        
        self.action_server = actionlib.SimpleActionServer("action_server", aMusselAction, auto_start=False)
        self.action_server.register_goal_callback(self.action_execute_cb)
        self.action_server.register_preempt_callback(self.action_preempt_cb)
        self.action_server.start()

        while not rospy.is_shutdown():
            
            if self.action_server.is_active():
                while self.position is None:
                    rospy.sleep(0.1)
                
                if self.action_rec_flag == 1:  # received action
                    
                    if self.as_goal.id == 0:
                        rospy.logerr('change depth action %f %f', self.position.depth, self.as_goal.position.depth)
                        self.pos_old = NED(self.position.north, self.position.east, self.position.depth)
                        dl = abs(self.as_goal.position.depth - self.position.depth)
            
                        if dl < 0.1:
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            rospy.logerr('finished executing change_depth action')
                            self.action_server.set_succeeded(self.as_res)
                        else:
                            try:
                                depth_enable = rospy.ServiceProxy('DEPTH_enable', EnableControl)
                                depth_enable(enable=True)
    
                                velcon_enable = rospy.ServiceProxy('VelCon_enable', EnableControl)
                                velcon_enable(enable=True)
                                
                                config_vel_controller = rospy.ServiceProxy('ConfigureVelocityController', ConfigureVelocityController)
                                config_vel_controller(ControllerName="DEPTH", desired_mode=[0, 0, 2, 0, 0, 0])
    
                                stateRef = NavSts()
                                stateRef.header.seq = 0
                                stateRef.header.stamp.secs = 0
                                stateRef.header.stamp.nsecs = 0
                                stateRef.global_position.latitude = 0.0
                                stateRef.global_position.longitude = 0.0
                                stateRef.origin.latitude = 0.0
                                stateRef.origin.longitude = 0.0
                                stateRef.position.north = self.position.north
                                stateRef.position.east = self.position.east
                                stateRef.position.depth = self.as_goal.position.depth
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
                                self.action_rec_flag = 2
    
                            except rospy.ServiceException, e:
                                rospy.logerr("Failed to call change depth action" % e)
                                self.as_res.id = 0
                                self.as_res.status = -1
                                self.action_server.set_aborted(self.as_res)
                                self.action_rec_flag = 0  # waiting for new action

                elif self.action_rec_flag == 2:  # executing action

                    self.as_res.id = self.as_goal.id
                    self.as_feed.id = self.as_goal.id

                    if self.as_goal.id == 0:
                        dL = abs(self.as_goal.position.depth - self.pos_old.depth)
                        dl = abs(self.as_goal.position.depth - self.position.depth)
                        if len(self.pos_err) < 10:
                            self.pos_err.append(dl)
                        else:
                            self.pos_err.pop(0)
                            self.pos_err.append(dl)

                        if (len(self.pos_err) == 10) and (math.fabs(sum(self.pos_err) / len(self.pos_err)) < 0.05):  # mission is successfully finished
                            self.action_rec_flag = 0  # waiting for new action
                            self.as_res.status = 0
                            self.pos_err = []
                            rospy.logerr('finished executing change depth action')
                            self.action_server.set_succeeded(self.as_res)
                        else:  #mission is still ongoing
                            self.as_feed.status = (1 - dl / dL) * 100  # mission completeness
                            self.as_feed.position = self.position
                            self.action_server.publish_feedback(self.as_feed)
            else:
                pass

            rospy.sleep(rospy.Duration(0.05))

    def position_cb(self, msg):
        self.position = NED(msg.position.north, msg.position.east, msg.position.depth) 

    def action_execute_cb(self):
        rospy.logerr('Received action = ')
        self.action_rec_flag = 1
        self.as_goal = self.action_server.accept_new_goal()

    def action_preempt_cb(self):
        rospy.loginfo('Cancelling action')


if __name__ == '__main__':
    
    rospy.init_node('action_server', anonymous=True)
    try:
        aMusselActionServer()
    except rospy.ROSInterruptException:
        pass