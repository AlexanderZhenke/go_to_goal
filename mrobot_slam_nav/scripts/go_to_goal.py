#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point,Quaternion,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from random import sample
from math import pow,sqrt
class NavTest():
    def __init__(self):
        #init node
        rospy.init_node('exploring_slam',anonymous=True)
        rospy.on_shutdown(self.shutdown)
        # stop time in target point(s)
        self.rest_time = rospy.get_param("~rest_time", 2)  
        # whether simulink  
        self.fake_test = rospy.get_param("~fake_test", True)  
        # goal state  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
        # set goal position  
        # click 2D Nav Goal in rviz，the click map in anywhere   
        locations = dict()  
        #https://www.jianshu.com/p/17c016778879
        locations['1'] = Pose(Point(4.600, -3.2700, 0.000),  Quaternion(0.000, 0.000, -0.447, 0.894))  
        #locations['2'] = Pose(Point(4.231, -6.050, 0.000),  Quaternion(0.000, 0.000, -0.847, 0.532))  
        #locations['3'] = Pose(Point(-0.674, -5.244, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
        #locations['4'] = Pose(Point(-5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))  
        #locations['5'] = Pose(Point(-4.701, -0.590, 0.000), Quaternion(0.000, 0.000, 0.340, 0.940))  
        #locations['6'] = Pose(Point(2.924, 0.018, 0.000),   Quaternion(0.000, 0.000, 0.000, 1.000))  

        #publish the information of control
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist,queue_size=5)
        #subcribe move_base server msg
        self.move_base = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        #waiting
        self.move_base.wait_for_server(rospy.Duration(60))   
        #save the init position in rviz
        initial_pose = PoseWithCovarianceStamped()
        #save succeed  runtime and distance value
        n_locations = len(locations)  
        n_goals = 0  
        n_successes = 0  
        i = n_locations  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = "" 

        #make sure have init position
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        #main loop
        while not rospy.is_shutdown():
            location = '1' 
            distance = sqrt(pow(locations[location].position.x -   
                               initial_pose.pose.pose.position.x, 2) +  
                            pow(locations[location].position.y -   
                                initial_pose.pose.pose.position.y, 2)) 
            #set next target point
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now() 

            # move  
            self.move_base.send_goal(self.goal)  

            # time limit (s)
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   

            # check whether the target point has been reached  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")   
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state])) 

            #run time
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0   

            rospy.sleep(self.rest_time) 

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose 

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1) 

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")  