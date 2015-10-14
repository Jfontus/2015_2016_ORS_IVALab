#!/usr/bin/env python

### GETTING THE CODE TO WORK ####

# STEP 1: ENTER "roslaunch turtlebot_bringup minimal.launch" in a separate terminal window (turns on turtlebot)

# STEP 2: ENTER "roslaunch turtlebot_navigation gmapping_demo.launch" in another separate terminal window (activates Laserscan device)

# STEP 3: RUN THE CODE

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import pdb
import os
import sys
import numpy as np

class Scan_msg:

    def __init__(self):
	'''Initializes an object of this class.
	The constructor creates a publisher, a twist message.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	self.msg = Twist()
      	self.msg.angular.z = 0
	self.msg.linear.x = 0 # Sets the velocity of the robot
	self.point_data = 0 # Stores laserscan data
        self.dist_setter = 1 # Sets the desired robot distance
        self.error = 0 # Stores the desired distance error
        self.max_forward = 0.5 # Sets the maximum forward velocity
        self.max_backward = -0.5 # Sets the maximum backwards velocity

    def sort(self, laserscan): # Processes the Kinetic's laserscan data
        #pdb.set_trace()
	data = laserscan.ranges
	small_data = np.array(list(data[314:324]))
        # Calculates the average while ignoring Nan values using numpy 
        # library
        self.point_data = small_data[~np.isnan(small_data)].mean()
        rospy.loginfo("avg: " + str(self.point_data))

    def movement(self, pointdata): # Responsible for setting the velocity of the robot

 	if (pointdata - self.dist_setter) < 0 :
	    self.error = pointdata - self.dist_setter
            self.movement_sat(self.error)

	elif (pointdata - self.dist_setter) > 0 :
	    self.error = pointdata - self.dist_setter
            self.movement_sat(self.error)

        else:
	    self.msg.linear.x = 0
	self.pub.publish(self.msg)

    def movement_sat(self,error): # Prevents the robot from moving at a dangerous velocity

	if error > 0: #Moving forward saturation control
            if error < self.max_forward:
               self.msg.linear.x = error
            else:
               self.msg.linear.x = self.max_forward
        elif error < 0: # Moving backwards Saturation Control
            if error > self.max_backward:
               self.msg.linear.x = error
            else:
               self.msg.linear.x = self.max_backward

	
    def for_callback(self,laserscan): # Loops through the laserscan data and the movement data

	self.sort(laserscan)
        self.movement(self.point_data)

#################################################################

def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    rospy.spin()

################################

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()
