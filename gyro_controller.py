#!/usr/bin/env python

### GETTING THE CODE TO WORK ####

# STEP 1: ENTER "roslaunch turtlebot_bringup minimal.launch" in a separate terminal window (turns on turtlebot)

# STEP 2: ENTER "roslaunch turtlebot_navigation gmapping_demo.launch" in another separate terminal window (activates Laserscan device)

# STEP 3: RUN THE CODE

import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import cmath
import pdb
import os
import sys

class Bump_msg:

    def __init__(self):
	'''Initializes an object of this class.
	The constructor creates a publisher, a twist message.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	self.msg = Twist()
      	self.msg.angular.z = 0
	self.msg.linear.x = 0 # Sets the velocity of the robot
	self.state = 0


    def sort(self, data): # Processes the Kinetic's laserscan data
        a = data.pose.pose.orientation.w
	b = data.pose.pose.orientation.z
	z = complex(a,b)
	z = z*z
	print cmath.phase(z)

        #rospy.loginfo("Button state: " + str(self.state))

    def movement(self): # Responsible for setting the velocity of the robot	
	    self.pub.publish(self.msg) 
	    print "publishing"	  	
	
    def for_callback(self,data): # Loops through the laserscan data and the movement data

	self.sort(data)
        self.movement()

#################################################################

def call_back(odomsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(odomsg)

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('gryo_node')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber("/odom", Odometry, call_back)
    rospy.spin()

################################

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Bump_msg()
    listener()
 
