#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import pdb
import os
import sys

class Scan_msg:

    def __init__(self):
	'''Initializes an object of this class.
	The constructor creates a publisher, a twist message.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	self.msg = Twist()
      	self.msg.angular.z = 0
	self.msg.linear.x = .2
	self.point_data = 0

    def sort(self, laserscan):
        #pdb.set_trace()
	data = laserscan.ranges
	small_data = list(data[314:324])
	entry = 0
        #print length
	while entry<len(small_data):
            #print small_data[entry]
	    if math.isnan(float(small_data[entry])) :
	       del small_data[entry]
	    else :
	        entry+=1
        #pdb.set_trace()
	if len(small_data)!= 0:
	   self.point_data = float(sum(small_data))/len(small_data)
	else :
	    print "NOT READING"	
	#print point_data
	rospy.loginfo("avg: " + str(self.point_data))

    def movement(self, pointdata):
 	if pointdata < .65 :
	   self.msg.linear.x = 0
	else :
	    self.msg.linear.x = .1
	self.pub.publish(self.msg)
	
    def for_callback(self,laserscan):
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
