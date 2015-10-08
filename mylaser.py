#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import pdb
import os
import sys

class Scan_msg:

    def __init__(self):
	self.data = 0
	self.small_data = 0
	self.point_data = 0


    def sort(self, laserscan):
        #pdb.set_trace()
	data = laserscan.ranges
	small_data = list(data[314:324])
	length = len(small_data)
        #print length
	for entry in range(0,length-1):
            #print small_data[entry]
	    if math.isnan(float(small_data[entry])) :
	       del small_data[entry]
	    else :
	        a = 0
        #pdb.set_trace()
	point_data = float(sum(small_data))/len(small_data)
	#print point_data
	rospy.loginfo("Avg of The DATA " + str(point_data))

    def for_callback(self,laserscan):
	self.sort(laserscan)

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
