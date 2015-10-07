#!/usr/bin/env python

# This code was taken from the following website (below):
# https://www.fer.unizg.hr/_download/repository/lec04-ros-programming-python.pdf

import roslib
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan):
  rospy.loginfo(len(scan.ranges), min(scan.ranges))
  
def listener():
  rospy.init_node('laser_listener')
  rospy.Subscriber('scan', LaserScan, scan_callback)
  rospy.spin()
  
if __name__ == '__main__':
  listener()
