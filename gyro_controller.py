#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import cmath


class Odo_msg:

    def __init__(self):
	'''Initializes an object of this class.
	The constructor creates a publisher, a twist message.'''
	self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
	self.msg = Twist()
      	self.msg.angular.z = 0
	self.msg.linear.x = 0 
	self.theta_err = 0
	self.heading = 150  #desired heading in degrees
	self.k = .5  #proportionality constant
	self.saturation = 1


    def process(self, data):
	'''Processes gyro's data.
	Complex number is created from quaternion and angle is corrected'''
	z = complex(data.pose.pose.orientation.w, data.pose.pose.orientation.z) #constructs complex number from quaternion
	z_curr = z*z #angle must be doubled
	self.theta_err = math.radians(self.heading) - cmath.phase(z_curr) 
	rospy.loginfo("theta_err = " + str(self.theta_err))

    def movement(self, error): 
	'''Sets robot's angular velocity based on error.
	Deadzone and saturation implemented before publishing'''
        #deadzone
        if abs(error) < .05 :
        	error = 0
        u = self.k*error
        if u > self.saturation :
           self.msg.angular.z = self.saturation
        elif u < -self.saturation :
             self.msg.angular.z = -self.saturation
        else :
            self.msg.angular.z = u
	self.pub.publish(self.msg) 
	 
    def for_callback(self,data):
	self.process(data)
        self.movement(self.theta_err)

def call_back(odomsg):
    '''Passes Odometry message to for_callback function of sub_obj.
    Parameter odomsg is Odometry message.'''
    sub_obj.for_callback(odomsg)

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('gryo_node')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber("/odom", Odometry, call_back)
    rospy.spin()

if __name__ == "__main__":
    '''An Odo_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Odo_msg()
    listener()
 
