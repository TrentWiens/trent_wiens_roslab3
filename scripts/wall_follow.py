#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1#TODO
kd = 1#TODO
ki = 1#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.

	ranges = data.ranges
	
	rotAngle = angle + 90
	
	if rotAngle >= 360:
		rotAngle = rotAngle - 360
	elif rotAngle < 0:
		rotAngle = rotAngle + 360
	
	angleRatio = rotAngle/360.0
	rangeIndex = angleRatio * 1080
	rangeIndex = rangeIndex + 1
	rangeIndex = int(rangeIndex)
	outRange = ranges[rangeIndex]
	
	if not math.isnan(outRange):
        	return outRange
        else:
        	return 0.0

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        
        theta = 45
        thetar = 45*math.pi/180
        
        a = self.getRange(data, theta)
        b = self.getRange(data, 0)
        
        ctheta = math.cos(thetar)
        stheta = math.sin(thetar)
        
        num = a * ctheta - b
        denom = a * stheta
        
        alpha = math.atan2(num, denom)
        
        Dt = b * math.cos(alpha)
        
        print(Dt)
        
        return 0.0 

    def lidar_callback(self, data):
        """ 
        """
	dist = self.followLeft(data,1)
        error = 0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
