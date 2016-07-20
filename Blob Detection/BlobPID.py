#!/usr/bin/env python

import rospy
import math
import numpy
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from racecar.msg import BlobDetections
from numpy.core.defchararray import lower

class BlobPID():
    
    e1=0
    e2=0
    #get the angle
    def getSteeringCmd(self,error,fullLeft,fullRight):
        Kp =.6
        Kd = .7
        de= ((self.e1-self.e2)+(error-self.e1))/.2
        self.e2=self.e1
        self.e1=error
        u=Kp*error+Kd*de
        return self.clip(fullLeft,fullRight,u)
    
    
    #passed to the subscriber
    def callback(self,msg):
        error=640-msg.locations.x
        if(error>10):
            self.drive_cmd.drive.steering_angle = 1
        elif(error<-10):
            self.drive_cmd.drive.steering_angle = -1
        else:
            self.drive_cmd.drive.steering_angle = 0
        
            
        self.drive.publish(drive_cmd) # post this message
    
    def __init__(self):
        #setup the node
        rospy.init_node('BlobPID', anonymous=False)
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        #sets the subscriber
        rospy.Subscriber('blob_detections', BlobDetections, self.callback2)
        
         # constant travel speed in meters/second
        speed = 0.0
        
        # fill out fields in ackermann steering message (to go straight)
        self.drive_cmd = AckermannDriveStamped()
        self.drive_cmd.drive.speed = speed
        rospy.spin()
def die():
    print "We dead"
    rospy.loginfo("I died")
try:
    BlobPID()
except Exception:
    die()