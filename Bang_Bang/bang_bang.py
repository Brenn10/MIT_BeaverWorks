#!/usr/bin/python

import rospy
from ackermann_msgs import AckermannDrive
from sensor_msgs import LaserScan

#this holds the answer from the call back each time its called
global angle=0

#gets the error
def getError(goal,L,begin,end):
    return goal-(min(L[begin:end]))

#get the angle
def getSteeringCmd(error,threshold,fullLeft,fullRight):
    if(abs(error)<threshold):
        return 0
    elif error>0:
        return fullLeft
    else:
        return fullRight

#passed to the subscriber
def callback(msg):
    #get the laser information
    error=getError(.4, msg.ranges, 670, 1080)
    angle=getSteeringCmd(error, .03, 1, -1)
    

def converter():
    rospy.init_node('bang_bang_node')
    sub = rospy.Subscriber('scan',LaserScan,callback)
    pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation",AckermannDrive)
    rate=rospy.rate(10)
    while not rospy.is_shutdown():
        msg=AckermannDrive()
        msg.speed=1.0
        msg.steering_angle=angle
        
        pub.publish(msg)
        
        rate.sleep()

     
if __name__ == "__main__":
    try:
        converter()
    except rospy.ROSInterruptException:
        pass
        
    
    
    