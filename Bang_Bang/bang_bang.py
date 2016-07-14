#!/usr/bin/env python
# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan, Joy # joystick and laser scanner msgs
class WallE():
    
    # variable to return distance to wall from callback
    # (at 90 and 60 degrees to left of vehicle)
    #scan = [0.0, 0.0] # Ensure it is created before being used
    steer=0
    
    def getError(self,goal,L,begin,end):
        return goal-(min(L[begin:end]))
    
    #get the angle
    def getSteeringCmd(self,error,threshold,fullLeft,fullRight):
        if(abs(error)<threshold):
            return 0
        elif error>0:
            return fullLeft
        else:
            return fullRight
    
    #passed to the subscriber
    def callback(self,msg):
        #get the laser information
        error=self.getError(.4, msg.ranges, 670, 1080)
        angle=self.getSteeringCmd(error, .03, 1, -1)
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)
    
    def __init__(self):
        #setup the node
        rospy.init_node('wall_bang', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # output messages/sec (also impacts latency)
        rate = 10 
        r = rospy.Rate(rate)
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=5)
        
        #sets the subscriber
        rospy.Subscriber('scan', LaserScan, self.callback)
        
        # set control parameters
        speed = 0.5 # constant travel speed in meters/second
        dist_wall = 0.333 # start and stay at ~1ft from wall
        dist_trav = 5.0 # meters to travel in time travel mode
        
        # fill out fields in ackermann steering message (to go straight)
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = 0.0
        #r60 = dist_wall / math.cos(math.radians(30.0)) # expected distance if parallel to wall
        
        # assume correct vehicle pose (at start)
        #scan = [dist_wall, r60] 
        
        # main processing loop (runs for pre-determined duration in time travel mode)
        time = dist_trav / speed
        ticks = int(time * rate) # convert drive time to ticks
        for t in range(ticks):
            # bang-bang controller for steering direction (single point version - don't use)
            # turn full left (1.0) or right (-1.0)
            #drive_cmd.drive.steering_angle = math.copysign(1.0,-(scan[1]-r60))
            drive_cmd.drive.steering_angle=self.steer
            
            self.drive.publish(drive_cmd) # post this message
            #Chill out for a bit
            r.sleep() 
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        
        
    
        
        
if __name__ == '__main__':
    try:
        WallE()
    except:
        rospy.loginfo("WallE node terminated.")
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
    
