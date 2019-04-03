#!/usr/bin/env python

import rospy
import math
from rosmovement.msg import movMsg
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

class Movement:
    def __init__(self):
        self.velocity_publisher=False
        self.is_simulation=False
        self.anglefactor = 0.45
        self.distancefactor = 0.04/8.8
        self.speed = 0.1
        self.rspeed = 30
        rospy.init_node('rosmovement', anonymous=True)
        rospy.Subscriber("move_it", movMsg, self.processMovement, queue_size=1)
        if self.is_simulation:
            self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        else:
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.grabber_publisher = rospy.Publisher('grabber', UInt8, queue_size=10)
        rospy.spin()

    def processMovement(self, msg):
        distance = msg.distance * self.distancefactor
        angle = msg.angle * self.anglefactor

        vel_msg = Twist()
        grab_msg = UInt8()

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        if msg.open == 2: # Open the gripper
            grab_msg.data = 60
            self.grabber_publisher.publish(grab_msg)
        elif msg.open == 1: # Close the gripper
            grab_msg.data = 81
            self.grabber_publisher.publish(grab_msg)

        angular_speed = self.rspeed*2*math.pi/360
        relative_angle = abs(angle)*2*math.pi/360
        if (angle!=0):
            if (angle < 0):
                vel_msg.angular.z = -abs(angular_speed)
            if (angle > 0):
                vel_msg.angular.z = abs(angular_speed)
        
            current_angle = 0
            t0 = rospy.Time.now().to_sec()
            while(current_angle < relative_angle):
                self.velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed*(t1-t0)

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
        if (distance!=0):
            if (distance < 0):
                vel_msg.linear.x = -self.speed
            if (distance > 0):
                vel_msg.linear.x = self.speed
    
            current_distance = 0
            t0 = rospy.Time.now().to_sec()
            while(abs(current_distance) < abs(distance)):
                self.velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_distance = self.speed * (t1 - t0)

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        mov = Movement()
    except rospy.ROSInterruptException: pass
