#!/usr/bin/env python

import rospy
from rosmovement.msg import movMsg
from geometry_msgs.msg import Twist


velocity_publisher=False
simulation=True
anglefactor = 0.5
distancefactor = 0.02
 

def processMovement(msg):
    global distantancefactor
    global anglefactor
    PI = 3.1415926535897
    global velocity_publisher
    global simulation
    # Receiveing the user's input
    speed = 1
    rspeed = 30
    distance = msg.distance * distancefactor
    angle = msg.angle * anglefactor

    vel_msg = Twist()

    # Since we are moving just in x-axis
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    angular_speed = rspeed*2*PI/360
    relative_angle = abs(angle)*2*PI/360
    if (angle!=0):
        if (angle < 0):
            vel_msg.angular.z = -abs(angular_speed)
        if (angle > 0):
            vel_msg.angular.z = abs(angular_speed)
        
        current_angle = 0
        t0 = rospy.Time.now().to_sec()
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    
    if (distance!=0):
        if (distance < 0):
            vel_msg.linear.x = -speed
        if (distance > 0):
            vel_msg.linear.x = speed
    
        current_distance = 0
        t0 = rospy.Time.now().to_sec()
        while(abs(current_distance) < abs(distance)):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)


    # After the loop, stops the robot
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def move():
    # Starts a new node
    rospy.init_node('rosmovement', anonymous=True)
    rospy.Subscriber("move_it", movMsg, processMovement, queue_size=1)
    global velocity_publisher 
    if simulation:
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    else:
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        move()
    except rospy.ROSInterruptException: pass
