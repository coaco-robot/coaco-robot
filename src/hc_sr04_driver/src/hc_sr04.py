#!/usr/bin/env python
# Always use a voltage divider to connect the HC SR04 sensor to the Raspberry Pi

import rospy
from Bluetin_Echo import Echo
from std_msgs.msg import Float32

# Set pinout mode of the Raspberry Pi
TRIG_PIN = 23
ECHO_PIN = 24
SPEED_OF_SOUND = 343 
PRECISION = 2
FREQ = 5
NUMBER_OF_SAMPLES = 5
QUEUE_SIZE = 1

# Driver function for the HC SR04 sensor
def driver(echo):
    pub = rospy.Publisher("distance_sensor", Float32, queue_size=QUEUE_SIZE)
    rospy.init_node("hc_sr04", anonymous=True)

    # Keep polling the sensor and push the data to the ROS network
    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
        distance = round(echo.read("cm", NUMBER_OF_SAMPLES), PRECISION)
        rospy.loginfo("Distance: {} cm".format(distance))
        pub.publish(Float32(distance))
        rate.sleep()

if __name__ == "__main__":
    echo = Echo(TRIG_PIN, ECHO_PIN, SPEED_OF_SOUND)
    try:
        driver(echo)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt")
    finally:
        echo.stop()

