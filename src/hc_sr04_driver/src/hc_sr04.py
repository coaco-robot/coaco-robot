#!/usr/bin/env python
# Always use a voltage divider to connect the HC SR04 sensor to the Raspberry Pi

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32

# Set pinout mode of the Raspberry Pi
GPIO.setmode(GPIO.BCM)
TRIG_PIN = 23
ECHO_PIN = 24
SETTLE_TIME = 2
ECHO_PULSE_TIME = 0.00001
SPEED_OF_SOUND = 34300/2 # 0.5 speed of sound, as required by the formula
PRECISION = 2
QUEUE_SIZE = 10
FREQ = 5

# Driver function for the HC SR04 sensor
def driver():
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    time.sleep(SETTLE_TIME)
    pub = rospy.Publisher("distance_sensor", Float32, queue_size=QUEUE_SIZE)
    rospy.init_node("hc_sr04", anonymous=True)

    # Keep polling the sensor and push the data to the ROS network
    rate = rospy.Rate(FREQ)
    while not rospy.is_shutdown():
        # Send TRIGGER puls
        GPIO.output(TRIG_PIN, True)
        time.sleep(ECHO_PULSE_TIME)
        GPIO.output(TRIG_PIN, False)

        # Measure the time of the ECHO
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

        # Calculate the distance using the distance of sound
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * SPEED_OF_SOUND
        distance = round(distance, PRECISION)
        rospy.loginfo("Distance: {} cm".format(distance))
        pub.publish(Float32(distance))
        rate.sleep()

if __name__ == "__main__":
    try:
        driver()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt")
    finally:
        GPIO.cleanup()

