#!/usr/bin/env python
import rospy
import time
import std_msgs
from constants import *
from state_machine import StateMachine
from can_detector.msg import CanDetection
from rpi_neopixel.msg import NeopixelMessage
from rosmovement.msg import movMsg
RASPICAM_WIDTH = 640
RASPICAM_HEIGHT = 480
MARGIN = 20


class Controller(object):
    def __init__(self):
        # Init node
        rospy.init_node("controller", log_level=rospy.DEBUG)
#        self._distance_sensor = rospy.Subscriber("distance_sensor",
#                                                 std_msgs.msg.Float32,
#                                                 self.process_distance_sensor)
#        self._seek_thermal = rospy.Subscriber("seek_thermal",
#                                              std_msgs.mgs.Image,
#                                              self.process_seek_thermal)
#        self._aruco_detector = rospy.Subscriber("marker_detector/detection",
#                                              std,
#                                              self.process_aruco_detector)
        self._neopixel = rospy.Publisher("rpi_neopixel",
                                         NeopixelMessage,
                                         queue_size=1)
#        self._camera_feeds = rospy.Subscriber("camera_feeds",
#                                         std_msgs.msg.Image,
#                                         self.process_camera_feeds,
#                                         queue_size=1)
        self._movement = rospy.Publisher("move_it",
                                         movMsg,
                                         queue_size=1)
        self._can_detector = rospy.Subscriber("can_detector/detection",
                                              CanDetection,
                                              self.process_can_detector)
        self.has_reached_can = False


        # Init state machine
       # self._state_machine = StateMachine(States.INIT, self.init_robot)
       # self._state_machine.add_state(States.LOOK_FOR_CAN, self.look_for_can)
       # self._state_machine.add_state(States.MOVE_TO_CAN, self.move_to_can)
       # self._state_machine.add_state(States.GRAB_CAN, self.grab_can)
       # self._state_machine.add_state(States.MOVE_TO_FRIDGE, self.move_to_fridge)
       # self._state_machine.add_state(States.DROP_CAN, self.drop_can)
       # self._state_machine.add_state(States.SLEEP, self.sleep)

        # Processed data
        self.has_found_can = False 
        self.can_coordinates = (0, 0, 0, 0) # x, y, width, height

        # Start running
        self.run()

    def init_robot(self):
        rospy.loginfo("Init robot")
        self.fancy_leds(NEOPIXEL_OFF)
        self._movement.publish(movMsg(0, 0, OPEN_GRABBER))

    def look_for_can(self):
        rospy.loginfo("Looking for can")

        # Look for can by turning around our own axis
        while True:
            rospy.loginfo("Turning 1 degrees")
            time.sleep(SLEEP_LOOK_FOR_CAN)
            if self.has_found_can:
                center = self.can.bb_width / 2 + self.can.bb_x
                rospy.loginfo("Center bounding box: {} < {} < {}".format(RASPICAM_WIDTH / 2 - MARGIN, center, RASPICAM_WIDTH/2 + MARGIN))
                self.fancy_leds(NEOPIXEL_WHITE)
                center_image = RASPICAM_WIDTH/2
                direction = (center - center_image > 0)
                if direction:
                    self._movement.publish(movMsg(0, 1, NOTHING_GRABBER))
                else:
                    self._movement.publish(movMsg(0, -1, NOTHING_GRABBER))

                
                if (RASPICAM_WIDTH / 2 - MARGIN) < center < (RASPICAM_WIDTH / 2 + MARGIN):
                    rospy.loginfo("End of turns")
                    self.fancy_leds(NEOPIXEL_RED, "detected")
                    return
            else:
                self._movement.publish(movMsg(0, 1, NOTHING_GRABBER))


    def move_to_can(self):
        rospy.loginfo("Moving to can")
        self._movement.publish(movMsg(0, 0, OPEN_GRABBER))

        # Move towards can
        while True:
            rospy.loginfo("Moving forward by 1 cm")
            self._movement.publish(movMsg(1, 0, NOTHING_GRABBER))
            time.sleep(SLEEP_MOVE_TOWARDS_CAN)
            center = self.can.bb_height / 2 + self.can.bb_y
            rospy.loginfo("Center bounding box: {} < {} < {}".format(0, center, 50))
            self.fancy_leds(NEOPIXEL_GREEN)
            if 0 < center < 50:
                self._movement.publish(movMsg(0, 0, CLOSE_GRABBER))
                self.fancy_leds(NEOPIXEL_GREEN, "detected")
                rospy.loginfo("Can detection OK, stop")
                return 

    def grab_can(self):
        rospy.loginfo("Grabbing can")
        self._movement.publish(movMsg(0, 0, CLOSE_GRABBER))
        time.sleep(SLEEP_GRAB_CAN)

    def move_to_fridge(self):
        rospy.loginfo("Moving to fridge")

    def drop_can(self):
        rospy.loginfo("Dropping can")
        self._movement.publish(movMsg(0, 0, OPEN_GRABBER))
        time.sleep(SLEEP_DROP_CAN)

    def sleep(self):
        rospy.loginfo("Sleeping for {} seconds".format(SLEEP_TIME))
        time.sleep(SLEEP_TIME)

    def process_distance_sensor(self, distance):
        rospy.logdebug("Distance received: {} cm".format(distance))

    def process_seek_thermal(self, heat_image):
        rospy.logdebug("Heat image received")

    def process_aruco_detector(self, aruco):
        rospy.logdebug("Aruco received")

    def process_can_detector(self, can):
        rospy.logdebug("Can detection received")
        self.has_found_can = True
        self.can = can

    def run(self):
        """
        Run method which operates the state changes.
        """
        #self._state_machine.next_state()
        self.init_robot()
        self.look_for_can()
        self.move_to_can()
        time.sleep(5)
        self._movement.publish(movMsg(0, 0, OPEN_GRABBER))
        exit()

    def fancy_leds(self, color, animate=""):
        msg = NeopixelMessage()
        msg.pixels = [color for i in range(0, 5)]
        msg.animation = animate
        self._neopixel.publish(msg)


# Start controller
if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt")
