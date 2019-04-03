#!/usr/bin/env python
import rospy
import time
import std_msgs
from constants import States, SLEEP_TIME
from state_machine import StateMachine
from can_detector.msg import CanDetection
from rpi_neopixel.msg import NeopixelMessage
from rosmovement.msg import movMsg


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
#        self._neopixel = rospy.Publisher("rpi_neopixel",
#                                         NeopixelMessage,
#                                         queue_size=1)
#        self._camera_feeds = rospy.Subscriber("camera_feeds",
#                                         std_msgs.msg.Image,
#                                         self.process_camera_feeds,
#                                         queue_size=1)
        self._movement = rospy.Publisher("move_it",
                                         movMsg,
                                         queue_size=1)
#        self._can_detector = rospy.Subscriber("can_detector/detection",
#                                              CanDetection,
#                                              self.process_can_detector)


        # Init state machine
        self._state_machine = StateMachine(States.INIT, self.init_robot)
        self._state_machine.add_state(States.LOOK_FOR_CAN, self.look_for_can)
        self._state_machine.add_state(States.MOVE_TO_CAN, self.move_to_can)
        self._state_machine.add_state(States.GRAB_CAN, self.grab_can)
        self._state_machine.add_state(States.MOVE_TO_FRIDGE, self.move_to_fridge)
        self._state_machine.add_state(States.DROP_CAN, self.drop_can)
        self._state_machine.add_state(States.SLEEP, self.sleep)

        # Start running
        self.run()

    def init_robot(self):
        rospy.loginfo("Init robot")

    def look_for_can(self):
        rospy.loginfo("Looking for can")
        while True:
            rospy.loginfo("Turning 5 degrees")
            self._movement.publish(movMsg(0, 5, 0))
            time.sleep(SLEEP_LOOK_FOR_CAN)

    def move_to_can(self):
        rospy.loginfo("Moving to can")

    def grab_can(self):
        rospy.loginfo("Grabbing can")

    def move_to_fridge(self):
        rospy.loginfo("Moving to fridge")

    def drop_can(self):
        rospy.loginfo("Dropping can")

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

    def process_camera_feeds(self, feed):
        rospy.logdebug("Camera feed received")

    def run(self):
        """
        Run method which operates the state changes.
        """
        for i in range(0, 10):
            self._state_machine.next_state()

# Start controller
if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt")
