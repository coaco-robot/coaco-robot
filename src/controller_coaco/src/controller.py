#!/usr/bin/env python
import rospy
import time
from constants import States, SLEEP_TIME
from state_machine import StateMachine


class Controller(object):
    def __init__(self):
        # Init node
        rospy.init_node("controller", log_level=rospy.DEBUG)

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

    def run(self):
        for i in range(0, 20):
            self.next_state()

    def next_state(self):
        self._state_machine.next_state()

# Start controller
if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt")
