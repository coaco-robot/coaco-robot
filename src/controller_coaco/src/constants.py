#!/usr/bin/env python
from enum import Enum


SLEEP_TIME = 3 # seconds 

class States(Enum):
    INIT = 0
    LOOK_FOR_CAN = 1
    MOVE_TO_CAN = 2
    GRAB_CAN = 3
    MOVE_TO_FRIDGE = 4
    DROP_CAN = 5
    SLEEP = 6
