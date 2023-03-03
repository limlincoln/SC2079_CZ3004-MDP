import numpy as np
from enum import Enum
from enum import IntEnum


class DIRECTION(Enum):
    TOP = np.pi / 2
    RIGHT = 0
    LEFT = np.pi
    BOTTOM = -(np.pi / 2)

    NORTHWEST = np.pi - np.pi / 4
    NORTHEAST = np.pi / 4
    SOUTHEAST = -(np.pi / 4)
    SOUTHWEST = -(np.pi - np.pi / 4)



class DIRECTIONINT(Enum):
    TOP = 0
    RIGHT = 1
    LEFT = 2
    BOTTOM = 3
    NORTHWEST = 4
    NORTHEAST = 5
    SOUTHEAST = 6
    SOUTHWEST = 7



class MOVEMENT(IntEnum):

    FORWARD = 0
    REVERSE = 1
    LEFT = 2
    RIGHT = 3

    REVRIGHT = 4
    REVLEFT = 5
    TURN_O_RIGHT = 6
    TURN_O_LEFT = 7



class COST(IntEnum):
    INFINITE_COST = 9999
    MOVE_COST = 10
    MOVE_COST_DIAG = 15
    TURN_COST = 999

    TURN_COST_DIAG = 10
    WAYPOINT_PENALTY = 1000
