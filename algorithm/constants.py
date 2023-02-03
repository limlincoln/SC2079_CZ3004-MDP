import numpy as np
from enum import Enum
from enum import IntEnum


class DIRECTION(Enum):
    TOP = np.pi / 2
    RIGHT = 0
    LEFT = np.pi
    BOTTOM = -(np.pi / 2)


class DIRECTIONINT(Enum):
    TOP = 0
    RIGHT = 1
    LEFT = 2
    BOTTOM = 3


class MOVEMENT(Enum):
    FORWARD = 0
    REVERSE = 1
    LEFT = 2
    RIGHT = 3


class COST(IntEnum):
    INFINITE_COST = 9999
    MOVE_COST = 10
    MOVE_COST_DIAG = 15
    TURN_COST = 20
    TURN_COST_DIAG = 10
    WAYPOINT_PENALTY = 1000
