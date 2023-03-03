import numpy as np

from algorithm.constants import DIRECTION


def radiansToDegrees(direction: DIRECTION):
    if direction == DIRECTION.TOP:
        return 90
    elif direction == DIRECTION.LEFT:
        return 180
    elif direction == DIRECTION.RIGHT:
        return 0
    else:
        return -90


def basic_angle(theta):
    """
    Converts angle to basic angle
    :param theta: radians
    :return:
    """

    while theta > np.pi:
        theta -= 2.0 * np.pi

    while theta < -np.pi:
        theta += 2.0 * np.pi

    return theta