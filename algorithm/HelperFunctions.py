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
