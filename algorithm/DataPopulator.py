import algorithm.settings as settings
from algorithm.Entities.Obstacle import Obstacle


def getTestObstacles():
    test = [Obstacle((50, 90), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '1'),
            Obstacle((70, 140), "W", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '2'),
            Obstacle((120, 90), "E", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '3'),
            Obstacle((150, 150), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '4'),
            Obstacle((150, 40), "W", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '5')]
    return test

def getTestObstacles1():
    test = [
        Obstacle((40, 30), "N", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '1'),
        Obstacle((80, 80), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '2'),
        Obstacle((120, 100), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '3'),
        Obstacle((160, 140), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '4'),
        Obstacle((170, 30), "N", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '5')

    ]
    return test

def getTestObstacles2():
    test = [
        Obstacle((10, 150), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '1'),
        Obstacle((150, 80), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '2'),
        Obstacle((120, 100), "S", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '3'),
        Obstacle((160, 120), "N", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '4'),
        Obstacle((30, 170), "E", (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '5')

    ]
    return test
