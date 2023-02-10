import algorithm.settings as settings
from algorithm.Entities.Obstacle import Obstacle


def getTestObstacles():
    test = [Obstacle((50, 90), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((70, 140), "left", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((120, 90), "right", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((150, 150), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((150, 40), "left", (settings.BLOCK_SIZE, settings.BLOCK_SIZE))]
    return test

def getTestObstacles1():
    test = [
        Obstacle((40, 30), "top", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((80, 60), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((120, 100), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((160, 140), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((170, 30), "top", (settings.BLOCK_SIZE, settings.BLOCK_SIZE))

    ]
    return test

def getTestObstacles2():
    test = [
        Obstacle((10, 150), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((150, 80), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((120, 100), "left", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((160, 120), "top", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
        Obstacle((30, 170), "right", (settings.BLOCK_SIZE, settings.BLOCK_SIZE))

    ]
    return test
