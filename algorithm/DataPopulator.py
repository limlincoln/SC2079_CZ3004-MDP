import algorithm.settings as settings
from algorithm.Entities.Obstacle import Obstacle


def getTestObstacles():
    test = [Obstacle((50, 90), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((70, 140), "left", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((120, 90), "right", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((150, 150), "bottom", (settings.BLOCK_SIZE, settings.BLOCK_SIZE)),
            Obstacle((150, 40), "left", (settings.BLOCK_SIZE, settings.BLOCK_SIZE))]
    return test