import settings


class Obstacle:
    def __init__(self, pos, imageOrientation, dimension):
        self.pos = pos
        self.imageOrientation = imageOrientation
        self.dimension = dimension
        self.gridPosition = ((pos[0]) * settings.BLOCK_SIZE, (pos[1]) * settings.BLOCK_SIZE)


