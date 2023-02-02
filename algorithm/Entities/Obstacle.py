import settings


class Obstacle:
    def __init__(self, pos, imageOrientation, dimension):
        self.pos = pos
        self.imageOrientation = imageOrientation
        self.dimension = dimension
        self.gridPosition = (pos[0] // settings.GRID_SCALE_FACTOR) * settings.BLOCK_SIZE + settings.GRID_OFFSET, \
                            (settings.GRID_Y_OFFSET - (pos[1] // settings.GRID_SCALE_FACTOR) * settings.BLOCK_SIZE) + \
                            settings.GRID_OFFSET
