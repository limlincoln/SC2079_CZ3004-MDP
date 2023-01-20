from algorithm.Entities import Obstacle
class Astar:
    def __init__(self, grid, obstacles: list[Obstacle]):
        #40x40 grid
        self.grid = grid

        self.obstacles = obstacles

