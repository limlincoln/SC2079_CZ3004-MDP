import settings
from algorithm.Entities import Obstacle
class Astar:
    def __init__(self, grid, obstacles: list[Obstacle]):
        #20x20 grid
        self.grid = grid
        self.obstacles = obstacles
        self.targetLocations = self.generateTargetLocation(obstacles)
        print(self.targetLocations)

    def computePath(self):
        pass


    def generateTargetLocation(self, obstacles: list[Obstacle]):
        targetLocations = []
        for ob in obstacles:
            if ob.imageOrientation == "right":
                targetLocations.append((ob.pos[0] + 5, ob.pos[1] - 1))
            elif ob.imageOrientation == "top":
                targetLocations.append((ob.pos[0] + 1, ob.pos[1] - 5))
            elif ob.imageOrientation == "left":
                targetLocations.append((ob.pos[0] - 5, ob.pos[1] - 1))
            else:
                targetLocations.append((ob.pos[0] + 1, ob.pos[1] + 5))
        return targetLocations

    def dubinsPathComputation(self, start, goal):
        pass
