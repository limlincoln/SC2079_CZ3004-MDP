from Entities.Rectangle import Rectangle
from Entities.Obstacle import Obstacle
from scipy.spatial import KDTree
import numpy as np


class StaticEnvironment:
    """

    """
    def __init__(self, dimensions, obstacles: list[Obstacle]):
        self.dimensions = dimensions
        self.obstacles = obstacles
        self.kdtree = KDTree([obs.pos for obs in self.obstacles])

    def isWalkable(self, x,y, time=0):
        """
         Checks if the robot can occupy this location
        :param x: int
            x coordinate
        :param y: int
            y coordinate
        :param time: float

        :return:
        true if walkable
        """
        if x < 0 or x > (self.dimensions[0]-30) or y < 0 or (y > self.dimensions[1]-30):
            return False
        robotRect = Rectangle((x,y), 'R')
        for obstacle in self.obstacles:
            if robotRect.isCollided(Rectangle(obstacle.pos, 'O')):
                return False
        return True

    def randomFreeSpace(self):
        x = np.random.rand()*self.dimensions[0]
        y = np.random.rand()*self.dimensions[1]
        while not self.isWalkable(x,y):
            x = np.random.rand()*self.dimensions[0]
            y = np.random.rand()*self.dimensions[1]
        return  x , y , np.random.rand()*np.pi*2

    def closeObstacles(self, x, y, nbObstacles=1):
        "Get close Obstacles"
        return [self.obstacles[index] for index in self.kdtree.query((x,y),nbObstacles)[1]]

    def generateTargetLocation(self):
        """
        get all the configurations that the robot needs to visit
        :param obstacles: List[Obstacles]
        :return:
            list of configurations in the form (x,y,direction in char)
        """
        targetLocations = []
        for ob in self.obstacles:
            if ob.imageOrientation == "right":
                targetLocations.append((ob.pos[0] + 50, ob.pos[0] - 10, 0))
            elif ob.imageOrientation == "top":
                targetLocations.append((ob.pos[0] + 10, ob.pos[1] + 50, np.pi/2))
            elif ob.imageOrientation == "left":
                targetLocations.append((ob.pos[0] - 50, ob.pos[1] + 10, np.pi))
            else:
                targetLocations.append((ob.pos[0] + 10, ob.pos[1] - 50, -(np.pi/2)))
        return targetLocations