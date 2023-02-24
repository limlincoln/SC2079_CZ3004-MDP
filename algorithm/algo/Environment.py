import pygame

from algorithm.Entities.Rectangle import Rectangle
from algorithm.Entities.Obstacle import Obstacle
from scipy.spatial import KDTree
import numpy as np
from algorithm.constants import DIRECTION
from algorithm.Entities.RectRobot import RectRobot
from algorithm.HelperFunctions import radiansToDegrees
import collision as cs
from algorithm.Entities.VirtualRect import VirtualRect
class StaticEnvironment:
    """

    """

    def __init__(self, dimensions, obstacles: list[Obstacle]):
        self.dimensions = dimensions
        self.obstacles = obstacles
        # for future use
        self.kdtree = KDTree([obs.pos for obs in self.obstacles])
        self.obID = []
        self.targetLocations = self.generateTargeLocations()

    def isWalkable(self, x, y, time=0):
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
        if x < 0 or x > (self.dimensions[0] - 30) or y < 0 or (y > self.dimensions[1] - 30):
            return False
        robotRect = Rectangle((x, y), 'R')
        for obstacle in self.obstacles:
            pos = Rectangle(obstacle.pos, 'O')
            if robotRect.isCollided(pos):
                return False
        return True

    def isWalkableV2(self, x, y, time=0):
        """
        testing
        :param x:
        :param y:
        :param time:
        :return:
        """
        if x < 0 or x > (self.dimensions[0] - 30) or y < 0 or (y > self.dimensions[1] - 30):
            return False
        robotRect = RectRobot((x, y))
        for obstacle in self.obstacles:
            if robotRect.isCollided(obstacle):
                return False
        return True

    def randomFreeSpace(self):
        x = np.random.rand() * self.dimensions[0]
        y = np.random.rand() * self.dimensions[1]
        while not self.isWalkable(x, y):
            x = np.random.rand() * self.dimensions[0]
            y = np.random.rand() * self.dimensions[1]
        return x, y, np.random.rand() * np.pi * 2

    def closeObstacles(self, x, y, nbObstacles=1):
        "Get close Obstacles"
        return [self.obstacles[index] for index in self.kdtree.query((x, y), nbObstacles)[1]]

    def getTargetLocation(self):
        """
        get all the configurations that the robot needs to visit
        :param obstacles: List[Obstacles]
        :return:
            list of configurations in the form (x,y,direction in char)
        """
        return self.targetLocations

    def generateTargeLocations(self):
        """
        generate the required locations
        :return:
        """

        targetLocations = []

        for ob in self.obstacles:
            if ob.imageOrientation == "E":
                targetLocations.append((ob.pos[0] + 50, ob.pos[1] - 10, DIRECTION.LEFT))
            elif ob.imageOrientation == "N":
                targetLocations.append((ob.pos[0] - 10, ob.pos[1] + 50, DIRECTION.BOTTOM))
            elif ob.imageOrientation == "W":
                targetLocations.append((ob.pos[0] - 50, ob.pos[1] - 10, DIRECTION.RIGHT))
            else:
                targetLocations.append((ob.pos[0] - 10, ob.pos[1] - 50, DIRECTION.TOP))
            self.obID.append(ob.ObId)
        return targetLocations

    def generateTargetLocationInRads(self):
        """
        same stuff as the generateTargetLocation but in rads
        :return:
        """

        targetLocations = []
        for ob in self.obstacles:
            if ob.imageOrientation == "right":
                targetLocations.append((ob.pos[0] + 40, ob.pos[1] - 10, DIRECTION.LEFT.value))
            elif ob.imageOrientation == "top":

                targetLocations.append((ob.pos[0] - 10, ob.pos[1] + 40, DIRECTION.BOTTOM.value))
            elif ob.imageOrientation == "left":
                targetLocations.append((ob.pos[0] - 40, ob.pos[1] - 10, DIRECTION.RIGHT.value))
            else:
                targetLocations.append((ob.pos[0] + 10, ob.pos[1] - 40, DIRECTION.TOP.value))
        return targetLocations


class AdvancedEnvironment:
    def __init__(self, dimensions, obstacles: list[Obstacle]):
        self.dimensions = dimensions
        self.obstacles = obstacles
        self.collisionPoints = self.generateCollisionPoints(self.obstacles)
        self.targets = self.generateTargetLocationsInRads(self.obstacles)
        self.virtualObstacles = self.generateVirtualObstacles(self.obstacles)

    def generateTargetLocationsInRads(self, obstacles):

        targetLocations = []
        for ob in obstacles:
            if ob.imageOrientation == 'E':
                targetLocations.append((ob.pos[0] + 40, ob.pos[1] + 5, DIRECTION.LEFT.value, ob.ObId))
            elif ob.imageOrientation == 'N':
                targetLocations.append((ob.pos[0] + 5, ob.pos[1] + 40, DIRECTION.BOTTOM.value, ob.ObId))
            elif ob.imageOrientation == 'W':
                targetLocations.append((ob.pos[0] - 40, ob.pos[1] + 5, DIRECTION.RIGHT.value, ob.ObId))
            else:
                targetLocations.append((ob.pos[0] + 5, ob.pos[1] - 40, DIRECTION.TOP.value, ob.ObId))
        return targetLocations

    def generateVirtualObstacles(self, obstacles: list[Obstacle]):
        """
        Generate Virtual Obstacles
        :param obstacles: list[Obstacles]
        :return: list[VirutalObstacles]
        """
        list = []
        for ob in obstacles:
            virutalOb = VirtualRect(ob)
            list.append(virutalOb)

        return list

    def isWalkable(self, robotPos):
        """
        Check if the robot is able to move here.
        :param robotPos: tuple (x,y)
        :return: bool
        """
        if robotPos[0] < 15 or robotPos[0] > (self.dimensions[0] - 15) or robotPos[1] < 15 or (robotPos[1] > self.dimensions[1] - 15):
            return False

        for ob in self.virtualObstacles:
            if ob.isCollided(robotPos):
                return False
        return True
    def generateCollisionPoints(self, obstacles):
        """
                for ob in obstacles:
            rect = pygame.Rect(ob.pos, (10,10))
            rect.bottomleft = ob.pos
            pivot = pygame.math.Vector2(ob.pos)
            p0 = pygame.math.Vector2(rect.topleft) +pivot
            p1 = pygame.math.Vector2(rect.topright) + pivot
            p2 = pygame.math.Vector2(rect.bottomright) + pivot
            p3 = pygame.math.Vector2(rect.bottomleft) + pivot
            self.collisionPoints.append([p0,p1,p2,p3])
        :param obstacles:
        :return:
        """
        collision_list = []
        for ob in obstacles:
            vectors = self.generateVectors(ob.pos, 'O')
            center = cs.Vector(ob.pos[0], ob.pos[1])
            polygon = cs.Poly(center, vectors)
            collision_list.append(polygon)
        return collision_list

    def generateVectors(self, pos, type):
        """
        Unused atm
        :param pos:
        :param type:
        :return:
        """
        assert type in 'RO'
        if type == 'O':
            p0 = cs.Vector(pos[0]-15, pos[1]-15)
            p1 = cs.Vector(pos[0]-15, pos[1]+25)
            p2 = cs.Vector(pos[0]+25, pos[1]-15)
            p3 = cs.Vector(pos[0]+25, pos[1]+15)
        elif type == 'R':
            print("test R")
            p0 = cs.Vector(pos[0]-15, pos[1]-15)
            p1 = cs.Vector(pos[0]-15, pos[1]+15)
            p2 = cs.Vector(pos[0]+15, pos[1]-15)
            p3 = cs.Vector(pos[0]+15, pos[1]+15)
        return [p0,p1,p2,p3]
    def collisionCheck(self, pos):
        """
        Unused atm
        :param pos:
        :return:
        """

        angle = pos[2]
        vectors = self.generateVectors((pos[0], pos[1]), 'R')
        center = cs.Vector(pos[0], pos[1])
        polygon = cs.Poly(center, vectors, angle)
        print(polygon.aabb)
        for ob in self.collisionPoints:
            print(ob.aabb)
            if cs.collide(ob, polygon):
                return False
        return True

    def randomFreeSpace(self):
        x = np.random.rand() * self.dimensions[0]
        y = np.random.rand() * self.dimensions[1]
        while not self.isWalkable((x, y)):
            x = np.random.rand() * self.dimensions[0]
            y = np.random.rand() * self.dimensions[1]
        return x, y, np.random.rand() * np.pi * 2

        """
        
        rect = pygame.Rect((pos[0], pos[1]), (30, 30))
        rect.center = (pos[0], pos[1])
        pivot = pygame.math.Vector2((pos[0], pos[1]))
        angle = -(90 - pos[2])
        p0 = (pygame.math.Vector2(rect.topleft) - pivot).rotate(angle) + pivot
        p1 = (pygame.math.Vector2(rect.topright) - pivot).rotate(angle) + pivot
        p2 = (pygame.math.Vector2(rect.bottomright) - pivot).rotate(angle) + pivot
        p3 = (pygame.math.Vector2(rect.bottomleft) - pivot).rotate(angle) + pivot
        
        """

