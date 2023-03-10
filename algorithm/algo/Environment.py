
import pygame

from algorithm.Entities.Rectangle import Rectangle
from algorithm.Entities.Obstacle import Obstacle
from scipy.spatial import KDTree
import numpy as np
from algorithm.constants import DIRECTION
from algorithm.Entities.RectRobot import RectRobot
from algorithm.HelperFunctions import radiansToDegrees
from algorithm.Entities.VirtualRect import VirtualRect
import shapely as sp
from algorithm.HelperFunctions import basic_angle

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
    def turn_check(self, pos):
        """
              points = []
        current = pos
        length = (2 * np.pi * 20) / 4
        if t == 's' or t == 'b':
            length = 10
        for x in np.arange(0, length, 2):
            current = self.nextPos(current, t, 2)
            points.append(current)
        for point in points:
            if not self.isWalkable(point[0],point[1]):
                return False
        return True
        :param pos:
        :param t:
        :return:
        """
        for obs in self.obstacles:
            ob_pos = Rectangle(obs.pos, 'O')
            print(pos[2])
            if not self.direction_turn_check(pos[2], pos, (ob_pos.x, ob_pos.y)):
                return False
        return True


    def direction_turn_check(self, dir, pos1, pos2):
        if dir == DIRECTION.TOP:
            if pos1[1] - pos2[1] < 10:
                return False
        elif dir == DIRECTION.RIGHT:
            if pos1[0] - pos2[0] < 40:
                return False
        elif dir == DIRECTION.LEFT:
            if pos2[0] - pos1[0] < 40:
                return False
        else:
            if pos2[1] - pos1[1] < 10:
                return False
        return True

    def nextPos(self, pos, type, delta):
        """
        Get the next position in continuous step
        :param pos:
        :param v:
        :param steeringAngle:
        :return:
        """
        direction = pos[2]
        if type == "s":
            new_X = pos[0] + delta * np.cos(direction)
            new_Y = pos[1] + delta * np.sin(direction)
            new_orientation = direction
        elif type == "d":
            new_X = pos[0] + delta * np.cos(direction)
            new_Y = pos[1] + delta * np.sin(direction)
            new_orientation = basic_angle(direction - delta / 20)
        elif type == "u":
            new_X = pos[0] + delta * np.cos(direction)
            new_Y = pos[1] + delta * np.sin(direction)
            new_orientation = basic_angle(direction + delta / 20)
        elif type == "b":
            new_X = pos[0] - delta * np.cos(direction)
            new_Y = pos[1] - delta * np.sin(direction)
            new_orientation = direction
        elif type == "w":
            new_X = pos[0] - delta * np.cos(direction)
            new_Y = pos[1] - delta * np.sin(direction)
            new_orientation = basic_angle(direction + delta / 20)
        elif type == "v":
            new_X = pos[0] - delta * np.cos(direction)
            new_Y = pos[1] - delta * np.sin(direction)
            new_orientation = basic_angle(direction - delta / 20)
        return new_X, new_Y, new_orientation
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
        possible_pos = {"E": [(40,-10), (40,0), (40,-20)],
                        "N": [(-10, 40), (0,40), (-20,40)],
                        "W": [(-40,-10), (-40,0), (-40,-20)],
                        "S": [(-10,-40), (-20,-40), (0,-40)]}

        for ob in self.obstacles:
            valid_pos = None
            if ob.imageOrientation == "E":
                for x in possible_pos["E"]:
                    if self.isWalkable(ob.pos[0]+x[0], ob.pos[1]+x[1]):
                        valid_pos = ob.pos[0]+x[0], ob.pos[1]+x[1], DIRECTION.LEFT
                        break
            elif ob.imageOrientation == "N":
                for x in possible_pos["N"]:
                    if self.isWalkable(ob.pos[0]+x[0], ob.pos[1]+x[1]):
                        valid_pos = ob.pos[0]+x[0], ob.pos[1]+x[1], DIRECTION.BOTTOM
                        break
            elif ob.imageOrientation == "W":
                for x in possible_pos["W"]:
                    if self.isWalkable(ob.pos[0]+x[0], ob.pos[1]+x[1]):
                        valid_pos = ob.pos[0]+x[0], ob.pos[1]+x[1], DIRECTION.RIGHT
                        break
            else:
                for x in possible_pos["S"]:
                    if self.isWalkable(ob.pos[0]+x[0], ob.pos[1]+x[1]):
                        valid_pos = ob.pos[0]+x[0], ob.pos[1]+x[1], DIRECTION.TOP
                        break
            if valid_pos:
                targetLocations.append(valid_pos)
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
                targetLocations.append((ob.pos[0] + 40, ob.pos[1] - 5, DIRECTION.LEFT.value))
            elif ob.imageOrientation == "top":

                targetLocations.append((ob.pos[0] - 5, ob.pos[1] + 40, DIRECTION.BOTTOM.value))
            elif ob.imageOrientation == "left":
                targetLocations.append((ob.pos[0] - 40, ob.pos[1] - 5, DIRECTION.RIGHT.value))
            else:
                targetLocations.append((ob.pos[0] + 5, ob.pos[1] - 40, DIRECTION.TOP.value))
        return targetLocations


class AdvancedEnvironment:
    def __init__(self, dimensions, obstacles: list[Obstacle], task=1):
        self.dimensions = dimensions
        self.obstacles = obstacles
        self.task = task
        self.targets = self.generateTargetLocationsInRads(self.obstacles)
        self.virtualObstacles = self.generateVirtualObstacles(self.obstacles)
        self.KDtree = KDTree([obs.center for obs in self.virtualObstacles])
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
            if self.task == 1:
                virutalOb = VirtualRect(ob)
                list.append(virutalOb)
            elif self.task == 2:
                virutalOb = VirtualRect(ob, 2)
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
            if ob.collides(robotPos):
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

