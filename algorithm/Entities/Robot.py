import pygame

from algorithm import settings
from Entities.RectRobot import RectRobot


class Robot:
    def __init__(self, ob):
        self.x = 0
        self.y = 800
        self.pos = (self.x, self.y)
        self.height = 300
        self.width = 300
        self.orientation = 90
        self.image = pygame.transform.scale(pygame.image.load("assets/car.png"), (6*settings.BLOCK_SIZE, 6*settings.BLOCK_SIZE))
        self.car_rect = self.image.get_rect()
        self.arrow = pygame.transform.scale(pygame.image.load("assets/icons8-arrow-100.png"), (6*settings.BLOCK_SIZE, 6*settings.BLOCK_SIZE))
        self.arrow = pygame.transform.rotate(self.arrow, 90)
        self.command = "S"
        self.obstacles = ob
        print(self.obstacles)

    def drawCar(self, SCREEN):
        turn = 0
        if self.command == "R":
            turn = -90
        elif self.command == "L":
            turn = 90

        self.image = pygame.transform.rotate(self.image, turn)
        self.arrow = pygame.transform.rotate(self.arrow, turn)
        self.car_rect.bottomleft = (self.x,  self.y)
        SCREEN.blit(self.image, self.car_rect)
        SCREEN.blit(self.arrow, self.car_rect)

    def moveStraight(self, StraightDistance):
        self.command = 'S'
        if self.orientation == 90 and self.isWalkable((self.x, self.y-StraightDistance)):
            self.y = self.y - StraightDistance
        elif self.orientation == 0 and self.isWalkable((self.x + StraightDistance, self.y)):
            self.x = StraightDistance + self.x
        elif self.orientation == 180 and self.isWalkable((self.x - StraightDistance, self.y)):
            self.x = self.x - StraightDistance
        else:
            if self.isWalkable((self.x, self.y + StraightDistance)):
                self.y = self.y + StraightDistance

    def turnLeft(self, turningDistance):
        angle = self.orientation + 90
        if angle > 270:
            angle = 0
        self.orientation = angle
        if self.orientation == 270 or self.orientation == 0:
            if self.isWalkable((self.x, self.y - settings.TURNING_RADIUS)):
                self.y = self.y - settings.TURNING_RADIUS
        else:
            if self.isWalkable((self.x, self.y + settings.TURNING_RADIUS)):
                self.y = self.y + settings.TURNING_RADIUS

        self.command = 'L'

    def turnRight(self, turningDistance):
        angle = self.orientation - 90
        print(self.orientation, angle)
        if angle < 0:
            angle = 270
        self.orientation = angle
        if self.orientation == 270 or self.orientation == 180:
            if self.isWalkable((self.x, self.y - settings.TURNING_RADIUS)):
                self.y = self.y - settings.TURNING_RADIUS
        else:
            if self.isWalkable((self.x, self.y - settings.TURNING_RADIUS)):
                self.y = self.y - settings.TURNING_RADIUS

       # self.x = self.x - turningDistance
        self.command = 'R'

    # check if the robot can move to the goal position
    def isWalkable(self, goalPosition):
        goalRobotPosition = RectRobot(goalPosition)
        if goalPosition[0] < 0 or goalPosition[0] > 770:
            return False
        elif goalPosition[1] < 30 or goalPosition[1] > 800:
            return False
        for ob in self.obstacles:
            if goalRobotPosition.isCollided(ob):
                return False
        print("true")
        return True
