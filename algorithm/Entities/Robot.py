import pygame

from algorithm import settings


class Robot:
    def __init__(self):
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
        if self.orientation == 90:
            self.y = self.y - StraightDistance
        elif self.orientation == 0:
            self.x = StraightDistance + self.x
        elif self.orientation == 180:
            self.x = self.x - StraightDistance
        else:
            self.y = self.y + StraightDistance


    def turnLeft(self, turningDistance):
        angle = self.orientation + 90
        if angle > 270:
            angle = 0
        self.orientation = angle
        if self.orientation == 270 or self.orientation == 0:
            self.y = self.y - settings.TURNING_RADIUS
        else:
            self.y = self.y + settings.TURNING_RADIUS

        self.command = 'L'
    def turnRight(self, turningDistance):

        angle = self.orientation - 90
        print(self.orientation,angle)
        if angle < 0:
            angle = 270
        self.orientation = angle
        if self.orientation == 270 or self.orientation == 180:
            self.y = self.y - settings.TURNING_RADIUS
        else:
            self.y = self.y - settings.TURNING_RADIUS

        self.x = self.x - turningDistance
        self.command = 'R'

