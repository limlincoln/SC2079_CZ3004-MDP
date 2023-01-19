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

    def drawCar(self, SCREEN):
        self.image = pygame.transform.rotate(self.image, self.orientation)
        self.car_rect.bottomleft = (self.x,  self.y)
        SCREEN.blit(self.image, self.car_rect)
        # rect = self.arrow.get_rect()
        # rect.bottomleft = (800,800)
        # self.SCREEN.blit(self.arrow, rect)
    def moveStraight(self, StraightDistance):
        if self.orientation == 90:
            self.y = StraightDistance + self.y
        elif self.orientation == 0:
            self.x = StraightDistance + self.x
        elif self.orientation == 180:
            self.x = self.x - StraightDistance
        else:
            self.y = self.y - StraightDistance


    def turning(self, turningDistance, orientation):
        if orientation == 0 and self.orientation == 90:
            self.orientation = 0
            self.x = turningDistance + self.x
            self.y = settings.TURNING_RADIUS + self.y
        elif orientation == 180 and self.orientation == 90:
            self.orientation = 180
            self.x = self.x - turningDistance
            self.y = self.y + settings.TURNING_RADIUS

