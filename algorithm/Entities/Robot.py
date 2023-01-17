import math
import pygame

import settings


class Robot:
    def __init__(self):
        self.pos = (800, 800)
        self.height = 300
        self.width = 300
        self.orientation = 90
        self.image = pygame.transform.scale(pygame.image.load("assets/car.png"), (6*settings.BLOCK_SIZE, 6*settings.BLOCK_SIZE))

    def drawCar(self, SCREEN):
        car_rect = self.image.get_rect()
        car_rect.bottomleft = (0, 800)
        SCREEN.blit(self.image, car_rect)
        # rect = self.arrow.get_rect()
        # rect.bottomleft = (800,800)
        # self.SCREEN.blit(self.arrow, rect)
    def collisonRadius(self):
        pass
