import pygame

from algorithm import settings
from Entities.arena import Arena

class Robot:
    def __init__(self, ob):
        self.x = 0
        self.y = 0
        self.pos = (self.x, self.y)
        self.height = 300
        self.width = 300
        self.orientation = 90
        self.image = pygame.transform.scale(pygame.image.load("assets/car.png"), (3 * settings.BLOCK_SIZE, 3
                                                                                  * settings.BLOCK_SIZE))
        self.car_rect = self.image.get_rect()
        self.arrow = pygame.transform.scale(pygame.image.load("assets/icons8-arrow-100.png"), (3 * settings.BLOCK_SIZE,
                                                                                               3 * settings.BLOCK_SIZE))
        self.arrow = pygame.transform.rotate(self.arrow, 90)
        self.command = "S"
        self.obstacles = ob
        print(self.obstacles)

    def drawCar(self, SCREEN):
        """
        Draw the car on the SCREEN according to its pos and orientation
        :param SCREEN: SCREEN
        :return: None
        """
        turn = 0
        if self.command == "R":
            turn = -90
        elif self.command == "L":
            turn = 90

        self.image = pygame.transform.rotate(self.image, turn)
        self.arrow = pygame.transform.rotate(self.arrow, turn)
        pos = (self.x, self.y)
        self.car_rect.bottomleft = Arena.posConverter(pos)
        SCREEN.blit(self.image, self.car_rect)
        SCREEN.blit(self.arrow, self.car_rect)

    def moveToDo(self, command):
        pass

