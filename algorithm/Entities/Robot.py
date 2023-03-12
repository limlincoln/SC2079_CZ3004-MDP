import pygame

from algorithm import settings

from algorithm.Entities.arena import Arena
from algorithm.constants import DIRECTION
from algorithm.HelperFunctions import radiansToDegrees
from algorithm.Entities.STMCommand import STMCommand



class Robot:
    def __init__(self, ob):
        self.x = 0
        self.y = 0
        self.pos = (self.x, self.y)
        self.height = 300
        self.width = 300
        self.orientation = DIRECTION.TOP
        self.oldOrientation = self.orientation
        self.orientationList = [DIRECTION.TOP, DIRECTION.RIGHT, DIRECTION.BOTTOM, DIRECTION.LEFT]
        self.image = pygame.transform.scale(pygame.image.load("assets/car.png"), (3 * settings.BLOCK_SIZE, 3
                                                                                  * settings.BLOCK_SIZE))
        self.car_rect = self.image.get_rect()
        self.arrow = pygame.transform.scale(pygame.image.load("assets/icons8-arrow-100.png"), (3 * settings.BLOCK_SIZE,
                                                                                               3 * settings.BLOCK_SIZE))
        self.arrow = pygame.transform.rotate(self.arrow, 90)

        self.command = None
        self.obstacles = ob

    def drawCar(self, SCREEN):
        """
        Draw the car on the SCREEN according to its pos and orientation
        :param SCREEN: SCREEN
        :return: None
        """
        if self.orientation != self.oldOrientation:
            self.image = pygame.transform.rotate(self.image, -(
                    radiansToDegrees(self.oldOrientation) - radiansToDegrees(self.orientation)))
            self.arrow = pygame.transform.rotate(self.arrow, -(
                    radiansToDegrees(self.oldOrientation) - radiansToDegrees(self.orientation)))
        self.oldOrientation = self.orientation
        self.pos = (self.x, self.y)
        print(self.pos)
        self.car_rect.bottomleft = Arena.posConverter(self.pos)
        SCREEN.blit(self.image, self.car_rect)
        SCREEN.blit(self.arrow, self.car_rect)


    def moveToDo(self, command, SCREEN):
        """
        To Set pos of the car to the next location ready for display
        :param command: tuple (direction, command)
        :return:
        """

        self.x = command[0]
        self.y = command[1]
        self.orientation = command[2]

    def setCurrentCommand(self, command):
        self.command = STMCommand(command)
        self.command.setTick()
        

