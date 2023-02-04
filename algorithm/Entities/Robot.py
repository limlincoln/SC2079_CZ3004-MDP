import pygame

from algorithm import settings
from algorithm.Entities.arena import Arena
from algorithm.constants import DIRECTION


class Robot:
    def __init__(self, ob):
        self.x = 0
        self.y = 0
        self.pos = (self.x, self.y)
        self.height = 300
        self.width = 300
        self.orientation = DIRECTION.TOP
        self.orientationList = [DIRECTION.TOP, DIRECTION.RIGHT, DIRECTION.BOTTOM, DIRECTION.LEFT]
        self.image = pygame.transform.scale(pygame.image.load("assets/car.png"), (3 * settings.BLOCK_SIZE, 3
                                                                                  * settings.BLOCK_SIZE))
        self.car_rect = self.image.get_rect()
        self.arrow = pygame.transform.scale(pygame.image.load("assets/icons8-arrow-100.png"), (3 * settings.BLOCK_SIZE,
                                                                                               3 * settings.BLOCK_SIZE))
        self.arrow = pygame.transform.rotate(self.arrow, 90)
        self.command = "S"
        self.obstacles = ob

    def drawCar(self, SCREEN):
        """
        Draw the car on the SCREEN according to its pos and orientation
        :param SCREEN: SCREEN
        :return: None
        """
        turn = 0
        if self.command == "R":
            turn = 90
        elif self.command == "L":
            turn = -90

        self.image = pygame.transform.rotate(self.image, turn)
        self.arrow = pygame.transform.rotate(self.arrow, turn)
        pos = (self.x, self.y)
        print(pos)
        self.car_rect.bottomleft = Arena.posConverter(pos)
        SCREEN.blit(self.image, self.car_rect)
        SCREEN.blit(self.arrow, self.car_rect)

    def moveToDo(self, command: tuple, SCREEN):
        """
        To Set pos of the car to the next location ready for display
        :param command: tuple (direction, command)
        :return:
        """
        self.command = command[3]
        self.x = command[0]
        self.y = command[1]
        """
        # if the car is going straight or reverse
        if self.command == 'S':
            if command[0] == DIRECTION.TOP:
                self.y += 10
            elif command[0] == DIRECTION.LEFT:
                self.x -= 10
            elif command[0] == DIRECTION.RIGHT:
                self.x += 10
            else:
                self.y -= 10
        # if car is going reverse:
        elif self.command == 'SV':
            if command[0] == DIRECTION.TOP:
                self.y -= 10
            elif command[0] == DIRECTION.LEFT:
                self.x += 10
            elif command[0] == DIRECTION.RIGHT:
                self.x -= 10
            else:
                self.y += 10
        # if car going to turn right:
        elif self.command == 'R':
            print(command)

            if command[0] == DIRECTION.TOP:
                self.x += 20
                self.y += 20

            elif command[0] == DIRECTION.LEFT:
                self.x -= 20
                self.y += 20

            elif command[0] == DIRECTION.RIGHT:

                self.x += 20
                self.y -= 20
            else:

                self.x -= 20
                self.y -= 20
        # if the car is to turn left:
        elif self.command == 'L':

            if command[0] == DIRECTION.TOP:
                self.x -= 20
                self.y += 20

            elif command[0] == DIRECTION.LEFT:
                self.x -= 20
                self.y -= 20

            elif command[0] == DIRECTION.RIGHT:
                self.x += 20
                self.y += 20
            else:
                self.x += 20
                self.y -= 20

        
        
        """
