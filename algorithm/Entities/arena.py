import pygame
from algorithm import settings
"""
Represents the navigational area (default: 40x40 grid of 5x5cm grid cells)
"""
class Arena:
    def __init__(self,obstacles, width, height, blockSize):
        self.obstacles = obstacles
        self.height = height
        self.width = width
        self.blockSize = blockSize
        self.obList = []


    def drawGrid(self, SCREEN):
        RED = (251,0,0)
        for x in range(0,self.width, self.blockSize):
            for y in range(0, self.height, self.blockSize):
                grid = pygame.Rect(x,y, self.blockSize, self.blockSize)
                pygame.draw.rect(SCREEN, settings.WHITE, grid, 1)
        for obstacle in self.obstacles:
            ob = pygame.Rect(obstacle.pos, obstacle.dimension)
            self.obList.append(ob)
            pygame.draw.rect(SCREEN, settings.GREEN, ob)
            self.drawBorder(obstacle,SCREEN,RED,ob)

    def drawBorder(self, obstacle,  SCREEN, COLOUR, ob):
        if obstacle.imageOrientation == "top":
            pygame.draw.line(SCREEN, COLOUR, ob.topleft, ob.topright, 2)
        elif obstacle.imageOrientation == "right":
            pygame.draw.line(SCREEN, COLOUR, ob.topright, ob.bottomright, 2)
        elif obstacle.imageOrientation == "bottom":
            pygame.draw.line(SCREEN, COLOUR, ob.bottomleft, ob.bottomright, 2)
        elif obstacle.imageOrientation == "left":
            pygame.draw.line(SCREEN, COLOUR, ob.topleft, ob.bottomleft, 2)

    def updateGrid(self, robot, SCREEN):
        SCREEN.fill((0,0,0))
        self.drawGrid(SCREEN)
        robot.drawCar(SCREEN)
