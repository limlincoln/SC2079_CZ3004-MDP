import pygame
from algorithm import settings
from algorithm.Entities.Rectangle import Rectangle
from algorithm.Entities.Obstacle import Obstacle
from itertools import groupby
"""
Represents the navigational area (default: 20x20 grid of 10x10cm grid cells)
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
        for x in range(0 + settings.GRID_OFFSET,self.width, self.blockSize):
            for y in range(0 + settings.GRID_OFFSET, self.height, self.blockSize):
                grid = pygame.Rect(x,y, self.blockSize, self.blockSize)
                pygame.draw.rect(SCREEN, settings.WHITE, grid, 1)
        for obstacle in self.obstacles:
            ob = pygame.Rect(obstacle.gridPosition, obstacle.dimension)
            ob.bottomleft = obstacle.gridPosition
            self.obList.append(ob)
            pygame.draw.rect(SCREEN, settings.GREEN, ob)
            self.drawBorder(obstacle, SCREEN, RED, ob)
            self.drawInvisibleObstacle(obstacle,SCREEN, (0, 100, 255))

    def drawBorder(self, obstacle,  SCREEN, COLOUR, ob):
        if obstacle.imageOrientation == "N":
            pygame.draw.line(SCREEN, COLOUR, ob.topleft, ob.topright, 2)
        elif obstacle.imageOrientation == "E":
            pygame.draw.line(SCREEN, COLOUR, ob.topright, ob.bottomright, 2)
        elif obstacle.imageOrientation == "S":
            pygame.draw.line(SCREEN, COLOUR, ob.bottomleft, ob.bottomright, 2)
        elif obstacle.imageOrientation == "W":
            pygame.draw.line(SCREEN, COLOUR, ob.topleft, ob.bottomleft, 2)

    def drawInvisibleObstacle(self, obstacle: Obstacle, SCREEN, COLOUR):
        newRect = Rectangle(obstacle.pos, 'O')
        dim = ((newRect.length//10)*settings.BLOCK_SIZE, (newRect.length//10)*settings.BLOCK_SIZE)
        rectOb = pygame.Rect(obstacle.pos, dim)
        rectOb.topleft = self.posConverter((newRect.x, newRect.y))
        pygame.draw.line(SCREEN, COLOUR, rectOb.topleft, rectOb.topright, 2)
        pygame.draw.line(SCREEN, COLOUR, rectOb.topright, rectOb.bottomright, 2)
        pygame.draw.line(SCREEN, COLOUR, rectOb.bottomleft, rectOb.bottomright, 2)
        pygame.draw.line(SCREEN, COLOUR, rectOb.topleft, rectOb.bottomleft, 2)
    def drawStuff(self, stuff: list[tuple], SCREEN, COLOUR):
        for s in stuff:
            pygame.draw.circle(SCREEN, COLOUR, self.posConverter(s), 20)

    def updateGrid(self, robot, SCREEN):
        SCREEN.fill((0,0,0))
        self.drawGrid(SCREEN)
        robot.drawCar(SCREEN)
        self.robotCollisionRect(robot,SCREEN, (0, 100, 255))


    def robotCollisionRect(self, robot, screen, colour):
        newRect = Rectangle((robot.x,robot.y), 'R')
        dim = ((newRect.length // 10) * settings.BLOCK_SIZE, (newRect.length // 10) * settings.BLOCK_SIZE)
        rectOb = pygame.Rect((newRect.x, newRect.y), dim)
        rectOb.topleft = self.posConverter((newRect.x, newRect.y))
        pygame.draw.line(screen, colour, rectOb.topleft, rectOb.topright, 2)
        pygame.draw.line(screen, colour, rectOb.topright, rectOb.bottomright, 2)
        pygame.draw.line(screen, colour, rectOb.bottomleft, rectOb.bottomright, 2)
        pygame.draw.line(screen, colour, rectOb.topleft, rectOb.bottomleft, 2)

    @staticmethod
    def posConverter(pos):
        return (pos[0] // settings.GRID_SCALE_FACTOR) * settings.BLOCK_SIZE + settings.GRID_OFFSET, \
                            (settings.GRID_Y_OFFSET - (pos[1] // settings.GRID_SCALE_FACTOR) * settings.BLOCK_SIZE) + \
                            settings.GRID_OFFSET

    @staticmethod
    def drawPath(path: list):
        pass

