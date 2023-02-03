import sys
import pygame
import algorithm.settings as settings
from algorithm.Entities.arena import Arena
from algorithm.Entities.Obstacle import Obstacle
from algorithm.Entities.Robot import Robot
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import DIRECTION
from algorithm.algo.TSP import NearestNeighbour

class Simulator:
    """
    Main class for the simulator
    """

    def __init__(self, env, obstacles):
        self.running = False
        self.commandList = []
        self.screen = None
        self.clock = None
        self.env = env
        self.arena = None
        self.obstacles = obstacles
        self.robot = None
        self.commandCounter = 0
        self.moveCar = None
    def init(self):
        """
        set up the simulator
        :return:
        """
        pygame.init()
        self.running = True
        self.screen = pygame.display.set_mode((settings.WINDOW_WIDTH, settings.WINDOW_HEIGHT))
        self.clock = pygame.time.Clock()
        self.screen.fill(settings.BLACK)
        pygame.display.set_caption("Calculating Path...")
        TSP = NearestNeighbour(self.env, (0,0, DIRECTION.TOP, 'P'))
        self.commandList = TSP.computeSequence()
        print("path:", self.commandList)
        pygame.display.set_caption("Starting simulator....")
        self.arena = Arena(self.obstacles, 400 + settings.GRID_OFFSET, 400 + settings.GRID_OFFSET, settings.BLOCK_SIZE)
        self.arena.drawStuff(self.env.generateTargetLocation(), self.screen, settings.GREEN)
        self.robot = Robot(self.arena.obList)
        self.arena.drawGrid(self.screen)
        self.robot.drawCar(self.screen)
        self.moveCar = pygame.USEREVENT + 0
        pygame.time.set_timer(self.moveCar, 1000)

    def render(self):
        """
        Set up screen
        :return:

        """


    def events(self):
        """
        event handling
        :return:
        """

        for event in pygame.event.get():
            if event.type == self.moveCar:
                if self.commandCounter < len(self.commandList):
                    print("working")
                    self.robot.moveToDo(self.commandList[self.commandCounter], self.screen)
                    self.commandCounter += 1
                self.arena.updateGrid(self.robot, self.screen)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update()

    def run(self):
        """run for real"""
        while self.running:
            self.events()
            #self.render()


