import sys
import pygame
import algorithm.settings as settings
from algorithm.Entities.arena import Arena
from algorithm.Entities.Obstacle import Obstacle
from algorithm.Entities.Robot import Robot
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import DIRECTION
from algorithm.algo.TSP import NearestNeighbour
from algorithm.algo.Astar import Astar
class Simulator:
    """
    Main class for the simulator
    """

    def __init__(self, env, obstacles, shortestPath):
        self.running = False
        self.commandList = []
        self.screen : pygame.Surface = None
        self.clock = None
        self.env = env
        self.arena = None
        self.obstacles = obstacles
        self.robot = None
        self.commandCounter = 0
        self.moveCar = None
        self.text = None
        self.font = None
        self.optimalCoords = None
        self.shortestPath = shortestPath
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
        self.font = pygame.font.Font('assets/font.ttf', 32)
        text = self.font.render("Initiating.... Please Wait", True, settings.GREEN, settings.BLUE)
        self.text = text
        self.text.get_rect().center = (600, 400)
        self.screen.blit(self.text, self.text.get_rect())
        if self.shortestPath:

            TSP = Astar(self.env, (0,0, DIRECTION.TOP, 'P'), self.env.generateTargetLocation()[0])
            TSP.computePath()
            self.optimalCoords = TSP.getPath()
            self.commandList = TSP.getCommandPath()
            print("yo")
        else:
            TSP = NearestNeighbour(self.env, (0,0, DIRECTION.TOP, 'P'))
            TSP.computeSequence()
            self.commandList = TSP.getCommandList()
            self.optimalCoords = TSP.getOptimalWithCoords()

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
        if len(self.commandList) > 0:
            self.text = self.font.render("Command:" + self.commandList[self.commandCounter][1], True, settings.GREEN, settings.BLUE)
            self.text.get_rect().center = (600,400)
            self.screen.blit(self.text, self.text.get_rect())
            direction = self.font.render("Direction:" + self.commandList[self.commandCounter][0].name, True, settings.GREEN, settings.BLUE)
            direction.get_rect().center = (600, 200)
            self.screen.blit(direction, (0,50))

    def events(self):
        """
        event handling
        :return:
        """

        for event in pygame.event.get():
            if event.type == self.moveCar:
                if self.commandCounter < len(self.commandList)-1:
                    self.robot.moveToDo(self.optimalCoords[self.commandCounter], self.screen)
                    self.commandCounter += 1
                self.arena.updateGrid(self.robot, self.screen)
                self.arena.drawStuff(self.env.generateTargetLocation(), self.screen, settings.GREEN)
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update()

    def run(self):
        """run for real"""
        while self.running:
            self.events()
            self.render()


