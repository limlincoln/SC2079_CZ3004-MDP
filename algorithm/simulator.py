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
        self.timer = None
        self.text = None
        self.font = None
        self.optimalCoords = None
        self.shortestPath = shortestPath
        self.timeCounter = 0
        self.textTimer = None
        self.pause = False
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
        else:
            TSP = NearestNeighbour(self.env, (0,0, DIRECTION.TOP, 'P'))
            TSP.computeSequence()
            self.commandList = TSP.getCommandList()
            self.optimalCoords = TSP.getOptimalWithCoords()
            print(TSP.getSTMCommands(self.optimalCoords))

        pygame.display.set_caption("Starting simulator....")
        self.arena = Arena(self.obstacles, 400 + settings.GRID_OFFSET, 400 + settings.GRID_OFFSET, settings.BLOCK_SIZE)
        self.arena.drawStuff(self.env.generateTargetLocation(), self.screen, settings.GREEN)
        self.robot = Robot(self.arena.obList)
        self.arena.drawGrid(self.screen)
        self.robot.drawCar(self.screen)
        self.moveCar = pygame.USEREVENT + 0
        self.timer = pygame.USEREVENT + 1
        pygame.time.set_timer(self.moveCar, 1000)
        pygame.time.set_timer(self.timer, 1000)


    def render(self):
        """
        Set up screen
        :return:
        """
        if len(self.commandList) > 0 and self.commandCounter <= len(self.commandList)-1:
            self.text = self.font.render("Command:" + self.commandList[self.commandCounter][1], True, settings.GREEN, settings.BLUE)
            self.text.get_rect().center = (600,400)
            self.screen.blit(self.text, self.text.get_rect())
            direction = self.font.render("Direction:" + self.commandList[self.commandCounter][0].name, True, settings.GREEN, settings.BLUE)
            direction.get_rect().center = (600, 200)
            self.screen.blit(direction, (0,50))
            self.textTimer = self.font.render("Time ( secs):" +str(self.timeCounter), True, settings.GREEN, settings.BLUE)
            self.textTimer.get_rect().center = (600, 600)
            self.screen.blit(self.textTimer,(0, 80))

    def events(self):
        """
        event handling
        :return:
        """

        for event in pygame.event.get():

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_p:
                    self.pause = not self.pause
                elif event.key == pygame.K_LEFT and self.pause and self.commandCounter > 0:
                    self.commandCounter -= 1
                elif event.key == pygame.K_RIGHT and self.pause and self.commandCounter <= len(self.commandList) - 1:
                    self.commandCounter += 1

            if event.type == self.moveCar:
                if self.commandCounter <= len(self.commandList)-1:
                    if self.robot.command is None:
                        self.robot.setCurrentCommand(self.optimalCoords[self.commandCounter][3])
                    elif self.robot.command.tick == 0 and not self.pause:
                        self.commandCounter += 1
                        if self.commandCounter <= len(self.commandList) - 1:
                            self.robot.setCurrentCommand(self.optimalCoords[self.commandCounter][3])
                if self.commandCounter <= len(self.commandList) - 1:
                    self.robot.moveToDo(self.optimalCoords[self.commandCounter])
                if self.robot.command is not None and  self.robot.command.tick > 0:
                    self.robot.command.yoloTick()
                self.arena.updateGrid(self.robot, self.screen)
                self.arena.drawStuff(self.env.generateTargetLocation(), self.screen, settings.GREEN)

            if event.type == self.timer and not self.pause:
                self.timeCounter += 1

            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        pygame.display.update()

    def run(self):
        """run for real"""
        while self.running:
            self.events()
            self.render()
            self.clock.tick(30)


