import sys
import pygame
from tkinter import *
from tkinter import messagebox
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

    def __init__(self, env: StaticEnvironment, obstacles: list[Obstacle], shortestPath: bool):
        self.running = False
        self.commandList = []
        self.screen: pygame.Surface = None
        self.clock = None
        self.env = env
        self.arena = None
        self.obstacles = obstacles
        self.robot: Robot = None
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
        self.scanCounter = 0
        self.scanText = None
        self.ScanCheck = None
        self.done = False
        Tk().wm_withdraw()  # to hide the main window


    def init(self):
        """
        set up the simulator with screen, clock, arena  and the pathing algo
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

            TSP = Astar(self.env, (0,0, DIRECTION.TOP, 'P'), self.env.getTargetLocation()[0])
            TSP.computePath()
            self.optimalCoords = TSP.getPath()
            self.commandList = TSP.getCommandPath()
        else:
            TSP = NearestNeighbour(self.env, (0,0, DIRECTION.TOP, 'P'))
            TSP.computeSequence()
            self.optimalCoords = TSP.getOptimalWithCoords()
            self.commandList = TSP.getSTMCommands(self.optimalCoords)

        pygame.display.set_caption("Starting simulator....")
        self.ScanCheck = [x for x in self.env.getTargetLocation()]
        self.arena = Arena(self.obstacles, 400 + settings.GRID_OFFSET, 400 + settings.GRID_OFFSET, settings.BLOCK_SIZE)
        self.arena.drawStuff(self.env.getTargetLocation(), self.screen, settings.GREEN)
        self.robot = Robot(self.arena.obList)
        self.arena.drawGrid(self.screen)
        self.robot.drawCar(self.screen)
        self.moveCar = pygame.USEREVENT + 0
        self.timer = pygame.USEREVENT + 1
        pygame.time.set_timer(self.moveCar, 1000)
        pygame.time.set_timer(self.timer, 1000)
        pygame.display.set_caption("Car go vroom vroom")


    def render(self):
        """
        Set up text for screen
        :return: None
        """
        if len(self.commandList) > 0 and self.commandCounter <= len(self.commandList)-1:
            self.text = self.font.render("Command: " + self.commandList[self.commandCounter], True, settings.GREEN, settings.BLUE)
            self.text.get_rect().center = (600,400)
            self.screen.blit(self.text, self.text.get_rect())
            direction = self.font.render("Direction: " + self.optimalCoords[self.commandCounter][2].name, True, settings.GREEN, settings.BLUE)
            direction.get_rect().center = (600, 200)
            self.screen.blit(direction, (0,30))
            self.textTimer = self.font.render("Time ( secs): " +str(self.timeCounter), True, settings.GREEN, settings.BLUE)
            self.textTimer.get_rect().center = (600, 600)
            self.screen.blit(self.textTimer,(0, 60))
        self.scanText = self.font.render("Image Scanned: " +str(self.scanCounter), True, settings.GREEN, settings.BLUE)
        self.scanText.get_rect().center = (600,600)
        self.screen.blit(self.scanText, (500, 100))
    def events(self):
        """
        event handling every frame
        :return:
        """

        # pop up window after the simulator is done
        if self.scanCounter == 5 and not self.done:
            messagebox.showinfo('ALL Images Found', 'OK')
            pygame.display.set_caption("Pathing Done")
            self.done = True
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_p:
                    self.pause = not self.pause
                elif event.key == pygame.K_LEFT and self.pause and self.commandCounter > 0:
                    self.commandCounter -= 1
                    self.timeCounter = 0
                elif event.key == pygame.K_RIGHT and self.pause and self.commandCounter <= len(self.commandList) - 1:
                    self.commandCounter += 1
                    self.timeCounter = 0
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
                # check if already scan image:
                pos = (self.robot.pos[0], self.robot.pos[1], self.robot.orientation)
                if pos in self.ScanCheck:
                    self.scanCounter += 1
                    self.ScanCheck.remove(pos)
                self.arena.updateGrid(self.robot, self.screen)
                self.arena.drawStuff(self.env.getTargetLocation(), self.screen, settings.GREEN)
            if event.type == self.timer and not self.pause:
                self.timeCounter += 1

            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        pygame.display.flip()

    def run(self):
        """run for real"""
        while self.running:
            self.events()
            self.render()
            self.clock.tick(30)


