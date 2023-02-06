import sys
import pygame
import algorithm.settings as settings
from algorithm.Entities.arena import Arena
from algorithm.Entities.Obstacle import Obstacle
from algorithm.Entities.Robot import Robot
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import DIRECTION
from algorithm.algo.TSP import NearestNeighbour
from algorithm.simulator import Simulator
from DataPopulator import getTestObstacles
from algorithm.constants import MOVEMENT

def main():
    obs = getTestObstacles()
    sim = Simulator(StaticEnvironment((200,200), obs), obs, False)
    sim.init()
    sim.run()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print(MOVEMENT.RIGHT.value)
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
