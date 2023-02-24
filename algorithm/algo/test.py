import re
import unittest
import algorithm.settings as settings
from algorithm.constants import DIRECTION
from algorithm.algo.Dubins import Dubins
from algorithm.algo.Environment import StaticEnvironment, AdvancedEnvironment
from algorithm.Entities.Obstacle import Obstacle
from Astar import Astar
from algorithm.DataPopulator import getTestObstacles, getTestObstacles1
from algorithm.simulator import Simulator
from algorithm.algo.DubinsV2 import DubinsV2
import numpy as np
from algorithm.algo.TSPV2 import NearestNeighbour
class SimSum(unittest.TestCase):


    def testRegexString(self):
        ObList = {}

        s1 = "Obstacle: 1, Col: 1, Row: 19"
        s2 = "Obstacle: 1, Facing: E"
        s3 = "Obstacle: 2, Facing: W"
        s4 = "Obstacle: 2, Col: 1, Row: 8"
        s5 = "Obstacle: 1, Col: 3, Row: 2"
        def obstacle_string_converter(s: str):
            """
            converts string to useable tuple for Obstacle object instantiation
            :param s: string
            :return: tuple
            """
            s_to_list = s.split(',')
            values = re.findall(r'\d+', s)
            if len(s_to_list) == 3:
                if ObList.get(values[0]) is None:
                    ObList[values[0]] = [(int(values[1]), int(values[2])), None, (settings.BLOCK_SIZE, settings.BLOCK_SIZE),values[0]]
                else:
                    ObList[values[0]][0] = (int(values[1]), int(values[2]))
            else:
                direction = re.findall(r'\b[A-Z]+(?:\s+[A-Z]+)*\b', s_to_list[1])
                if ObList.get(values[0]) is None:
                    ObList[values[0]] = [None, direction[0], (settings.BLOCK_SIZE, settings.BLOCK_SIZE), values[0]]
                else:
                    ObList[values[0]][1] = direction[0]

        obstacle_string_converter(s1)
        obstacle_string_converter(s2)
        obstacle_string_converter(s3)
        obstacle_string_converter(s4)
        obstacle_string_converter(s5)

    def testEnvironment(self):
        env = AdvancedEnvironment((200,200), getTestObstacles())
        dubins = DubinsV2(28, 10, env)
        path = dubins.compute_best((60,60, DIRECTION.TOP.value), (105, 105, DIRECTION.LEFT.value))
        print(path)

if __name__ == '__main__':
    unittest.main()
