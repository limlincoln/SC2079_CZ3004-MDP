import re
import unittest
import algorithm.settings as settings
from algorithm.constants import DIRECTION
from algorithm.algo.Dubins import Dubins
from algorithm.algo.Environment import StaticEnvironment, AdvancedEnvironment
from algorithm.Entities.Obstacle import Obstacle
from Astar import Astar
from algorithm.DataPopulator import getTestObstacles, getTestObstacles1, getTestObstacles2
from algorithm.simulator import Simulator
from algorithm.algo.DubinsV2 import DubinsV2
import numpy as np
from algorithm.algo.TSPV2 import NearestNeighbour
from algorithm.algo.HStar import HybridAstar
from algorithm.algo.RRT import RRT
from algorithm.HelperFunctions import basic_angle
def sampleTSP(dubins, env, start):
    for options in env.targets:
        print(dubins.compute_best(start, options))
        start = options

def sampleHstarTSP( start, env, dubins):
    for options in env.targets:
        hstar = HybridAstar(env, dubins, start, options, True)
        path = hstar.solve()
        print(path.moves)
        start = options

class SimSum(unittest.TestCase):


    def testEnvironment(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles1())
        tsp = NearestNeighbour(env, (15, 15, DIRECTION.TOP.value))
       # path = tsp.computeSequence()
       # print(path)

    def testdubins(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(26,10, env)
       # sampleTSP(dubins, env, (15, 15, DIRECTION.TOP.value))
    def testHstar(self):
        env = AdvancedEnvironment((200,200), getTestObstacles())
        dubins = DubinsV2(26, 10, env)
        sampleHstarTSP((15,15, DIRECTION.TOP.value), env, dubins)
    def testRRT(self):
        env = AdvancedEnvironment((200,200), getTestObstacles())
        dubins = DubinsV2(26,10,env)
        rrt = RRT(env, dubins, (15,15,DIRECTION.TOP.value), env.targets[1])
        print(env.targets[1])
        rrt.run()
        edge = rrt.select_best_edge()
        print(edge.node_from, edge.path, edge.node_to)
        edge = rrt.select_best_edge()
        print(edge.node_from, edge.path, edge.node_to)

    def testOneObstacle(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(26,10,env)
        start = (15,15,DIRECTION.TOP.value)
        end = env.targets[4]
        path = dubins.compute_best(start,end)
        dubins.plot(path[1])

    def testFailedObstacle(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(26,10,env)
        start = (15,15,DIRECTION.TOP.value)
        end = env.targets[2]
        path = dubins.computeAllPath(start, end)
        coords = dubins.generatePathCoords(start, end, path[1])
        dubins.plot(coords)

    def testContinousValue(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        def newPos(pos, type, delta):
            assert type in 'RSL'
            if type == "S":
                new_X = pos[0] + delta * np.cos(pos[2])
                new_Y = pos[1] + delta * np.sin(pos[2])
                new_orientation = pos[2]
            elif type == "R":
                new_X = pos[0] + delta * np.cos(pos[2])
                new_Y = pos[1] + delta * np.sin(pos[2])
                new_orientation = basic_angle(pos[2] - delta / 25)
            elif type == "L":
                new_X = pos[0] + delta * np.cos(pos[2])
                new_Y = pos[1] + delta * np.sin(pos[2])
                new_orientation = basic_angle(pos[2] + delta / 25)
            return new_X, new_Y, new_orientation
        length = 40
        start = (50, 60, DIRECTION.TOP.value)
        points = []
        current = start
        for x in np.arange(0, length, 0.5):
            current = newPos(current, 'R', 0.5)
            points.append(current)
        dubins = DubinsV2(28, 10, env)
        print(points[-1])
        dubins.plot(points)



if __name__ == '__main__':
    unittest.main()