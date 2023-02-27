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
import numpy as pi
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
        sampleHstarTSP((15,15,DIRECTION.TOP.value), env, dubins)
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
        astar = HybridAstar(env,dubins, (15,15,DIRECTION.TOP.value), env.targets[0], True)
        astar.solve()
        path = dubins.computeAllPath((15,15,DIRECTION.TOP.value), env.targets[0])
        print(path)

if __name__ == '__main__':
    unittest.main()