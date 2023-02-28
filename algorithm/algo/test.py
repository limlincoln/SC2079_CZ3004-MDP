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
    path = []
    for options in env.targets:
        hstar = HybridAstar(env, dubins, start, options, True)
        hstar.solve()
        path.append(hstar.path)
        start = options
    return path

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
        dubins = DubinsV2(28, 10, env)
        path = sampleHstarTSP((15,15, DIRECTION.TOP.value), env, dubins)
        coords  = {}
        for index , node  in enumerate(path):
            coords[index] = node

        points = []
        for path in coords:
            points.extend([x.path for x in coords[path]])
        print(points[0])
        dubins.plot(points)
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
        dubins.plot(path)

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

    def testTask2(self):
        env = AdvancedEnvironment((200, 400), [Obstacle((60, 200), 'W', (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '2')], 2)
        dubins = DubinsV2(28, 10, env)
        start = (30, 200, DIRECTION.RIGHT.value)
        end = (60, 250, DIRECTION.RIGHT.value)
        path = dubins.computeAllPath(start, end)
        coord_list = []
        for p in path:
            coords = dubins.generatePathCoords(start,end,p)
            coord_list.append(coords)

        dubins.plot(coord_list)

    def testTSP(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles2())
        tsp = NearestNeighbour(env, (15, 15, DIRECTION.TOP.value))
        path = tsp.computeSequence()
        print(path)
        pathString = []
        for oneOb in path[0]:
            key = oneOb[-1].pos[3]
            tempString = ""
            for index , node in enumerate(oneOb):
                trail = ","
                if type(node.moves) == tuple:
                    for i, dubinsTuple in enumerate(node.moves[:3]):
                        if i == len(node.moves[:3])-1 and index == len(oneOb) - 1:
                            trail = ""
                        tempString += dubinsTuple + trail
                else:
                    if index == len(oneOb) - 1:
                        trail = ""
                    tempString += str(node.moves) + trail
            pathString.append((key, tempString))

        print(pathString)
        # (ObstacleID, "s3.012,s1.204")




if __name__ == '__main__':
    unittest.main()