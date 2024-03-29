
import re
import time
import unittest
import algorithm.settings as settings
from algorithm.constants import DIRECTION
from algorithm.algo.Dubins import Dubins
from algorithm.algo.Environment import StaticEnvironment, AdvancedEnvironment
from algorithm.Entities.Obstacle import Obstacle
from Astar import Astar
from algorithm.DataPopulator import getTestObstacles, getTestObstacles1, getTestObstacles2, getTestObstacles3
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


def task2Path(dubins, start, pathList):
    coord_list = []
    command_list = []
    for option in pathList:
        path = dubins.compute_best_task2(start, option)
        command_list.append(dubins.path_converterV2(path, option))
        coords = dubins.generatePathCoords(start, option, path)
        coord_list.append(coords)
        start = option

    return coord_list, command_list


def convertToPath(path):
    pathString = []
    print(path)
    for oneOb in path:
        print(oneOb[-1].pos)
        key = oneOb[-1].pos[3]
        tempString= ""
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
    return pathString
    
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
        start = (15, 15, DIRECTION.TOP.value)
        end = env.targets[4]
        path = dubins.compute_best(start,end)
        dubins.plot(path)

    def testFailedObstacle(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(26, 10, env)
        start = (15, 15, DIRECTION.TOP.value)
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
        distance_offset_1 = 90
        distance_offset_2 = 90
        env = AdvancedEnvironment((400, 160), [Obstacle((60, 80), 'W', (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '2'), Obstacle((70+60,80),'S', (settings.BLOCK_SIZE, settings.BLOCK_SIZE), '3')], 2)
        dubins = DubinsV2(30, 76, env)
        start = (0,80, DIRECTION.RIGHT.value)
        # for left, left
        path_list_left_left = [(60+5, 105, DIRECTION.RIGHT.value),(70+60+5, 80+55, DIRECTION.RIGHT.value), (70+60+10+50, 80, DIRECTION.BOTTOM.value), (70+60+10, 80-55, DIRECTION.LEFT.value), (0,80, DIRECTION.LEFT.value)]
        # for left, right
        path_list_left_right = [(60+5, 105, DIRECTION.RIGHT.value),(70+60+5, 80-55, DIRECTION.RIGHT.value), (70+60+10+50, 80, DIRECTION.TOP.value), (70+60+10, 80+55, DIRECTION.LEFT.value),(0,80, DIRECTION.LEFT.value)]

        # for right, right =
        path_list_right_right = [(60+5, 55, DIRECTION.RIGHT.value),(70+60+5, 80-55, DIRECTION.RIGHT.value), (70++60+10+50, 80, DIRECTION.TOP.value), (70+60+10, 80+55, DIRECTION.LEFT.value), (0,80, DIRECTION.LEFT.value)]
        # for right, left
        path_list_right_left = [(60+5, 55, DIRECTION.RIGHT.value), (70+60+5, 80+55, DIRECTION.RIGHT.value), (70+60+10+50, 80, DIRECTION.BOTTOM.value), (70+60+10,80-55, DIRECTION.LEFT.value),(60,80-55,DIRECTION.LEFT.value), (0,80,DIRECTION.LEFT.value)]
        coord_list, command_list = task2Path(dubins,start,path_list_right_left)
        print(command_list)
        dubins.plot(coord_list)

    def testTSPForRPI(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        tsp = NearestNeighbour(env, (15, 15, DIRECTION.TOP.value))
        path = tsp.computeSequence()[0]
        print(path)
        pathString = []
        for oneOb in path:
            print(oneOb)
            key = oneOb[-1].pos[3]
            tempString = ""
            coords = []
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
                coords.extend(node.path)
            pathString.append((key, tempString, coords))
        print(pathString)
        # (ObstacleID, "s3.012,s1.204")

    def testTSPPlot(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        tsp = NearestNeighbour(env, (15, 15, DIRECTION.TOP.value))
        clock = time.perf_counter()
        path = tsp.computeSequence()[0]
        end = time.perf_counter()

        print("time taken to calculate path:", end-clock)
        coords = []
        for ob in path:
            ob_coords = []
            for node in ob:
                ob_coords.extend(node.path)
            coords.append(ob_coords)
        tsp.dubins.save_path(coords)
        print(convertToPath(path))


    def testtesttest(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(28, 10, env)
        start = (15, 15, DIRECTION.TOP.value)
        hstar = HybridAstar(env, dubins, start, env.targets[0], reverse=True)
        hstar.solve()
        path = hstar.path
        print(path)
        temp = []

        for node in path:
            coords = node.path
            temp.append(coords)

        dubins.plot(temp)


    def test(self):
        env = AdvancedEnvironment((200, 200), getTestObstacles())
        dubins = DubinsV2(28,10,env)
        start = (15, 15, DIRECTION.TOP.value)
        hstar = HybridAstar(env, dubins, start, env.targets[0], reverse=True)
        points, final = hstar.generateState((50, 50, DIRECTION.RIGHT.value), 'RR', 0.5)
        print(final)
        print(points[-1])
        points2, final = hstar.generateState(final, 'RL', 0.5)
        print(final)
        print(points2[-1])
        dubins.plot([points, points2])


    def testDubinsCurves(self):
        env = AdvancedEnvironment((200,200), getTestObstacles3())
        dubins = DubinsV2(28,10,env)
        start = (80,60,DIRECTION.RIGHT.value)
        options = dubins.computeAllPath(start, env.targets[4])
        coords_list = []
        option = options[3]
        path = dubins.generatePathCoords(start, env.targets[4], option)
        print(option)

        coords_list.append(path)

        dubins.plot(coords_list)


if __name__ == '__main__':
    unittest.main()

