import time
import traceback

from algorithm.algo.Environment import AdvancedEnvironment
import itertools
from algorithm.algo.DubinsV2 import DubinsV2
from algorithm.algo.HStar import HybridAstar

class NearestNeighbour:
    def __init__(self, env: AdvancedEnvironment, start):
        self.env = env
        self.start = start
        self.targetLocations = env.targets
        self.dubins = DubinsV2(30.5, 19, self.env)

    def computeSequence(self):
        """
        returns the sequence of vertices to visit with minimum cost
        :return:
        sequence of verticles with their pos
        """
        permutations = list(itertools.permutations(self.targetLocations))
        choice = []
        optimalPath = []
        perm_cost_list = []
        for perm in permutations:
            perm_cost_list.append(self.distance_for_sequence(perm))
        min_cost_sequence = sorted(perm_cost_list, key=lambda x: x[1], reverse=False)[:15]
        clock = time.perf_counter()
        for perm in min_cost_sequence:
            path = self.findPath(perm[0])
            if path:
                choice.append(path)
                if time.perf_counter() - clock > 15:
                    break
        if choice:
            cost_list = []
            for path in choice:
                cost_list.append(path[1])
            optimalPath = choice[cost_list.index(min(cost_list, key=lambda x: x))]
        return optimalPath


    def euclideanDistance(self, start, end):

        return ((end[0]-start[0])**2 + (end[1]-start[1])**2)**0.5


    def findPath(self, targetLocations: list):
        """
        find dubins path
        :param targetLocations: list[tuple]
        :return: path
        """
        complete_path = []

        counter = 0
        start = self.start
        cost = 0
        for ob in targetLocations:
            try:
                hstar = HybridAstar(self.env, self.dubins, start, ob, False)
                next = hstar.solve()
                if next == None:
                    return None
                path = hstar.path
                for node in path:
                    cost += node.cost
            except Exception as e:
                print(traceback.format_exc())
                print("no path")
                break
            complete_path.append(path)
            counter += 1
            start = next.pos

        if counter != len(targetLocations):
            print("path is incomplete!!")

        return complete_path, cost

    def distance_for_sequence(self, sequence):
        distance = 0
        for i in range(len(sequence) - 1):
            distance += self.euclideanDistance(sequence[i], sequence[i + 1])
        return sequence, distance