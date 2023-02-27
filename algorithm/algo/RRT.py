from algorithm.algo.Environment import AdvancedEnvironment
import numpy as np
from algorithm.algo.DubinsV2 import DubinsV2


def dist(a, b):
    return np.sqrt(np.square(a[0] - b[0]) + np.square(a[1] - b[1]))


class Node:
    def __init__(self, pos, time, cost):
        self.pos = pos
        self.cost = cost
        self.time = time
        self.destination_list = []


class Edge:
    """

    """

    def __init__(self, node_from, node_to, path, cost):
        self.node_from = node_from
        self.node_to = node_to
        self.path = path
        self.cost = cost


class RRT:
    def __init__(self, env: AdvancedEnvironment, dubins: DubinsV2, start, goal, pre=(5, 5, 1)):
        self.nodes = {}
        self.edges = {}
        self.env = env
        self.dubins = dubins
        self.goal = goal
        self.pre = pre
        self.start = start

    def run(self, nb_iteration=100, goal_rate=.5):

        self.nodes[self.start] = Node(self.start, 0, 0)

        for _ in range(nb_iteration):
            if np.random.rand() > 1 - goal_rate:
                sample = self.goal
            else:
                sample = self.env.randomFreeSpace()

            options = self.select_options(sample, 10)
            for node, option in options:
                if option[0] == float('inf'):
                    break
                path = self.dubins.collision_check(option, node, sample)

                if path:
                    self.nodes[sample] = Node(sample, self.nodes[node].time + option[0],
                                              self.nodes[node].cost + option[0])
                    self.nodes[node].destination_list.append(sample)
                    self.edges[node, sample] = Edge(node, sample, option, option[0])

                    if self.in_goal_region(sample):
                        return
                    break

    def select_options(self, sample, nb_options, metric="local"):
        if metric == "local":
            options = []
            for node in self.nodes:
                options.extend([(node, opt) for opt in self.dubins.computeAllPath(node, sample)])

            options.sort(key=lambda x: x[1][0])
            options = options[:nb_options]
        else:
            # Euclidian distance as a metric
            options = [(node, (node, sample)) for node in self.nodes]
            options.sort(key=lambda x: x[1])
            options = options[:nb_options]
            new_opt = []
            for node, _ in options:
                db_options = self.dubins.computeAllPath(node, sample)
                new_opt.append((node, min(db_options, key=lambda x: x[0])))
            options = new_opt
        return options

    def in_goal_region(self, sample):
        for i, value in enumerate(sample):
            if i == len(self.goal) -1:
                break
            if abs(self.goal[i] - value) > self.pre[i]:
                return False
        return True

    def select_best_edge(self):
        """

        :return:
        """
        node = max([(child, self.children_count(child))\
                    for child in self.nodes[self.start].destination_list],
                   key=lambda x: x[1])[0]
        best_edge = self.edges[(self.start, node)]

        for child in self.nodes[self.start].destination_list:
            if child == node:
                continue
            self.edges.pop((self.start, child))
            self.delete_all_children(child)
        self.nodes.pop(self.start)
        self.start = node
        return best_edge


    def delete_all_children(self, node):
        """

        :param node:
        :return:
        """

        if self.nodes[node].destination_list:
            for child in self.nodes[node].destination_list:
                self.edges.pop((node, child))
                self.delete_all_children(child)
        self.nodes.pop(node)


    def children_count(self, node):
        """

        :param node:
        :return:
        """

        if not self.nodes[node].destination_list:
            return 0
        total = 0
        for child in self.nodes[node].destination_list:
            total += 1 + self.children_count(child)
        return total
