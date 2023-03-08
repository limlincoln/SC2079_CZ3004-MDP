import math

import numpy as np
from algorithm.algo.Environment import AdvancedEnvironment
import matplotlib.pyplot as plt
import matplotlib
from algorithm.HelperFunctions import basic_angle , radiansToDegrees

class DubinsV2:
    def __init__(self, radius, velocity, env: AdvancedEnvironment):
        """
        """
        self.env = env
        self.radius = radius
        self.velocity = velocity
        self.turningSpeed = (np.pi*self.radius)/7.2

    def computeAllPath(self, start, end):
        """
        compute all the possible paths for dubins
        let center_0_x be the turning circle at the start and center_1_x be the turning circle at the end
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :return: all options for the path
        """
        center_0_right = self.findCenter(start, 'R')
        center_0_left = self.findCenter(start, 'L')
        center_1_right = self.findCenter(end, 'R')
        center_1_left = self.findCenter(end, 'L')

        options = [self.lsl(start, end, center_0_left, center_1_left),
                   self.rsr(start, end, center_0_right, center_1_right),
                   self.rsl(start, end, center_0_right, center_1_left),
                   self.lsr(start, end, center_0_left, center_1_right),
                   self.rlr(start, end, center_0_right, center_1_right),
                   self.lrl(start, end, center_0_left, center_1_left)]
        copy = options.copy()
        for option in options:
            if np.isnan(option[0]) or option[0] == 999:
                copy.pop(copy.index(option))
        return copy

    def compute_best(self, start, end):
        """
        start the calculation of dubins path and return the shortest
        :param start: tuple
        :param end: tuple
        :return: path and converted to time stamp
        """
        paths = self.computeAllPath(start, end)
        loopPath = paths.copy()
        for path in loopPath:
            points = self.generatePathCoords(start, end, path, 2)
            for point in points:
                if not self.env.isWalkable(point):
                    paths.pop(paths.index(path))
                    break
        if not paths:
            return None
        best = min(paths, key=lambda x: x[0])
        commands = self.path_converter(best, end)
        coords = self.generatePathCoords(start, end, best, 2)
        return commands, coords

    def dubins_distance(self, start, end):
        paths = self.computeAllPath(start, end)
        best = min(paths, key=lambda x:x[0])
        return best[0]

    def collision_check(self, path, start, end):
        points = self.generatePathCoords(start, end, path, 2)
        for point in points:
            if not self.env.isWalkable(point):
                return False
        return True

    def path_converter(self, path, end):
        """
        convert the path into time that can be used to control the robot
        :param path: tuple
        :return: tuple
        """
        command = None
        if path[4] == 'LSL':
            command = ("l"+str(int(np.round(path[2] / self.turningSpeed, 4) * 1000)), "f"+str(int(np.round(path[1] / self.velocity, 4)* 1000)), "l"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'lsl'), end
        elif path[4] == 'RSR':
            command = ("r"+str(int(np.round(path[2] / self.turningSpeed, 4) * 1000)), "f"+str(int(np.round(path[1] / self.velocity, 4)* 1000)), "r"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'rsr'), end
        elif path[4] == 'RSL':
            command = ("r"+str(int(np.round(path[2] / self.turningSpeed, 4) * 1000)), "f"+str(int(np.round(path[1] / self.velocity, 4)* 1000)), "l"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'rsl'), end
        elif path[4] == 'LSR':
            command = ("l"+str(int(np.round(path[2] / self.turningSpeed, 4) * 1000)), "f"+str(int(np.round(path[1] / self.velocity, 4)* 1000)), "r"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'lsr'), end
        elif path[4] == 'RLR':
            command = ("r"+str(int(np.round(path[1] / self.turningSpeed, 4) * 1000)), "l"+str(int(np.round(path[2] / self.turningSpeed, 4)* 1000)), "r"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'rlr'), end
        elif path[4] == 'LRL':
            command = ("l"+str(int(np.round(path[1] / self.turningSpeed, 4) * 1000)), "r"+str(int(np.round(path[2] / self.turningSpeed, 4)* 1000)), "l"+str(int(np.round(path[3] / self.turningSpeed, 4)* 1000)), 'lrl'), end
        return command

    def lsl(self, start, end, p1, p2):
        """
        left turn then straight then left turn again
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        straight = self.distCenter(p1, p2)

        vector1 = (p2[0] - p1[0], p2[1] - p1[1])
        # rotate the angle by pi/2 clockwise
        vector2 = (vector1[1], -vector1[0])

        # finding the tangent points to the straight path
        pt1 = (p1[0] + (self.radius / straight) * vector2[0], p1[1] + (self.radius / straight) * vector2[1])
        pt2 = (pt1[0] + vector1[0], pt1[1] + vector1[1])
        # find the arc length
        arc1 = self.findArcLength(pt1, p1, start, 'L')
        arc2 = self.findArcLength(end, p2, pt2, 'L')
        total_len = straight + arc1 + arc2
        return total_len, straight, arc1, arc2, 'LSL', [pt1, pt2]

    def rsr(self, start, end, p1, p2):
        """
        left turn then straight then left turn again
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        straight = self.distCenter(p1, p2)

        vector1 = (p2[0] - p1[0], p2[1] - p1[1])
        # rotate the angle by pi/2 anticlockwise
        vector2 = (-vector1[1], vector1[0])

        # finding the tangent points to the straight path
        pt1 = (p1[0] + (self.radius / straight) * vector2[0], p1[1] + (self.radius / straight) * vector2[1])
        pt2 = (pt1[0] + vector1[0], pt1[1] + vector1[1])
        # find the arc length
        arc1 = self.findArcLength(pt1, p1, start, 'R')
        arc2 = self.findArcLength(end, p2, pt2, 'R')
        total_len = straight + arc1 + arc2
        return total_len, straight, arc1, arc2, 'RSR', [pt1, pt2]

    def rsl(self, start, end, p1, p2):
        """
        left turn then straight then left turn again
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        dist = self.distCenter(p1, p2)
        if dist < self.radius:
            return (999, 999, 999)
        straight = np.sqrt(np.square(dist) - np.square(2 * self.radius))
        delta = np.arccos((2 * self.radius) / dist)

        vector1 = (p2[0] - p1[0], p2[1] - p1[1])
        vector2 = (vector1[0] * np.cos(delta) - vector1[1] * np.sin(delta),
                   vector1[0] * np.sin(delta) + vector1[1] * np.cos(delta))
        vector3 = -vector2[0], -vector2[1]

        pt1 = p1[0] + (self.radius / dist) * vector2[0], p1[1] + (self.radius / dist) * vector2[1]
        pt2 = p2[0] + (self.radius / dist) * vector3[0], p2[1] + (self.radius / dist) * vector3[1]

        arc1 = self.findArcLength(pt1, p1, start, 'R')
        arc2 = self.findArcLength(end, p2, pt2, 'L')
        total_len = straight + arc1 + arc2
        return total_len, straight, arc1, arc2, 'RSL', [pt1, pt2]

    def lsr(self, start, end, p1, p2):
        """
        left turn then straight then left turn again
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        dist = self.distCenter(p1, p2)
        if dist < self.radius:
            return (999, 999, 999)
        straight = np.sqrt(np.square(dist) - np.square(2 * self.radius))
        delta = np.arccos((2 * self.radius) / dist)
        delta = 2*np.pi - delta
        vector1 = (p2[0] - p1[0], p2[1] - p1[1])
        vector2 = (vector1[0] * np.cos(delta) - vector1[1] * np.sin(delta),
                   vector1[0] * np.sin(delta) + vector1[1] * np.cos(delta))
        vector3 = -vector2[0], -vector2[1]

        pt1 = p1[0] + (self.radius / dist) * vector2[0], p1[1] + (self.radius / dist) * vector2[1]
        pt2 = p2[0] + (self.radius / dist) * vector3[0], p2[1] + (self.radius / dist) * vector3[1]

        arc1 = self.findArcLength(pt1, p1, start, 'L')
        arc2 = self.findArcLength(end, p2, pt2, 'R')
        total_len = straight + arc1 + arc2
        return total_len, straight, arc1, arc2, 'LSR', [pt1, pt2]



    def rlr(self, start, end, p1, p2):
        """
        right then left right
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        # dist between p1 and p2

        dist = self.distCenter(p1, p2)

        # find middle point of the p1 and p2

        q = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2

        # dist between p1 and p3 is 2r

        vector1 = p2[0] - p1[0], p2[1] - p1[1]

        # rotate clockwise for this one

        vector2 = -vector1[1], vector1[0]

        # dist between q3 to q
        dist1 = np.sqrt(np.square(2 * self.radius) - np.square(dist / 2))

        p3 = q[0] + (dist1 / dist) * vector2[0], q[1] + (dist1 / dist) * vector2[1]

        pt1 = (p3 + p1) / 2
        pt2 = (p2 + p3) / 2
        if 2 * self.radius < dist > 4 * self.radius:
            return 999, 999, 999
        arc1 = self.findArcLength(pt1, p1, start, 'R')
        arc2 = self.findArcLength(pt2, p3, pt1, 'L')
        arc3 = self.findArcLength(end, p2, pt2, 'R')
        total_len = arc3 + arc1 + arc2
        return total_len, arc1, arc2, arc3, 'RLR', [pt1, pt2, p3]

    def lrl(self, start, end, p1, p2):
        """
        right then left right
        :param start: tuple (x,y,theta) at the center of the car
        :param end: tuple (x,y,theta) at the center of the car
        :param p1: tuple (x,y) turning circle at the start
        :param p2: tuple (x,y) turning circle at the end
        :return: tuple ( straight dist, first turn distance, last turn distance )
        """

        # dist between p1 and p2

        dist = self.distCenter(p1, p2)

        # find middle point of the p1 and p2

        q = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2

        # dist between p1 and p3 is 2r

        vector1 = p2[0] - p1[0], p2[1] - p1[1]

        # rotate clockwise for this one

        vector2 = p2[1] - p1[1], p1[0] - p2[0]

        # dist between q3 to q
        dist1 = np.sqrt(np.square(2 * self.radius) - np.square(dist / 2))

        p3 = q[0] + (dist1 / dist) * vector2[0], q[1] + (dist1 / dist) * vector2[1]

        pt1 = (p3 + p1) / 2
        pt2 = (p2 + p3) / 2
        if 2 * self.radius < dist > 4 * self.radius:
            return 999, 999, 999
        arc1 = self.findArcLength(pt1, p1, start, 'L')
        arc2 = self.findArcLength(pt2, p3, pt1, 'R')
        arc3 = self.findArcLength(end, p2, pt2, 'L')
        total_len = arc3 + arc1 + arc2
        return total_len, arc1, arc2, arc3, 'LRL', [pt1, pt2, p3]

    def findCenter(self, point, side):
        """
        find the circle based on the left/right turn as required
        :param point: tuple (x,y, theta) at the center of the robot
        :param side: char L/R
        :return: tuple (x,y) the center of the turning circle
        """
        assert side in 'LR'

        angle = point[2] + (np.pi / 2 if side == 'L' else -(np.pi / 2))

        return np.array((point[0] + np.cos(angle) * self.radius,
                         point[1] + np.sin(angle) * self.radius))

    def distCenter(self, p1, p2):
        """
        find the distance between two points ( center )
        :param p1: tuple (x,y)
        :param p2: tuple (x,y)
        :return: straight line distance
        """
        return np.sqrt(np.square((p2[0] - p1[0])) + np.square((p2[1] - p1[1])))

    def findArcLength(self, pt, p, robotPos, turn):
        """

        :param pt: tuple(x,y) pt to turn to in the circle
        :param p: tuple(x,y) circle of the turning circle
        :return: arc length
        """

        assert turn in 'LR'
        pos = (robotPos[0], robotPos[1])
        # let vector 1 be the vector from p1 to p and vector2 be the vector from p1 to pt
        vector1 = (pos[0] - p[0], pos[1] - p[1])
        vector2 = (pt[0] - p[0], pt[1] - p[1])

        alpha = math.atan2(vector2[1], vector2[0]) - math.atan2(vector1[1], vector1[0])
        if alpha < 0 and turn == 'L':
            alpha = alpha + 2 * np.pi
        elif alpha > 0 and turn == 'R':
            alpha = alpha - 2 * np.pi

        return abs(alpha * self.radius)

    def generatePathCoords(self, start, end, path, interval=0.5):
        """
        Generate the coords in discrete time steps to ensure validity of the path
        :param command:
        :param interval: time in secs (default to 0.5secs)
        :return: list[tuple]
        """
        type: str = path[4]
        total = path[0]
        if type == 'RLR' or type == 'LRL':
            return self.generateCurve(start, end, path, interval)
        turns = [*type]
        center_0 = self.findCenter(start, turns[0])
        center_1 = self.findCenter(end, turns[2])
        points = []
        if path[2] > 0:
            ini = np.array(path[5][0])
        else:
            ini = np.array(start[:2])
        if path[3] > 0:
            fin = np.array(path[5][1])
        else:
            fin = np.array(end[:2])
        dist_straight = self.distCenter(ini, fin)
        current = start
        for x in np.arange(0, total, interval):
            if x < path[2]:
                current = self.calculateCoords(current, interval, turns[0])
                points.append(current)
            elif x > total - path[3]:
                current = self.calculateCoords(current, interval, turns[2])
                points.append(current)
            else:
                current = self.calculateCoords(current, interval, turns[1])
                points.append(current)
        points.append(end[:3])
        return np.array(points)

    def generateCurve(self, start, end, path, interval):
        total = path[0]
        turns = path[4]
        turns = [*turns]
        center_0 = self.findCenter(start, turns[0])
        center_2 = self.findCenter(end, turns[2])
        center_1 = path[5][2]
        points = []
        current = start
        for x in np.arange(0, total, interval):
            if x < path[1]:
                current = self.calculateCoords(current, interval, turns[0])
                points.append(current)
            elif x > total - path[3]:
                current = self.calculateCoords(current, interval, turns[2])
                points.append(current)
            else:
                current = self.calculateCoords(current, interval, turns[1])
                points.append(current)
        points.append(end[:3])
        return np.array(points)
        
        


    def circleArc(self, reference, beta, center, x):

        """
        :param reference: float
            Angular starting point, in rads
        :param beta: float
            to know the direction of the rotation
        :param center: tuple
            (x,y) center of the circle
        :param x: float
        :return: coord of point on the circle, in a tuple
        """
        angle = reference[2] + ((x / self.radius) - np.pi / 2) * np.sign(beta)
        vect = np.array([np.cos(angle), np.sin(angle)])
        return center + self.radius * vect

    def calculateCoords(self, pos, delta, type):
        assert type in 'RSL'

        if type == "S":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = pos[2]
        elif type == "R":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] - delta / self.radius)
        elif type == "L":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] + delta / self.radius)

        return np.array([new_X, new_Y, new_orientation])

    def plot(self, path):
        matplotlib.use('TkAgg')
        for c in path:
            x = np.array([x[0] for x in c])
            y = np.array([y[1] for y in c])
            plt.scatter(x, y)
        for obs in self.env.virtualObstacles:
            x, y = obs.polygon.exterior.xy
            plt.plot(x, y)
        plt.show()

    def save_path(self, path):
        matplotlib.use('TkAgg')
        for c in path:
            x = np.array([x[0] for x in c])
            y = np.array([y[1] for y in c])
            plt.scatter(x, y)
        for obs in self.env.virtualObstacles:
            x, y = obs.polygon.exterior.xy
            plt.plot(x, y)
        plt.savefig("path.png")
