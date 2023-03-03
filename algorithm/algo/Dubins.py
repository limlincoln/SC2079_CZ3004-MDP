import numpy as np


def ortho(pos):
    return np.array((-pos[1], pos[0]))


def dist(a,b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5


class Dubins:
    """
    Class implementing a Dubins path planner with a constant turn radius.
    Attributes:
        radius: float
        pointSeparation: float
        precision of the path but computation time of colision check will be compromised
    """
    def __init__(self, radius, pointSeparation):
        assert radius > 0 and pointSeparation > 0
        self.radius = radius
        self.pointSeparation = pointSeparation

    def computeAllPath(self, start, end, sort=False):

        """
        computes all the possible dubin's path and return them, in the form of a list of tuples reprensenting each option.
        :param start: tuple
            starting configuration of robot in form (x,y, directions in rad)
        :param end: tuple

            ending configurations of robot in form (x,y, directions in rad)
        :param sort: bool
            sort in decreasing order if needed

        :return:
        The shortest list of points(x,y) linking the initial and final points given as input with only turns of radius
        and straight line
        """
        center_0_left = self.findCenter(start, 'L')
        center_0_right = self.findCenter(start, 'R')
        center_2_left = self.findCenter(end, 'L')
        center_2_right = self.findCenter(end, 'R')
        options = [self.lsl(start, end, center_0_left, center_2_left),
                   self.rsr(start, end, center_0_right, center_2_right),
                   self.rsl(start, end, center_0_right, center_2_left),
                   self.lsr(start, end, center_0_left, center_2_right),
                   self.rlr(start, end, center_0_right, center_2_right),
                   self.lrl(start, end, center_0_left, center_2_left)]
        if sort:
            options.sort(key=lambda x: x[0])
        return options


    def dubinsPath(self, start, end):
        """
        Computes all the possible Dubin's path and returns the sequence of points representing the shortest option.
        :param start: tuple
            starting configuration of the robot in the form of (x,y,direction in rads)

        :param end: tuple
            starting configuration of the robot in the form of (x,y, direction in rads)
        :return:
            the shortest list of points (x,y) linking the initial and final points.
            in the form of a (2xn) array
        """
        options = self.computeAllPath(start, end)
        dubinsPath, straight = min(options, key=lambda x: x[0])[1:]
        return self.generatePoints(start, end, dubinsPath, straight)
    def generatePoints(self, start, end, dubinsPath, straight):

        """

        :param start: tuple
            start configuration of the robot in the form of (x,y, direction in rads)
        :param end: tuple

            end configuration of the robot in the form of (x,y, direction in rads)
        :param dubinsPath: tuple
            contains the following:
                -the angle of the first turn in the first circle, in rads
                - the angle of the last turn in the last circle, in rads
                - the angle of the turn in the middle circle, in rads, or the length of central segment if straight is true.
        :param straight: bool
            to indicate whether there is a straight segment in the path
        :return:
        the shortest list of points (x,y) linking the start and end points
        in the form of a 2xn numpy array
        """

        if straight:
            return self.generatePointsStraight(start, end, dubinsPath)
        return self.generatePointsCurve(start, end, dubinsPath)

    def lsl(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        straight_dist = dist(center_0, center_2)
        alpha = np.arctan2((center_2-center_0)[1], (center_2-center_0)[0])
        beta_2 = (end[2]-alpha)%(2*np.pi)
        beta_0 = (alpha-start[2])%(2*np.pi)
        total_len = self.radius*(beta_2+beta_0)+straight_dist
        return (total_len, (beta_0, beta_2, straight_dist), True)

    def rsr(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)

        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.
        straight: bool
            to indicate whether this path contain any straight part.

        """
        straight_dist = dist(center_0, center_2)
        alpha = np.arctan2((center_2-center_0)[1], (center_2-center_0)[0])
        beta_2 = (-end[2]+alpha)%(2*np.pi)
        beta_0 = (-alpha+start[2])%(2*np.pi)
        total_len = self.radius*(beta_2+beta_0)+straight_dist
        return (total_len, (-beta_0, -beta_2, straight_dist), True)

    def rsl(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """

        median_point = (center_2 - center_0)/2
        psia = np.arctan2(median_point[1], median_point[0])
        half_intercenter = np.linalg.norm(median_point)
        if half_intercenter < self.radius:
            return (float('inf'), (0, 0, 0), True)
        alpha = np.arccos(self.radius/half_intercenter)
        beta_0 = -(psia+alpha-start[2]-np.pi/2)%(2*np.pi)
        beta_2 = (np.pi+end[2]-np.pi/2-alpha-psia)%(2*np.pi)
        straight_dist = 2*(half_intercenter**2-self.radius**2)**.5
        total_len = self.radius*(beta_2+beta_0)+straight_dist
        return (total_len, (-beta_0, beta_2, straight_dist), True)

    def lsr(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        median_point = (center_2 - center_0)/2
        psia = np.arctan2(median_point[1], median_point[0])
        half_intercenter = np.linalg.norm(median_point)
        if half_intercenter < self.radius:
            return (float('inf'), (0, 0, 0), True)
        alpha = np.arccos(self.radius/half_intercenter)
        beta_0 = (psia-alpha-start[2]+np.pi/2)%(2*np.pi)
        beta_2 = (.5*np.pi-end[2]-alpha+psia)%(2*np.pi)
        straight_dist = 2*(half_intercenter**2-self.radius**2)**.5
        total_len = self.radius*(beta_2+beta_0)+straight_dist
        return (total_len, (beta_0, -beta_2, straight_dist), True)

    def lrl(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        dist_intercenter = dist(center_0, center_2)
        intercenter = (center_2 - center_0)/2
        psia = np.arctan2(intercenter[1], intercenter[0])
        if 2*self.radius < dist_intercenter > 4*self.radius:
            return (float('inf'), (0, 0, 0), False)
        gamma = 2*np.arcsin(dist_intercenter/(4*self.radius))
        beta_0 = (psia-start[2]+np.pi/2+(np.pi-gamma)/2)%(2*np.pi)
        beta_1 = (-psia+np.pi/2+end[2]+(np.pi-gamma)/2)%(2*np.pi)
        total_len = (2*np.pi-gamma+abs(beta_0)+abs(beta_1))*self.radius
        return (total_len,
                (beta_0, beta_1, 2*np.pi-gamma),
                False)

    def rlr(self, start, end, center_0, center_2):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param end: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param center_0: tuple
            coordinates of the center of the starting turn (x,y)
        :param center_2: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaend, straightDistance): tuple

            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.
        straight: bool
            to indicate whether this path contain any straight part.

        """
        dist_intercenter = dist(center_0, center_2)
        intercenter = (center_2 - center_0)/2
        psia = np.arctan2(intercenter[1], intercenter[0])
        if 2*self.radius < dist_intercenter > 4*self.radius:
            return (float('inf'), (0, 0, 0), False)
        gamma = 2*np.arcsin(dist_intercenter/(4*self.radius))
        beta_0 = -((-psia+(start[2]+np.pi/2)+(np.pi-gamma)/2)%(2*np.pi))
        beta_1 = -((psia+np.pi/2-end[2]+(np.pi-gamma)/2)%(2*np.pi))
        total_len = (2*np.pi-gamma+abs(beta_0)+abs(beta_1))*self.radius
        return (total_len,
                (beta_0, beta_1, 2*np.pi-gamma),
                False)


    def findCenter(self, point, side):
        """

        :param point: tuple
        -initial point in the form (x,y, direction in rads)
        :param side: Char
        - Either 'L' or 'R' indicating left/right turn respectively
        :return:
        coordinates in a 2x1 array of the center of the circle describing the turn
        """
        assert side in 'LR'
        angle = point[2] + (np.pi/2 if side == 'L' else -np.pi/2)
        return np.array((point[0] + np.cos(angle)*self.radius,
                         point[1] + np.sin(angle)*self.radius))
    def generatePointsStraight(self, start, end, path):

        """

        :param start: tuple
            start configuration of the robot in form (x,y, directions in psi)
        :param end:

            end configuration of the robot in form (x,y, directions in psi)
        :param path: tuple
            The computed dubins path, a tuple containing:
                - The angle of the turn in the first circle, in rads
                - The angle of the turn in the last circle, in rads
                - the length of the straight line in between
                negative = right turn, positive = left turn
        :return:
            The shortest list of points (x,y) linking the intitial and final points
        """
        total = self.radius * (abs(path[1]) + abs(path[0])) + path[2]  # Path length
        center_0 = self.findCenter(start, 'L' if path[0] > 0 else 'R')
        center_2 = self.findCenter(end, 'L' if path[1] > 0 else 'R')

        # We first need to find the points where the straight segment starts
        if abs(path[0]) > 0:
            angle = start[2] + (abs(path[0]) - np.pi / 2) * np.sign(path[0])
            ini = center_0 + self.radius * np.array([np.cos(angle), np.sin(angle)])
        else:
            ini = np.array(start[:2])
        # We then identify its end
        if abs(path[1]) > 0:
            angle = end[2] + (-abs(path[1]) - np.pi / 2) * np.sign(path[1])
            fin = center_2 + self.radius * np.array([np.cos(angle), np.sin(angle)])
        else:
            fin = np.array(end[:2])
        dist_straight = dist(ini, fin)

        # We can now generate all the points with the desired precision
        points = []
        for x in np.arange(0, total, self.pointSeparation):
            if x < abs(path[0]) * self.radius:  # First turn
                points.append(self.circleArc(start, path[0], center_0, x))
            elif x > total - abs(path[1]) * self.radius:  # Last turn
                points.append(self.circleArc(end, path[1], center_2, x - total))
            else:  # Straight segment
                coeff = (x - abs(path[0]) * self.radius) / dist_straight
                points.append(coeff * fin + (1 - coeff) * ini)
        points.append(end[:2])
        return np.array(points)

    def generatePointsCurve(self, start, end, path):
        """

        :param start: tuple
            Start configuration of robot in the form (x,y, directions in rads).
        :param end:

            End configuration of robot in the form (x,y, directions in rads).
        :param path: tuple
            The computed dubins path consists of the following:
                - the angle of the turn in the first circle, in rads
                - the angle of the turn in the last circle, in rads
                - the angle of the turn in the middle circle, in rads
            (negative means right turn, positive angle means left turn)
        :return:
            The shortest list of points (x,y) linking the initial and final points
        """

        total = self.radius * (abs(path[1]) + abs(path[0]) + abs(path[2]))
        center_0 = self.findCenter(start, 'L' if path[0] > 0 else 'R')
        center_2 = self.findCenter(end, 'L' if path[1] > 0 else 'R')
        intercenter = dist(center_0, center_2)
        center_1 = (center_0 + center_2) / 2 + \
                   np.sign(path[0]) * ortho((center_2 - center_0) / intercenter) \
                   * (4 * self.radius ** 2 - (intercenter / 2) ** 2) ** .5
        psi_0 = np.arctan2((center_1 - center_0)[1],
                           (center_1 - center_0)[0]) - np.pi

        points = []
        for x in np.arange(0, total, self.pointSeparation):
            if x < abs(path[0]) * self.radius:  # First turn
                points.append(self.circleArc(start, path[0], center_0, x))
            elif x > total - abs(path[1]) * self.radius:  # Last turn
                points.append(self.circleArc(end, path[1], center_2, x - total))
            else:  # Middle Turn
                angle = psi_0 - np.sign(path[0]) * (x / self.radius - abs(path[0]))
                vect = np.array([np.cos(angle), np.sin(angle)])
                points.append(center_1 + self.radius * vect)
        points.append(end[:2])
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


