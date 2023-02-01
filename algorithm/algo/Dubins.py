import numpy as np
def ortho(pos):
    return np.array((-pos[1], pos[0]))

def dist(a,b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) **0.5
class Dubins:
    """
    Class implementing a Dubins path planner with a constant turn radius.
    Attributes:
        radius: float
        pointSeparation: float
        precision of the path but computation time of colision check will be compromised
    """
    def __init_(self, radius, pointSeparation):
        assert radius > 0 and pointSeparation > 0
        self.radius = radius
        self.pointSeparation = pointSeparation


    def computeAllPath(self, start, goal, sort=False):
        """
        computes all the possible dubin's path and return them, in the form of a list of tuples reprensenting each option.
        :param start: tuple
            starting configuration of robot in form (x,y, directions in rad)
        :param goal: tuple
            ending configurations of robot in form (x,y, directions in rad)
        :param sort: bool
            sort in decreasing order if needed

        :return:
        The shortest list of points(x,y) linking the initial and final points given as input with only turns of radius
        and straight line
        """
        startCenterLeft = self.findCenter(start, 'L')
        startCenterRight = self.findCenter(start, 'R')
        goalCenterLeft = self.findCenter(goal, 'L')
        goalCenterRight = self.findCenter(goal, 'R')
        options = [self.lsl(start, goal, startCenterLeft, goalCenterLeft),
                   self.rsr(start, goal, startCenterRight, goalCenterRight),
                   self.rsl(start, goal, startCenterRight, goalCenterLeft),
                   self.lsr(start, goal, startCenterLeft, goalCenterRight),
                   self.rlr(start, goal, startCenterRight, goalCenterRight),
                   self.lrl(start, goal, startCenterLeft, goalCenterLeft)]
        if sort:
            options.sort(key=lambda x: x[0])
        return options

    def dubinsPath(self, start,goal):
        """
        Computes all the possible Dubin's path and returns the sequence of points representing the shortest option.
        :param start: tuple
            starting configuration of the robot in the form of (x,y,direction in rads)
        :param goal: tuple
            starting configuration of the robot in the form of (x,y, direction in rads)
        :return:
            the shortest list of points (x,y) linking the initial and final points.
            in the form of a (2xn) array
        """
        options = self.computeAllPath(start, goal)
        dubinsPath, straight = min(options, key=lambda x: x[0])[1:]
        return self.generatePoints(start, goal , dubinsPath, straight)


    def generatePoints(self, start, goal, dubinsPath, straight):
        """

        :param start: tuple
            start configuration of the robot in the form of (x,y, direction in rads)
        :param goal: tuple
            end configuration of the robot in the form of (x,y, direction in rads)
        :param dubinsPath: tuple
            contains the following:
                -the angle of the first turn in the first circle, in rads
                - the angle of the last turn in the last circle, in rads
                - the angle of the turn in the middle circle, in rads, or the length of central segment if straight is true.
        :param straight: bool
            to indicate whether there is a straight segment in the path
        :return:
        the shortest list of points (x,y) linking the start and goal points
        in the form of a 2xn numpy array
        """

        if straight:
            return self.generatePointsStraight(start, goal, dubinsPath)
        return self.generatePointsCurve(start, goal, dubinsPath)

    def lsl(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        straightDistance = dist(startCenter, goalCenter)
        alpha = np.arctan2((goalCenter-startCenter)[1], (goalCenter-startCenter)[0])
        betaGoal = (goal[2]-alpha)%(2*np.pi)
        betaStart = (alpha-start[2])%(2*np.pi)
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (betaStart, betaGoal, straightDistance), True)

    def rsr(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        straightDistance = dist(startCenter, goalCenter)
        alpha = np.arctan2((goalCenter - startCenter)[1], (goalCenter - startCenter)[0])
        betaGoal = (-goal[2]+alpha)%(2*np.pi)
        betaStart = (-alpha+start[2])%(2*np.pi)
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (-betaStart, -betaGoal, straightDistance), True)

    def rsl(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """

        medianPoint = (goalCenter - startCenter)/2
        psia = np.arctan2(medianPoint[1], medianPoint[0])
        halfInterCenter = np.linalg.norm(medianPoint)
        if halfInterCenter < self.radius:
            return (float('inf'), (0,0,0), True)
        alpha = np.arccos(self.radius/halfInterCenter)
        betaStart = -(psia+alpha-start[2]-np.pi/2)%(2*np.pi)
        betaGoal = (np.pi+goal[2]-np.pi/2-alpha-psia)%(2*np.pi)
        straightDistance = 2*(halfInterCenter**2 -self.radius**2)**0.5
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (-betaStart, betaGoal, straightDistance), True)

    def lsr(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        medianPoint = (goalCenter - startCenter)/2
        psia = np.arctan2(medianPoint[1], medianPoint[0])
        halfInterCenter = np.linalg.norm(medianPoint)
        if halfInterCenter < self.radius:
            return (float('inf'), (0,0,0), True)
        alpha = np.arccos(self.radius/halfInterCenter)
        betaStart = (psia+alpha-start[2]-np.pi/2)%(2*np.pi)
        betaGoal = (.5*np.pi-goal[2]-alpha+psia)%(2*np.pi)
        straightDistance = 2*(halfInterCenter**2 - self.radius**2)**0.5
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (betaStart, -betaGoal, straightDistance), True)

    def lrl(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        distancInterCenter = dist(startCenter, goalCenter)
        interCenter = (goalCenter-startCenter) / 2
        psia = np.arctan2(interCenter[1], interCenter[0])
        if 2*self.radius < distancInterCenter > 4*self.radius:
            return (float('inf'), (0,0,0), True)
        gamma = 2*np.arcsin(distancInterCenter/(4*self.radius))
        betaStart = (psia-start[2] + np.pi/2 + (np.pi-gamma)/2)%(2*np.pi)
        betaGoal = (-psia+np.pi/2+goal[2]+(np.pi-gamma)/2)%(2*np.pi)
        totalLength = (2*np.pi-gamma+abs(betaStart)+abs(betaGoal)*self.radius)
        return (totalLength, (betaStart, betaGoal, 2*np.pi-gamma), False)

    def rlr(self, start, goal, startCenter, goalCenter):
        """

        :param start: tuple
            start configuration of the robot in the form (x,y,direction in rads)
        :param goal: tuple
            end configuration of the robot in the form (x,y, direction in rads)
        :param startCenter: tuple
            coordinates of the center of the starting turn (x,y)
        :param goalCenter: tuple
            coordinates of the center of the ending turn (x,y)
        :return:
        totalLength : float ( distance of this path)
        (betaStart, betaGoal, straightDistance): tuple
            the dubins paths with the angle of first turn, the angle of the last turn and the length of the straight segment.

        straight: bool
            to indicate whether this path contain any straight part.

        """
        distancInterCenter = dist(startCenter, goalCenter)
        interCenter = (goalCenter-startCenter) / 2
        psia = np.arctan2(interCenter[1], interCenter[0])
        if 2*self.radius < distancInterCenter > 4*self.radius:
            return (float('inf'), (0,0,0), True)
        gamma = 2*np.arcsin(distancInterCenter/(4*self.radius))
        betaStart = -((-psia+start[2] + np.pi/2) + (np.pi-gamma)/2)%(2*np.pi)
        betaGoal = -((psia+np.pi/2-goal[2]+(np.pi-gamma))/2)%(2*np.pi)
        totalLength = (2*np.pi-gamma+abs(betaStart)+abs(betaGoal)*self.radius)
        return (totalLength, (betaStart, betaGoal, 2*np.pi-gamma), False)

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
        return np.array((point[0]+np.cos(angle)*self.radius,
                         point[1] + np.sin(angle)*self.radius))
    def generatePointsStraight(self, start, goal, path):
        """

        :param start: tuple
            start configuration of the robot in form (x,y, directions in psi)
        :param goal:
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
        total = self.radius*(abs(path[1])+abs(path[0])) + path[2]
        startCenter = self.findCenter(start, 'L' if path[0] > 0 else 'R')
        goalCenter = self.findCenter(goal, 'L' if path[1] > 0 else 'R')

        #find the points where the straight segment starts

        if abs(path[0] > 0):
            angle = start[2] + (abs(path[0])-np.pi/2)*np.sign(path[0])
            ini = startCenter + self.radius*np.array([np.cos(angle), np.sin(angle)])
        else: ini = np.array(start[:2])

        # now the end
        if abs(path[1]) > 0:
            angle = goal[2]+ (-abs(path[1]) - np.pi/2)*np.sign(path[1])
            fin = startCenter + self.radius*np.array([np.cos(angle), np.sin(angle)])
        else: fin = np.array(goal[:2])
        StraightDistance = dist(ini, fin)

        # generating the points with desired precision

        points = []
        for x in np.arrange(0, total, self.pointSeparation):
            if x < abs(path[0]* self.radius):
                points.append(self.circleArc(start, path[0], startCenter, x))
            elif x > total - abs(path[1]*self.radius):
                points.append(self.circleArc(goal, path[1], goalCenter, startCenter, x-total))
            else:
                coeff = (x-abs(path[0])*self.radius)/StraightDistance
                points.append(coeff*fin + (1-coeff)*ini)
        points.append(goal[:2])
        return np.array(points)

    def generatePointsCurve(self, start, goal, path):
        """

        :param start: tuple
            Start configuration of robot in the form (x,y, directions in rads).
        :param goal:
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


        total = self.radius*(abs(path[1])+abs(path[0])+abs(path[2]))
        startCenter = self.findCenter(start, 'L'if path[0] > 0 else 'R')
        goalCenter = self.findCenter(goal, 'L' if path[1] > 0 else 'R')
        intercenter = dist(startCenter, goalCenter)
        middleCenter = (startCenter + goalCenter)/2 +\
                       np.sign(path[0])*ortho((goalCenter-startCenter)/intercenter)\
                       *(4*self.radius**2-(intercenter/2)**2)**0.5
        psi_0 = np.arctan2((middleCenter-startCenter)[1], (middleCenter-startCenter)[0])-np.pi

        points = []

        for x in np.arange(0,total, self.pointSeparation):
            if x < abs(path[0])*self.radius:
                points.append(self.circleArc(start, path[0], startCenter, x))
            elif x > total - abs(path[1])*self.radius:
                points.append(self.circleArc(goal, path[1], middleCenter, x-total))
            else:
                angle = psi_0-np.sign(path[0]*(x/self.radius-abs(path[0])))
                vector = np.array([np.cos(angle), np.sin(angle)])
                points.append(middleCenter+self.radius*vector)
        points.append(goal[:2])
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
        angle = reference[2]+((x/self.radius)-np.pi/2)*np.sign(beta)
        vector = np.array([np.cos(angle), np.sin(angle)])
        return center+self.radius*vector

