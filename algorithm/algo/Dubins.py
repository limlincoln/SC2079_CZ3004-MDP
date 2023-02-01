import numpy as np
def ortho(pos):
    return np.array((-pos[1], pos[0]))

def dist(a,b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) **0.5
class Dubins:
    def __init_(self, radius, pointSeparation):
        assert radius > 0 and pointSeparation > 0
        self.radius = radius
        self.pointSeparation = pointSeparation

    def computeAllPath(self, start, goal, sort=False):
        startCenterLeft = self.find_center(start, 'L')
        startCenterRight = self.find_center(start, 'R')
        goalCenterLeft = self.find_center(goal, 'L')
        goalCenterRight = self.find_center(goal, 'R')

        options = []

    def dubinsPath(self, start,goal):
        pass

    def generatePoints(self, start, goal, dubinsPath, straight):
        pass

    def lsl(self, start, goal, startCenter, goalCenter):
        straightDistance = dist(startCenter, goalCenter)
        alpha = np.arctan2((goalCenter-startCenter)[1], (goalCenter-startCenter)[0])
        betaGoal = (goal[2]-alpha)%(2*np.pi)
        betaStart = (alpha-start[2])%(2*np.pi)
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (betaStart, betaGoal, straightDistance), True)

    def rsr(self, start, goal, startCenter, goalCenter):
        straightDistance = dist(startCenter, goalCenter)
        alpha = np.arctan2((goalCenter - startCenter)[1], (goalCenter - startCenter)[0])
        betaGoal = (-goal[2]+alpha)%(2*np.pi)
        betaStart = (-alpha+start[2])%(2*np.pi)
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (-betaStart, -betaGoal, straightDistance), True)

    def rsl(self, start, goal, startCenter, goalCenter):

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
        medianPoint = (goalCenter - startCenter)/2
        psia = np.arctan2(medianPoint[1], medianPoint[0])
        halfInterCenter = np.linalg.norm(medianPoint)
        if halfInterCenter < self.radius:
            return (float('inf'), (0,0,0), True)
        alpha = np.arccos(self.radius/halfInterCenter)
        betaStart = (psia+alpha-start[2]-np.pi/2)%(2*np.pi)
        betaGoal = (.5*np.pi-end[2]-alpha+psia)%(2*np.pi)
        straightDistance = 2*(halfInterCenter**2 - self.radius**2)**0.5
        totalLength = self.radius*(betaGoal+betaStart)+straightDistance
        return (totalLength, (betaStart, -betaGoal, straightDistance), True)

    def lrl(self, start, goal, startCenter, goalCenter):
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
        assert side in 'LR'
        angle = point[2] + (np.pi/2 if side == 'L' else -np.pi/2)
        return np.array((point[0]+np.cos(angle)*self.radius,
                         point[1] + np.sin(angle)*self.radius))
    def generatePointsStraight(self, start, goal, path):
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

        pass
