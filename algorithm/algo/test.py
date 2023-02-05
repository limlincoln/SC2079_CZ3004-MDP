import unittest
import algorithm.settings as settings
from algorithm.constants import DIRECTION
from algorithm.algo.Dubins import Dubins
from algorithm.algo.Environment import StaticEnvironment
from RRT import RRT
from algorithm.Entities.Obstacle import Obstacle
from Astar import Astar
from algorithm.DataPopulator import getTestObstacles
from algorithm.simulator import Simulator

class SimSum(unittest.TestCase):

    def testDubins(self):
        """
        rrt dubins class
        :return:
        """
        Dubins(10,1)
        print('Dubins ok')

    def testEnvironment(self):
        """
        Test the environment class initialisation
        :return:
        """

        test = [Obstacle((40, 20), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE)),
                Obstacle((70, 70), "top", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
        env = StaticEnvironment((200,200), test)
        print("Static Environment ok")

    def testRRT(self):
        """
        Tests the RRT class
        :return:
        """
        test = [Obstacle((40, 20), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE)),
                Obstacle((70, 70), "top", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
        env = StaticEnvironment((100, 100), test)
        rrt = RRT(env)

        start = env.randomFreeSpace()
        end = env.generateTargetLocationInRads()[1]

        rrt.setStart(start)
        rrt.computePath(end, 200, metric='local')
        print(rrt.getPath(True))
        print('Static ok')
    def testAstar(self):
        """
        Tests the Astar
        :return:
        """

        test = getTestObstacles()
        env = StaticEnvironment((200, 200), test)
        for image in env.generateTargetLocation():
            aStar = Astar(env, (0,0, DIRECTION.TOP, 'P'), image)
            aStar.computePath()
            path = aStar.getPath()
            if path:
                print("Path okay")
            else:
                print("not okay", image)

        print("Astar OK")
    def testCollision(self):

        test = getTestObstacles()
        env = StaticEnvironment((200,200), test)
        testCases = [(10,70)]
        print(env.isWalkable(testCases[0][0],testCases[0][1]))

    def testSimulator(self):
        obs = getTestObstacles()
        sim = Simulator(StaticEnvironment((200, 200), obs), obs, True)
        sim.init()
        sim.run()




if __name__ == '__main__':
    unittest.main()
