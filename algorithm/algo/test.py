import unittest
import settings
from constants import DIRECTION
from algo.Dubins import Dubins
from algo.Environment import StaticEnvironment
from RRT import RRT
from Entities.Obstacle import Obstacle
from Astar import Astar

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
        end = env.randomFreeSpace()

        rrt.setStart(start)
        rrt.computePath(end, 200, metric='local')
        rrt.getPath(True)
        print('Static ok')
    def testAstar(self):
        """
        Tests the Astar
        :return:
        """

        test = [Obstacle((40, 20), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE)),
                Obstacle((70, 70), "top", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
        env = StaticEnvironment((100, 100), test)
        aStar = Astar(env, (0,0, DIRECTION.TOP, 'P'), env.generateTargetLocation()[0])

        print("Astar OK")


if __name__ == '__main__':
    unittest.main()
