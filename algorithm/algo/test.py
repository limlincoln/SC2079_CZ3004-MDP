import unittest
import settings
from algo.Dubins import Dubins
from algo.Environment import StaticEnvironment
from Astar import Astar
from Entities.Obstacle import Obstacle
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
        rrt = Astar(env)

        start = env.randomFreeSpace()
        end = env.randomFreeSpace()

        rrt.setStart(start)
        rrt.computePath(end, 200, metric='local')
        rrt.getPath(True)
        print('Static ok')


if __name__ == '__main__':
    unittest.main()
