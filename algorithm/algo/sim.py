import settings
import numpy as np
from algo.Environment import StaticEnvironment
from RRT import RRT
from Entities.Obstacle import Obstacle
from Astar import Astar
from constants import DIRECTION

test = [Obstacle((30, 0), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
env = StaticEnvironment((200, 200), test)
print(env.isWalkable(0,0,1))
aStar = Astar(env, (0,0, DIRECTION.TOP, 'P'), env.generateTargetLocation()[0])


aStar.computePath()
print(aStar.getPath())

"""
        test = [Obstacle((40, 20), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE)),
                Obstacle((70, 70), "top", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
        env = StaticEnvironment((100, 100), test)
        
rrt = RRT(env)
start = (0, 0, np.pi/2)
end = env.generateTargetLocation()[1]
print(end)

rrt.setStart(start)
rrt.computePath(end, 200, metric='local')
path = rrt.getPath(True)
"""

