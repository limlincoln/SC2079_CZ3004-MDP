import settings
import numpy as np
from algo.Environment import StaticEnvironment
from Astar import Astar
from Entities.Obstacle import Obstacle


test = [Obstacle((40, 20), "right", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE)),
        Obstacle((100, 100), "top", (2 * settings.BLOCK_SIZE, 2 * settings.BLOCK_SIZE))]
env = StaticEnvironment((200, 200), test)
rrt = Astar(env)

start = (0,0, np.pi/2)
end = env.generateTargetLocation()[1]
print(end)

rrt.setStart(start)
rrt.computePath(end, 200, metric='local')
path = rrt.getPath(True)

print(path)
