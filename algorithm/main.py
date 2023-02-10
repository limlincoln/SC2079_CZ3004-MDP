from algorithm.DataPopulator import getTestObstacles
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import MOVEMENT
from algorithm.simulator import Simulator


def main():
    obs = getTestObstacles()
    sim = Simulator(StaticEnvironment((200,200), obs), obs, False)
    sim.init()
    sim.run()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print(MOVEMENT.RIGHT.value)
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
