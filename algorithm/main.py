import DataPopulator
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import MOVEMENT
from algorithm.simulator import Simulator


def main():
    obs = DataPopulator.getTestObstacles()
    obs1 = DataPopulator.getTestObstacles1()
    obs2 = DataPopulator.getTestObstacles2()
    sim = Simulator(StaticEnvironment((200,200), obs2), obs2, False)
    sim.init()
    sim.run()

# Press the green button in the gutter to run the script.


if __name__ == '__main__':
    print(MOVEMENT.RIGHT.value)
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
