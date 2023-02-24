import DataPopulator
from algorithm.algo.Environment import StaticEnvironment
from algorithm.constants import MOVEMENT
from algorithm.simulator import Simulator
from algorithm.RPI.client import Client

def main():
    obs = DataPopulator.getTestObstacles()
    obs1 = DataPopulator.getTestObstacles1()
    obs2 = DataPopulator.getTestObstacles2()
    sim = Simulator(StaticEnvironment((200, 200), obs), obs, False)
    sim.init()
    sim.run()

# Press the green button in the gutter to run the script.


def startRpiClient():
    client = Client("192.168.3.3", 10050)
    client.run()


if __name__ == '__main__':
   # main()
    startRpiClient()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
