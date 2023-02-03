import sys
import pygame
import settings
from Entities.arena import Arena
from Entities.Obstacle import Obstacle
from Entities.Robot import Robot
from algo.Environment import StaticEnvironment


def main():
    global SCREEN, CLOCK
    pygame.init()
    SCREEN = pygame.display.set_mode((settings.WINDOW_WIDTH, settings.WINDOW_HEIGHT))
    CLOCK = pygame.time.Clock()
    SCREEN.fill(settings.BLACK)
    test = [
            Obstacle((70,20), "right", (2*settings.BLOCK_SIZE,2*settings.BLOCK_SIZE))]
    arena = Arena(test, 400+settings.GRID_OFFSET, 400+settings.GRID_OFFSET, settings.BLOCK_SIZE)
    robot = Robot(arena.obList)
    arena.drawGrid(SCREEN)
    robot.drawCar(SCREEN)
    env = StaticEnvironment((200,200), test)
    arena.drawStuff(env.generateTargetLocation(), SCREEN, settings.GREEN)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    robot.moveStraight(10)
                elif event.key == pygame.K_d:
                    robot.turnRight(10)
                elif event.key == pygame.K_a:
                    robot.turnLeft(10)
                arena.updateGrid(robot, SCREEN)
        pygame.display.update()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
