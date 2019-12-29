from src import consts
from src import main_driver
from src import robot_driver
import numpy as np


def main():
    W = consts.MapObject.WALL
    E = consts.MapObject.EMPTY
    m_map = np.array([[[W, W, W, W, W, W, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, W, W, W, W, W, W]]])
    m_real_map = np.array([[[W, W, W, W, W, W, W],
                            [W, E, E, E, E, E, W],
                            [W, E, E, E, E, E, W],
                            [W, E, E, E, E, E, W],
                            [W, E, E, E, E, E, W],
                            [W, E, E, E, E, E, W],
                            [W, E, E, E, E, E, W],
                            [W, W, W, W, W, W, W]]])
    print(m_map.shape)
    m_robots = [robot_driver.RobotDriver(1, m_real_map)]
    m_driver = main_driver.MainDriver(m_map, m_robots)
    m_driver.plan_random_path(robot=m_robots[0], length=10)
    m_driver.send_paths_to_robots()
    print(m_driver)
    while True:
        for robot in m_robots:
            robot.run()
            print(robot.get_position(), robot.get_status())
            m_driver.handle_robot_notify(robot=robot)


if __name__ == '__main__':
    main()
