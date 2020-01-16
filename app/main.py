from src import consts
from src import main_driver
from src import robot_driver
from src import gui
from src import temporary_map
from others import random_obstacles_generator
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


def main():
    draw_pause_time_ms = 1
    m_map = temporary_map.temporary_map_2_floors
    m_real_map = random_obstacles_generator.random_obstacles_generator(temporary_map.temporary_map_2_floors, 0.05,
                                                                       0.01)
    print(m_map.shape)
    m_robots = []
    m_robots.append(robot_driver.RobotDriver(1, m_real_map))
    m_robots.append(robot_driver.RobotDriver(2, m_real_map))
    m_driver = main_driver.MainDriver(m_map, m_robots)
    for robot in m_robots:
        m_driver.plan_random_path(robot=robot, length=1000)
    m_driver.send_paths_to_robots()
    m_gui = gui.MapGui()
    print(m_driver)
    while True:
        for robot in m_robots:
            robot.run()
            print(robot.get_position(), robot.get_status())
            m_driver.handle_robot_notify(robot=robot)
        m_gui.update(m_driver.get_map())
        m_gui.draw(draw_pause_time_ms)


if __name__ == '__main__':
    main()
