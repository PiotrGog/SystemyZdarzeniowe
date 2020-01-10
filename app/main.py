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
    m_map = temporary_map.temporary_map_1_floors
    m_real_map = random_obstacles_generator.random_obstacles_generator(temporary_map.temporary_map_1_floors, 0.1,
                                                                       0.05)
    print(m_map.shape)
    m_robots = [robot_driver.RobotDriver(1, m_real_map),
                robot_driver.RobotDriver(2, m_real_map)]
    m_driver = main_driver.MainDriver(m_map, m_robots)
    for robot in m_robots:
        m_driver.plan_random_path(robot=robot, length=1000)
    m_driver.send_paths_to_robots()
    m_gui = gui.MapGui()
    m_gui2 = gui.MapGui()
    print(m_driver)
    while True:
        for robot in m_robots:
            robot.run()
            print(robot.get_position(), robot.get_status())
            m_driver.handle_robot_notify(robot=robot)
        m_gui.update(m_driver.get_map())
        m_gui.draw(draw_pause_time_ms)
        # m_gui2.update(m_real_map)
        # m_gui2.draw(draw_pause_time_ms)


if __name__ == '__main__':
    main()
    # W = consts.MapObject.WALL
    # E = consts.MapObject.EMPTY
    #
    # m_map = np.array([[[W, W, W, W, W, W, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, E, E, E, E, E, W],
    #                    [W, W, W, W, W, W, W]]])
    #
    # G = nx.Graph()
    # mmap = m_map[0]
    # for r, row in enumerate(mmap):
    #     for c, col in enumerate(row):
    #         if mmap[r, c] != W:
    #             G.add_node((r, c))
    # for (x, y), _ in G.nodes.items():
    #     if G.has_node((x + 1, y)):
    #         G.add_edge((x, y), (x + 1, y))
    #     if G.has_node((x - 1, y)):
    #         G.add_edge((x, y), (x - 1, y))
    #     if G.has_node((x, y + 1)):
    #         G.add_edge((x, y), (x, y + 1))
    #     if G.has_node((x, y - 1)):
    #         G.add_edge((x, y), (x, y - 1))
    # print(G.number_of_nodes())
    # print(G.number_of_edges())
    # nx.draw(G, with_labels=True, font_weight='bold')
    # plt.show()
