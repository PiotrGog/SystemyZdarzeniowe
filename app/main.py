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


def dylatacja(img):
    result = np.copy(img)
    x_size, y_size = img.shape
    for x in range(x_size):
        for y in range(y_size):
            if np.any(img[x - 1:x + 2, y - 1:y + 2] == consts.MapObject.WALL):
                result[x, y] = consts.MapObject.WALL
    return result


def erozja(img):
    result = np.copy(img)
    x_size, y_size = img.shape
    for x in range(x_size):
        for y in range(y_size):
            if np.any(img[x - 1:x + 2, y - 1:y + 2] == consts.MapObject.EMPTY):
                result[x, y] = consts.MapObject.EMPTY
    return result


def close_rooms(map):
    result = np.copy(map)
    result_last = None
    z_size, x_size, y_size = result.shape

    while not np.array_equal(result_last, result):
        result_last = np.copy(result)
        for z in range(z_size):
            for x in range(x_size):
                for y in range(y_size):
                    if x + 1 < x_size and np.count_nonzero(result[z, x - 1:x + 1, y] == consts.MapObject.WALL) == 2:
                        result[z, x + 1, y] = consts.MapObject.WALL
                    if x - 1 >= 0 and np.count_nonzero(result[z, x:x + 2, y] == consts.MapObject.WALL) == 2:
                        result[z, x - 1, y] = consts.MapObject.WALL
                    if y + 1 < y_size and np.count_nonzero(result[z, x, y - 1:y + 1] == consts.MapObject.WALL) == 2:
                        result[z, x, y + 1] = consts.MapObject.WALL
                    if y - 1 >= 0 and np.count_nonzero(result[z, x, y:y + 2] == consts.MapObject.WALL) == 2:
                        result[z, x, y - 1] = consts.MapObject.WALL
    empty = set([tuple(x.reshape(1, -1)[0]) for x in np.argwhere(result == consts.MapObject.EMPTY)])
    closed = []
    while len(empty) > 0:
        new_element = empty.pop()
        closed.append([])
        neighbours = [new_element]
        while len(neighbours) > 0:
            z_e, x_e, y_e = neighbours.pop(0)
            closed[-1].append((z_e, x_e, y_e))
            for z in range(z_e - 1, z_e + 2):
                for x in range(x_e - 1, x_e + 2):
                    for y in range(y_e - 1, y_e + 2):
                        if (z, x, y) in empty:
                            neighbours.append((z, x, y))
                        empty.discard((z, x, y))
    return result, closed


def flood_fill(map, start_pos):
    tmp_map = np.copy(map)
    z_size, x_size, y_size = map.shape
    path = []

    def flood_fill_helper(start_pos, i):
        x, y = start_pos
        if not (0 <= x < x_size and 0 <= y < y_size) or tmp_map[0, x, y] != 0:
            return
        i = i + 1
        tmp_map[0, x, y] = i
        path.append((x, y))
        flood_fill_helper((x + 1, y), i)
        flood_fill_helper((x - 1, y), i)
        flood_fill_helper((x, y + 1), i)
        flood_fill_helper((x, y - 1), i)

    flood_fill_helper(start_pos, 1)
    return path, tmp_map


def graph_demo():
    W = consts.MapObject.WALL
    E = consts.MapObject.EMPTY
    S = consts.MapObject.STEPS

    m_map = np.array([[[W, W, W, W, W, W, W],
                       [W, S, S, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, W, W, W, W, W, W]],
                      [[W, W, W, W, W, W, W],
                       [W, S, S, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, E, E, E, E, E, W],
                       [W, W, W, W, W, W, W]]
                      ])

    G = nx.Graph()
    mmap = m_map[0]
    for r, row in enumerate(mmap):
        for c, col in enumerate(row):
            if mmap[r, c] == E:
                G.add_node((r, c))
    for (x, y), _ in G.nodes.items():
        if G.has_node((x + 1, y)):
            G.add_edge((x, y), (x + 1, y))
        if G.has_node((x - 1, y)):
            G.add_edge((x, y), (x - 1, y))
        if G.has_node((x, y + 1)):
            G.add_edge((x, y), (x, y + 1))
        if G.has_node((x, y - 1)):
            G.add_edge((x, y), (x, y - 1))
    print(G.number_of_nodes())
    print(G.number_of_edges())
    nx.draw(G, with_labels=True, font_weight='bold')
    plt.show()


def find_path(map, start, dest):
    pass


if __name__ == '__main__':
    graph_demo()
'''
    space = np.zeros((1, 10, 10))
    path, tmp = flood_fill(space, start_pos=(5, 5))
    print(path)

    plt.imshow(tmp[0])
    plt.show()
'''
# main()
# m_map = temporary_map.temporary_map_1_floors_rooms
# m_gui = gui.MapGui()
# m_gui.update(m_map)
# m_gui.draw(1000)
# m_map_c, closed = close_rooms(m_map)
# for i, r in enumerate(closed):
#     for c in r:
#         m_map[c[0], c[1], c[2]] = i + 1
# m_gui.update(m_map)
# m_gui.draw(1000)
