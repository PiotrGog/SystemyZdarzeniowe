from random import random
from src.consts import MapObject
import src.gui as gui
import numpy as np
import src.temporary_map as tmp_map


def random_obstacles_generator(empty_map, obstacle_prob, human_prob):
    result_map = np.zeros_like(empty_map)
    for f, floor in enumerate(empty_map):
        for r, row in enumerate(floor):
            for c, col in enumerate(row):
                prob = random()
                if empty_map[f, r, c] == MapObject.EMPTY and \
                        (prob < obstacle_prob or prob < human_prob):
                    if obstacle_prob < human_prob:
                        if prob < obstacle_prob:
                            result_map[f, r, c] = MapObject.OBSTACLE
                        elif prob < human_prob:
                            result_map[f, r, c] = MapObject.HUMAN
                    else:
                        if prob < human_prob:
                            result_map[f, r, c] = MapObject.HUMAN
                        elif prob < obstacle_prob:
                            result_map[f, r, c] = MapObject.OBSTACLE
                else:
                    result_map[f, r, c] = empty_map[f, r, c]
    return result_map


if __name__ == '__main__':
    m_gui = gui.MapGui()
    m_gui.update(tmp_map.temporary_map_1_floors)
    m_gui.draw(1000)
    m_gui.update(random_obstacles_generator(tmp_map.temporary_map_1_floors, 0.10, 0.05))
    m_gui.draw(1000)
