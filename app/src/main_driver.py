from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

import numpy as np
import random


class MainDriver(object):
    def __init__(self, map, robots):
        self.map_ = map
        self.paths_ = [[] for _ in robots]
        self.robots_ = robots
        self.robots_status = [(0, RobotStatus.STOP) for _ in robots]

    def plan_paths(self):
        def _get_map_available_next_coords(coords, x_max, y_max):
            available_coords = []
            for i in range(3):
                for j in range(3):
                    x, y = coords[1] - 1 + i, coords[2] - 1 + j
                    if x_max > x >= 0 and y_max > y >= 0 and (0, x, y) != coords:
                        available_coords.append((0, x, y))
            return random.choice(available_coords)

        for path in self.paths_:
            starting_position = (0, 0, 0)
            path.append(starting_position)
            for i in range(10):
                starting_position = _get_map_available_next_coords(starting_position, 50, 50)
                path.append(starting_position)

    def send_paths_to_robots(self):
        for robot, path in zip(self.robots_, self.paths_):
            robot.set_path(path)
    