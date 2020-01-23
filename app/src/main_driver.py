from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)
import logging

# from src import robot_driver

import numpy as np
import random
import networkx as nx
import numbers
import inspect


class MainDriver(object):
    def __init__(self, map, robots):
        self._map = np.copy(map)
        self._empty_map = np.copy(map)
        self._robots = robots
        self._paths = {r.get_id(): r._path.copy() for r in robots}
        self._robots_status = {r.get_id(): (0, RobotStatus.STOP) for r in robots}
        self.callbacks = {
            RobotNotification.NONE: MainDriver._robot_notify_none_callback,
            RobotNotification.ARRIVED: self._robot_notify_arrived_callback,
            RobotNotification.FOUND_HUMAN: self._robot_notify_found_human_callback,
            RobotNotification.FOUND_OBSTACLE: self._robot_notify_found_obstacle_callback,
            RobotNotification.WANT_RUN: self._robot_notify_want_run_callback,
        }

        self._robot_area = {r.get_id(): [] for r in robots}
        self._init_areas()
        self._map_graph = nx.Graph()
        self._init_graph()
        for robot in self._robots:
            self._set_map_field(robot._initial_localization, robot.get_id())

    def _init_areas(self):
        map_closed_areas, closed_areas = self._close_areas()
        sorted_closed_areas = sorted(closed_areas, key=lambda x: len(x))
        robots_amount = len(self._robots)
        # if len(sorted_closed_areas) >= robots_amount:
        for i, area in enumerate(sorted_closed_areas):
            self._robot_area[self._robots[i % robots_amount].get_id()].append(area)

    def _init_graph(self):
        available_map_objects = [MapObject.EMPTY, MapObject.STEPS, MapObject.VISITED]
        for z, floor in enumerate(self._map):
            for x, row in enumerate(floor):
                for y, col in enumerate(row):
                    if self._map[z, x, y] in available_map_objects or \
                            (isinstance(self._map[z, x, y], numbers.Number) and self._map[z, x, y] > 0):
                        self._map_graph.add_node((z, x, y))
        for (z, x, y), _ in self._map_graph.nodes.items():
            if self._map_graph.has_node((z, x + 1, y)):
                self._map_graph.add_edge((z, x, y), (z, x + 1, y))
            if self._map_graph.has_node((z, x - 1, y)):
                self._map_graph.add_edge((z, x, y), (z, x - 1, y))
            if self._map_graph.has_node((z, x, y + 1)):
                self._map_graph.add_edge((z, x, y), (z, x, y + 1))
            if self._map_graph.has_node((z, x, y - 1)):
                self._map_graph.add_edge((z, x, y), (z, x, y - 1))
            if self._map[z, x, y] == MapObject.STEPS:
                if self._map_graph.has_node((z + 1, x, y)) and self._map[z + 1, x, y] == MapObject.STEPS:
                    self._map_graph.add_edge((z, x, y), (z + 1, x, y))
                if self._map_graph.has_node((z - 1, x, y)) and self._map[z - 1, x, y] == MapObject.STEPS:
                    self._map_graph.add_edge((z, x, y), (z - 1, x, y))

    def _robot_notify_none_callback(self, robot):
        logging.info(f'_robot_notify_none_callback, robot {robot.get_id()}')
        pass

    def _clear_path(self, path):
        result_path = [path[0]]
        for p in path:
            if p != result_path[-1]:
                result_path.append(p)
        return result_path

    def _robot_notify_arrived_callback(self, robot):
        logging.info(f'_robot_notify_arrived_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        robot_step = self._get_robot_step(robot)
        if robot_step <= 0:
            return
        prev_coords = self._get_robot_coordinates(robot, -1)
        pz, px, py = prev_coords
        if type(self._get_map_field(prev_coords)) != MapObject \
                and self._empty_map[pz, px, py] == MapObject.EMPTY:
            self._set_map_field(prev_coords, MapObject.VISITED)
        else:
            self._set_map_field(prev_coords, self._empty_map[pz, px, py])
        floor, x, y = prev_coords
        floors, x_max, y_max = self._map.shape
        for x_i in range(3):
            for y_i in range(3):
                new_x = x - 1 + x_i
                new_y = y - 1 + y_i
                if 0 <= new_x < x_max and 0 <= new_y < y_max:
                    if self._map[floor, new_x, new_y] == MapObject.EMPTY:
                        self._map[floor, new_x, new_y] = MapObject.VISITED
        curr_coords = self._get_robot_coordinates(robot)
        self._set_map_field(curr_coords, robot.get_id())

    def _robot_notify_found_human_callback(self, robot):
        logging.info(f'_robot_notify_found_human_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        obstacles = robot.read_and_clear_obstacles()
        humans = robot.read_and_clear_humans()
        for coords in obstacles:
            self._set_map_field(coords, MapObject.OBSTACLE)
        for coords in humans:
            self._set_map_field(coords, MapObject.HUMAN)
        self._map_graph.remove_nodes_from(humans)
        self._map_graph.remove_nodes_from(obstacles)

    def _robot_notify_found_obstacle_callback(self, robot):
        logging.info(f'_robot_notify_found_obstacle_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        obstacles = robot.read_and_clear_obstacles()
        humans = robot.read_and_clear_humans()
        logging.info(f'obstacles {obstacles}')
        logging.info(f'humans {humans}')
        for coords in obstacles:
            self._set_map_field(coords, MapObject.OBSTACLE)
        for coords in humans:
            self._set_map_field(coords, MapObject.HUMAN)
        self._map_graph.remove_nodes_from(humans)
        self._map_graph.remove_nodes_from(obstacles)
        # z, x, y = robot.get_position()
        # self._map[z, x, y] = MapObject.OBSTACLE

    def _robot_notify_want_run_callback(self, robot):
        logging.info(f'_robot_notify_want_run_callback, robot {robot.get_id()}')
        next_coordinates = self._get_robot_coordinates(robot, 1)
        next_coordinates_state = self._get_map_field(next_coordinates)
        if type(next_coordinates_state) != MapObject:
            return
        self._set_map_field(next_coordinates, robot.get_id())
        self._set_robot_status(robot, RobotStatus.RUN)
        self._next_robot_step(robot)

    def _get_robot_status(self, robot):
        return self._robots_status[robot.get_id()][1]

    def _set_robot_status(self, robot, status):
        self._robots_status[robot.get_id()] = (self._robots_status[robot.get_id()][0], status)
        robot.set_status(status)

    def _next_robot_step(self, robot):
        step, status = self._robots_status[robot.get_id()]
        self._robots_status[robot.get_id()] = (step + 1, status)

    def _set_robot_step(self, robot, step):
        _, status = self._robots_status[robot.get_id()]
        self._robots_status[robot.get_id()] = (step, status)

    def _get_robot_step(self, robot):
        return self._robots_status[robot.get_id()][0]

    def _get_robot_coordinates(self, robot, offset_from_current=0):
        return self._paths[robot.get_id()][self._get_robot_step(robot) + offset_from_current]

    def _get_map_field(self, coordinates):
        z, x, y = coordinates
        return self._map[z, x, y]

    def _set_map_field(self, coordinates, status):
        z, x, y = coordinates
        self._map[z, x, y] = status

    @staticmethod
    def _get_map_available_next_coords(current_coords, map):
        available_coords = []
        non_visited_coords = []
        z_c, x_c, y_c = current_coords
        if MapObject.STEPS == map[z_c, x_c, y_c]:
            for z in range(z_c - 1, z_c + 2, 2):
                if 0 <= z < map.shape[0]:
                    available_coords.append((z, x_c, y_c))
                    if map[z, x_c, y_c] != MapObject.VISITED:
                        non_visited_coords.append((z, x_c, y_c))
        for x in range(x_c - 1, x_c + 2, 2):
            if 0 <= x < map.shape[1] and (map[z_c, x, y_c] == MapObject.EMPTY
                                          or map[z_c, x, y_c] == MapObject.VISITED
                                          or map[z_c, x, y_c] == MapObject.STEPS):
                available_coords.append((z_c, x, y_c))
                if map[z_c, x, y_c] != MapObject.VISITED:
                    non_visited_coords.append((z_c, x, y_c))
        for y in range(y_c - 1, y_c + 2, 2):
            if 0 <= y < map.shape[2] and (map[z_c, x_c, y] == MapObject.EMPTY
                                          or map[z_c, x_c, y] == MapObject.VISITED
                                          or map[z_c, x_c, y] == MapObject.STEPS):
                available_coords.append((z_c, x_c, y))
                if map[z_c, x_c, y] != MapObject.VISITED:
                    non_visited_coords.append((z_c, x_c, y))

        return non_visited_coords if len(non_visited_coords) > 0 else available_coords

    def get_map(self):
        return self._map

    def _graph_connection(self, start_pos, dest_pos):
        return nx.dijkstra_path(self._map_graph, start_pos, dest_pos)

    def _flood_fill(self, start_pos, robot_area):
        tmp_map = np.copy(self._map)
        z, x, y = start_pos
        tmp_map[z, x, y] = MapObject.EMPTY
        z_size, x_size, y_size = self._map.shape
        path = []

        def flood_fill_helper(start_pos, i):
            z, x, y = start_pos
            if not (0 <= x < x_size and 0 <= y < y_size) or tmp_map[z, x, y] != MapObject.EMPTY:
                return
            if not (z, x, y) in robot_area:
                return
            i = i + 1
            tmp_map[z, x, y] = i
            path.append((z, x, y))
            flood_fill_helper((z, x + 1, y), i)
            flood_fill_helper((z, x - 1, y), i)
            flood_fill_helper((z, x, y + 1), i)
            flood_fill_helper((z, x, y - 1), i)

        flood_fill_helper(start_pos, 1)
        return path, tmp_map

    def _close_areas(self):
        result = np.copy(self._map)
        result_last = None
        z_size, x_size, y_size = result.shape
        W = MapObject.WALL
        E = MapObject.EMPTY
        mask1 = np.array([[E, W, E],
                          [E, E, E]])
        mask2 = np.array([[E, E, E],
                          [E, W, E]])
        mask3 = np.array([[E, E],
                          [W, E],
                          [E, E]])
        mask4 = np.array([[E, E],
                          [E, W],
                          [E, E]])
        while not np.array_equal(result_last, result):
            result_last = np.copy(result)
            for z in range(z_size):
                for x in range(x_size - 1):
                    for y in range(y_size - 2):
                        if np.all(result[z, x:x + 2, y:y + 3] == mask1):
                            result[z, x + 1, y + 1] = MapObject.WALL
                        if np.all(result[z, x:x + 2, y:y + 3] == mask2):
                            result[z, x, y + 1] = MapObject.WALL
                for x in range(x_size - 2):
                    for y in range(y_size - 1):
                        if np.all(result[z, x:x + 3, y:y + 2] == mask3):
                            result[z, x + 1, y + 1] = MapObject.WALL
                        if np.all(result[z, x:x + 3, y:y + 2] == mask4):
                            result[z, x + 1, y] = MapObject.WALL
        empty = set([tuple(x.reshape(1, -1)[0]) for x in np.argwhere(result == MapObject.EMPTY)])
        diff_walls = set([tuple(x.reshape(1, -1)[0]) for x in np.argwhere(self._map == MapObject.EMPTY)]) - empty
        closed = []
        while len(empty) > 0:
            new_element = empty.pop()
            closed.append([])
            neighbours = [new_element]
            while len(neighbours) > 0:
                z_e, x_e, y_e = neighbours.pop(0)
                closed[-1].append((z_e, x_e, y_e))
                for x in range(x_e - 1, x_e + 2):
                    for y in range(y_e - 1, y_e + 2):
                        if (z_e, x, y) in empty:
                            neighbours.append((z_e, x, y))
                        if (z_e, x, y) in diff_walls:
                            closed[-1].append((z_e, x, y))
                        empty.discard((z_e, x, y))
                        diff_walls.discard((z_e, x, y))
        return result, closed

    def plan_path(self, **kwargs):
        robot = kwargs['robot']
        id = robot.get_id()
        robot_areas = self._robot_area[id]
        current_robot_step, current_robot_status = self._robots_status[id]

        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        if current_robot_status == RobotStatus.STOP:
            self._paths[id] = [robot_position_plan_path]  # reset path list
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            self._paths[id] = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]
        # path, _ = self._flood_fill(robot_position_plan_path)

        result_path = []
        for area_idx, area in enumerate(robot_areas):
            path_to_area = self._graph_connection(robot_position_plan_path, area[0])
            area_init_position = next(x for x in path_to_area if x in set(area))
            path_to_area = path_to_area[:path_to_area.index(area_init_position)]
            result_path = result_path + path_to_area
            robot_position_plan_path = area_init_position
            path, _ = self._flood_fill(robot_position_plan_path, area)
            for i in range(len(path) - 1):
                if abs(path[i][1] - path[i + 1][1]) + abs(path[i][2] - path[i + 1][2]) > 1:
                    connect_path = self._graph_connection(path[i], path[i + 1])
                    result_path = result_path + connect_path
                else:
                    result_path.append(path[i])
            robot_position_plan_path = result_path[-1]
        try:
            return_path = self._graph_connection(robot_position_plan_path, robot._initial_localization)
        except nx.NetworkXNoPath:
            logging.error('Cannot find return path')
            return_path = []
        result_path = result_path + return_path
        self._paths[id] = self._clear_path(result_path)
        return self._paths[id]

    def again_plan_path(self, **kwargs):
        robot = kwargs['robot']
        id = robot.get_id()
        current_robot_step, current_robot_status = self._robots_status[id]
        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        init_rp = robot_position_plan_path

        if current_robot_status == RobotStatus.STOP:
            # self._paths[id] = [robot_position_plan_path]  # reset path list
            path = [robot_position_plan_path]
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            # self._paths[id] = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]
            path = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]

        result_path = [robot_position_plan_path]

        left_steps = robot_coords_list[current_robot_step:]
        for i, (z, x, y) in enumerate(left_steps):
            if self._map[z, x, y] == MapObject.EMPTY:
                path.append((z, x, y))
                # robot_position_plan_path = (z, x, y)

        for i in range(len(path)):
            if abs(robot_position_plan_path[0] - path[i][0]) + \
                    abs(robot_position_plan_path[1] - path[i][1]) + \
                    abs(robot_position_plan_path[2] - path[i][2]) > 1:
                try:
                    connect_path = self._graph_connection(robot_position_plan_path,
                                                          path[i])
                    result_path = result_path + connect_path
                except nx.NetworkXNoPath:
                    pass
                except nx.NodeNotFound:
                    pass
            else:
                result_path.append(path[i])
            robot_position_plan_path = result_path[-1]
        try:
            return_path = self._graph_connection(robot_position_plan_path, robot._initial_localization)
        except nx.NetworkXNoPath:
            logging.error('Cannot find return path')
            return_path = []
        result_path = result_path + return_path
        self._paths[id] = self._clear_path(result_path)
        logging.info(f'{init_rp}, {self._paths[id][0]}')
        return self._paths[id]

    def again_plan_paths(self, **kwargs):
        for robot in self._robots:
            self.again_plan_path(robot=robot)

    def plan_paths(self, **kwargs):
        for robot in self._robots:
            self.plan_path(robot=robot)

    def plan_random_path(self, **kwargs):
        path_length = kwargs['length']
        robot = kwargs['robot']
        id = robot.get_id()
        current_robot_step, current_robot_status = self._robots_status[id]

        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        if current_robot_status == RobotStatus.STOP:
            self._paths[id] = [robot_position_plan_path]  # reset path list
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            self._paths[id] = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]
        for step in range(path_length):  # create path starting from current position
            available_coords = set(MainDriver._get_map_available_next_coords(robot_position_plan_path, self._map))
            robot_position_plan_path_next = random.choice(list(available_coords))
            while robot_position_plan_path_next in self._paths[id] and len(available_coords) > 1:
                available_coords.remove(robot_position_plan_path_next)
                robot_position_plan_path_next = random.choice(list(available_coords))
            robot_position_plan_path = robot_position_plan_path_next
            self._paths[id].append(robot_position_plan_path)
        return self._paths[id]

    def plan_random_paths(self):
        for robot in self._robots:
            self.plan_random_path(robot=robot, length=1000)

    def send_paths_to_robots(self):
        for robot in self._robots:
            robot.set_path(self._paths[robot.get_id()])
            robot.set_step(self._robots_status[robot.get_id()][0])

    def handle_robot_notify(self, robot):
        robot_notification = robot.get_notify()
        if robot_notification == RobotNotification.NONE:
            self._robot_notify_none_callback(robot)
        elif robot_notification == RobotNotification.WANT_RUN:
            self._robot_notify_want_run_callback(robot)
        elif robot_notification == RobotNotification.ARRIVED:
            self._robot_notify_arrived_callback(robot)
        elif robot_notification == RobotNotification.FOUND_OBSTACLE:
            self._robot_notify_found_obstacle_callback(robot)
            self.again_plan_paths()
            self.send_paths_to_robots()
        elif robot_notification == RobotNotification.FOUND_HUMAN:
            self._robot_notify_found_human_callback(robot)
            self.again_plan_paths()
            self.send_paths_to_robots()
        else:
            raise Exception("Illegal notification")
        # robot.reset_notify()
