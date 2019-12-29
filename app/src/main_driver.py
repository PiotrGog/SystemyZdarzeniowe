from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

# from src import robot_driver

import numpy as np
import random


class MainDriver(object):
    def __init__(self, map, robots):
        self._map = map
        self._robots = robots
        self._paths = {r.get_id(): [] for r in robots}
        self._robots_status = {r.get_id(): (-1, RobotStatus.STOP) for r in robots}

        self.callbacks = {
            RobotNotification.NONE: MainDriver._robot_notify_none_callback,
            RobotNotification.ARRIVED: self._robot_notify_arrived_callback,
            RobotNotification.FOUND_HUMAN: self._robot_notify_found_human_callback,
            RobotNotification.FOUND_OBSTACLE: self._robot_notify_found_obstacle_callback,
            RobotNotification.WANT_RUN: self._robot_notify_want_run_callback,
        }

    def _robot_notify_none_callback(self, robot):
        pass

    def _robot_notify_arrived_callback(self, robot):
        self._set_robot_status(robot, RobotStatus.STOP)
        robot_step = self._get_robot_step(robot)
        if robot_step <= 0:
            return
        prev_coords = self._get_robot_coordinates(robot, -1)
        self._set_map_field(prev_coords, MapObject.VISITED)
        curr_coords = self._get_robot_coordinates(robot)
        self._set_map_field(curr_coords, robot.get_id())

    def _robot_notify_found_human_callback(self, robot):
        self._set_map_field(robot.get_position(), MapObject.HUMAN)
        # z, x, y = robot.get_position()
        # self._map[z, x, y] = MapObject.HUMAN

    def _robot_notify_found_obstacle_callback(self, robot):
        self._set_map_field(robot.get_position(), MapObject.OBSTACLE)
        # z, x, y = robot.get_position()
        # self._map[z, x, y] = MapObject.OBSTACLE

    def _robot_notify_want_run_callback(self, robot):
        next_coordinates = self._get_robot_coordinates(robot, 1)
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
        z_c, x_c, y_c = current_coords
        if MapObject.STEPS == map[z_c, x_c, y_c]:
            for z in range(z_c - 1, z_c + 2, 2):
                if 0 <= z < map.shape[0]:
                    available_coords.append((z, x_c, y_c))
        for x in range(x_c - 1, x_c + 2, 2):
            if 0 <= x < map.shape[1] and not map[z_c, x, y_c] == MapObject.WALL:
                available_coords.append((z_c, x, y_c))
        for y in range(y_c - 1, y_c + 2, 2):
            if 0 <= y < map.shape[2] and not map[z_c, x_c, y] == MapObject.WALL:
                available_coords.append((z_c, x_c, y))
        return available_coords

    def plan_random_path(self, **kwargs):
        path_length = kwargs['length']
        robot = kwargs['robot']
        id = robot.get_id()
        current_robot_step, _ = self._robots_status[id]
        if -1 == current_robot_step:  # if no path was planned before
            empty_spot = random.choice(
                [tuple(e) for e in np.argwhere(self._map != MapObject.WALL)])  # find empty spot (no walls)
            self._paths[robot.get_id()] = [empty_spot]  # create robot path list with that position
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter

        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[self._robots_status[id][0]]  # get current robot position
        self._paths[id] = [robot_position_plan_path]  # reset path list
        self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        for step in range(path_length):  # create path starting from current position
            available_coords = MainDriver._get_map_available_next_coords(robot_position_plan_path, self._map)
            self._paths[id].append(random.choice(available_coords))
        return self._paths[id]

    def plan_random_paths(self):
        pass

    def send_paths_to_robots(self):
        for robot in self._robots:
            robot.set_path(self._paths[robot.get_id()])

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
        elif robot_notification == RobotNotification.FOUND_HUMAN:
            self._robot_notify_found_human_callback(robot)
        else:
            raise Exception("Illegal notification")
        robot.reset_notify()
