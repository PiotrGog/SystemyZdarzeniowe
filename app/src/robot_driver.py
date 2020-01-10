from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

import numpy as np
import random
import time


class RobotDriver(object):
    def __init__(self, robot_id, map):
        self._robot_id = robot_id
        self._path = []
        self._notification = RobotNotification.NONE
        self._map = np.copy(map)
        self._status = (0, RobotStatus.STOP)
        self._obstacles = []
        self._humans = []
        self._movement_iterations = 0

    def get_notify(self):
        return self._notification

    def reset_notify(self):
        self._notification = RobotNotification.NONE

    def _set_notify(self, notification):
        self._notification = notification

    def get_position(self):
        return self._path[self._status[0]]

    def _set_position(self):
        return self._path[self._status[0]]

    def get_status(self):
        return self._status

    def set_status(self, state):
        self._status = (self._status[0], state)

    def get_id(self):
        return self._robot_id

    def set_path(self, path):
        self._path = path

    def _arrived_event(self):
        self._notification = RobotNotification.ARRIVED

    def _want_run_event(self):
        self._notification = RobotNotification.WANT_RUN

    def _found_human_event(self):
        self._notification = RobotNotification.FOUND_HUMAN

    def _found_obstacle_event(self):
        self._notification = RobotNotification.FOUND_OBSTACLE

    def _none_event(self):
        self._notification = RobotNotification.NONE

    def detect_obstacle(self):  # wykrycie przeszkody z mapy rzeczywistej
        floor, x, y = self._path[self._status[0]]
        for x_i in range(3):
            for y_i in range(3):
                new_x = x - 1 + x_i
                new_y = y - 1 + y_i
                if self._map[floor, new_x, new_y] == MapObject.HUMAN:
                    self._notification = RobotNotification.FOUND_HUMAN
                    self._humans.append((floor, new_x, new_y))
                elif self._map[floor, new_x, new_y] == MapObject.OBSTACLE:
                    self._notification = RobotNotification.FOUND_OBSTACLE
                    self._obstacles.append((floor, new_x, new_y))
                else:
                    self._notification = RobotNotification.NONE
        print(self._notification)

    def run(self):  # przesuniecie robota po udzieleniu zgody na jazde gdy nie ma przeszkody
        robot_state = self.get_status()[1]
        if robot_state == RobotStatus.RUN:  # while still not arrived
            if self._notification == RobotNotification.WANT_RUN:
                self._status = (self._status[0] + 1, RobotStatus.RUN)
                self._notification = RobotNotification.NONE
            if self._movement_iterations > 0:
                self._movement_iterations -= 1  # decrease iterations steps
            elif self._movement_iterations <= 0:
                self._notification = RobotNotification.ARRIVED  # if arrived notify main driver
                self._status = (self._status[0], RobotStatus.STOP)

        elif robot_state == RobotStatus.STOP:
            self.detect_obstacle()
            if self._notification == RobotNotification.NONE:
                self._notification = RobotNotification.WANT_RUN
                self._movement_iterations = random.randint(5, 10)

        # if self._status[0] == len(self._path) - 1:
        #     self._notification = RobotNotification.ARRIVED
        #     return
        # self._notification = RobotNotification.WANT_RUN
        # if self._status[1] == RobotStatus.RUN:
        #     # time_rand = random.random() * 5
        #     # time.sleep(time_rand)
        #     self.detect_obstacle()
        #     if self._notification != RobotNotification.FOUND_OBSTACLE:
        #         self._status = (self._status[0] + 1, RobotStatus.STOP)
        #         self._notification = RobotNotification.ARRIVED
