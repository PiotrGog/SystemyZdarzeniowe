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
        self._map = map
        self._status = (0, RobotStatus.STOP)

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

    def detect_obstacle(self):  # wykrycie przeszkody z mapy rzeczywistej
        floor, x, y = self._path[self._status[0]]
        if self._map[floor, x, y] == MapObject.HUMAN:
            self._notification = RobotNotification.FOUND_HUMAN
        elif self._map[floor, x, y] == MapObject.OBSTACLE:
            self._notification = RobotNotification.FOUND_OBSTACLE

    def run(self):  # przesuniecie robota po udzieleniu zgody na jazde gdy nie ma przeszkody
        self._notification = RobotNotification.WANT_RUN
        if self._status[1] == RobotStatus.RUN:
            time_rand = random.random() * 5
            time.sleep(time_rand)
            self.detect_obstacle()
            if self._notification != RobotNotification.FOUND_OBSTACLE:
                self._status = (self._status[0] + 1, RobotStatus.STOP)
                self._notification = RobotNotification.ARRIVED
