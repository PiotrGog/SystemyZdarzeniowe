from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

from .main_driver import MainDriver

import numpy as np
import random
import time


class RobotDriver(object):
    def __init__(self, robotID, map):
        self.robotID = robotID
        self.path = []
        self.notification = RobotNotification.NONE
        self.map = map
        self.status = [0, RobotStatus.STOP]

    def get_notify(self):
        return self.notification

    def get_notify_details(self):
        return self.path[self.status[0]]

    def reset_notify(self):
        self.notification = RobotNotification.NONE

    def get_id(self):
        return self.robotID

    def set_path(self, path):
        self.path = path

    def set_status(self, state):
        self.status =[self.status[0], state]

    def detect_obstacle(self):  # wykrycie przeszkody z mapy rzeczywistej
        x, y, z = self.path[self.status[0]]
        if self.map[x, y, z] == MapObject.HUMAN:
            self.notification = RobotNotification.FOUND_HUMAN
        elif self.map[x, y, z] == MapObject.OBSTACLE:
            self.notification = RobotNotification.FOUND_OBSTACLE

    def run(self):  # przesuniecie robota po udzieleniu zgody na jazde gdy nie ma przeszkody
        self.notification = RobotNotification.WANT_RUN
        if self.status[1] == RobotStatus.RUN:
            time_rand = random(1, 2)
            time.sleep(time_rand)
            self.detect_obstacle(self)
            if self.notification != RobotNotification.FOUND_OBSTACLE:
                self.status = [self.status[0]+1, RobotStatus.STOP]
                self.notification = RobotNotification.ARRIVED
