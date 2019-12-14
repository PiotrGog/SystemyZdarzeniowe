from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

import numpy as np
import random
import time

global flag


def setTrack(array):  # ustawienie trasy
    global track
    track = array
    global i
    i = 0


def setStatus(state):  # ustawienie statusu
    global status
    i = status[0]
    stan = (i, state)


def run():  # przesuniecie robota po udzieleniu zgody na jazde
    global status
    timeRand = random(1, 2)
    setStatus(RobotStatus.RUN)
    time.sleep(timeRand)
    # tutaj sprawdzenie czy cos wykrylo
    setStatus(RobotStatus.STOP)
    flag = RobotNotification.ARRIVED
    status = (status[0] + 1, status[1])


def wantRun():  # ustawienie notyfikacji z prosba o przejazd
    global notification
    notification = RobotNotification.WANT_RUN


flag = RobotStatus.STOP
notification = RobotNotification.NONE
track = []
status = [0, RobotStatus.STOP]

while status[0] < len(track):
    wantRun()
    while True:
        if flag == RobotStatus.RUN:
            break
    run()


class RobotDriver(object):
    def __init__(self):
        pass

    def get_notify(self):
        pass

    def get_notify_details(self):
        pass

    def reset_notify(self):
        pass

    def get_id(self):
        pass

    def set_path(self):
        pass
