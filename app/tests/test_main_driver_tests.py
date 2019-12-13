import pytest
from pytest_mock import mocker
import numpy as np
from collections import Counter
from src.main_driver import MainDriver
from src.consts import MapObject, RobotNotification, RobotStatus
from .base_classes_mocks import RobotDriver


@pytest.fixture()
def map_empty():
    w = MapObject.WALL
    e = MapObject.EMPTY
    floor = [[e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e]]
    return np.array([floor])


@pytest.fixture()
def map_with_walls():
    w = MapObject.WALL
    e = MapObject.EMPTY
    floor = [[w, w, w, w, w, w, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w]]
    return np.array([floor])


@pytest.fixture()
def robots(mocker):
    robot = mocker.MagicMock()
    robot.get_id.return_value = 1
    return [robot]


def test_get_available_coordinates_in_empty_map(map_empty):
    coords = MainDriver._get_map_available_next_coords((0, 0, 0), map_empty)
    c1 = Counter(coords)
    c2 = Counter([(0, 1, 0), (0, 0, 1)])
    assert c1 == c2

    coords = MainDriver._get_map_available_next_coords((0, map_empty.shape[1] - 1, map_empty.shape[2] - 1), map_empty)
    c1 = Counter(coords)
    c2 = Counter([
        (0, map_empty.shape[1] - 2, map_empty.shape[2] - 1),
        (0, map_empty.shape[1] - 1, map_empty.shape[2] - 2)])
    assert c1 == c2


def test_get_available_coordinates_with_walls(map_with_walls):
    coords = MainDriver._get_map_available_next_coords((0, 1, 1), map_with_walls)
    c1 = Counter(coords)
    c2 = Counter([(0, 1, 2), (0, 2, 1)])
    assert c1 == c2

    coords = MainDriver._get_map_available_next_coords(
        (0, map_with_walls.shape[1] - 1, map_with_walls.shape[2] - 2), map_with_walls
    )
    c1 = Counter(coords)
    c2 = Counter([
        (0, map_with_walls.shape[1] - 2, map_with_walls.shape[2] - 2),
        (0, map_with_walls.shape[1] - 1, map_with_walls.shape[2] - 3)])
    assert c1 == c2


def test_plan_random_path(map_with_walls, robots):
    main_driver = MainDriver(map_with_walls, robots)
    new_path = main_driver.plan_random_path(robot=robots[0], length=10)
    assert 11 == len(new_path)  # current position+10 next steps


def test_handle_robot_notify_none(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.NONE
    mocker.spy(main_driver, '_robot_notify_none_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_none_callback.call_count == 1


def test_handle_robot_notify_arrived(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.ARRIVED
    mocker.spy(main_driver, '_robot_notify_arrived_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_arrived_callback.call_count == 1


def test_handle_robot_notify_want_run(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.WANT_RUN
    mocker.spy(main_driver, '_robot_notify_want_run_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_want_run_callback.call_count == 1


def test_handle_robot_notify_found_human(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.FOUND_HUMAN
    mocker.spy(main_driver, '_robot_notify_found_human_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_found_human_callback.call_count == 1


def test_handle_robot_notify_found_obstacle(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.FOUND_OBSTACLE
    mocker.spy(main_driver, '_robot_notify_found_obstacle_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_found_obstacle_callback.call_count == 1

# def test_robot_notify_want_run(map_with_walls, robots, mocker):
#     main_driver = MainDriver(map_with_walls, robots)
#     robots[0].get_notify.return_value = RobotNotification.WANT_RUN
#     # robots[0].get_status.return_value = RobotStatus.STOP
#     mocker.spy(main_driver, '_robot_notify_found_obstacle_callback')
#     main_driver.handle_robot_notify(robots[0])
#     assert main_driver._robot_notify_found_obstacle_callback.call_count == 1
