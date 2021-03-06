from src.consts import MapObject
import numpy as np

W = MapObject.WALL
E = MapObject.EMPTY
H = MapObject.HUMAN
O = MapObject.OBSTACLE
S = MapObject.STEPS
R = 1

temporary_map_3_floors = np.array(
    [[[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, W, H, H, H, H, H, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, W, H, H, H, H, H, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, W, W, W, W, W, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, S, S, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, S, S, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, S, S, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, S, S, E, E, E, W, ],
      [W, E, E, E, E, E, E, O, O, O, O, O, O, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, 0, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, 0, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, R, R, R, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, R, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, E, R, W, W, W, W, ]],
     [[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]],
     [[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]]])

temporary_map_2_floors = np.array(
    [[[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]],
     [[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, S, S, S, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]]])

temporary_map_1_floors = np.array(
    [[[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]]])

temporary_map_1_floors_small = np.array(
    [[[W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, O, O, E, E, E, E, W, ],
      [W, E, E, O, O, E, E, E, E, W, ],
      [W, E, E, O, O, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, ]]])

temporary_map_1_floors_rooms = np.array(
    [[[W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, E, E, E, E, E, E, E, E, W, E, E, E, E, E, E, E, E, E, E, E, E, E, E, W, ],
      [W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, W, ]]])
