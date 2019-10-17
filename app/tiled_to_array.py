import numpy as np
import json
import sys


def tiled_to_array(json_map_file_name):
    results = []
    with open(json_map_file_name, 'r') as json_map:
        data = json.load(json_map)
        for layer in data['layers']:
            height = layer['height']
            width = layer['width']
            data = layer['data']
            map_2d = np.resize(data, (height, width))
            results.append(map_2d)
    return np.array(results)


if __name__ == "__main__":
    maps = tiled_to_array(sys.argv[1])
    print(maps)
