import numpy as np
import json
import sys
import pickle


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
    maps = None
    if "load" == sys.argv[2]:
        with open(sys.argv[1], 'rb') as f:
            maps = pickle.load(f)
    else:
        maps = tiled_to_array(sys.argv[1])
        if "print" == sys.argv[2]:
            print(maps)
        if "text" == sys.argv[2]:
            with open("map.bin", 'w') as f:
                pickle.dump(maps, f)
        if "bin" == sys.argv[2]:
            with open("map.bin", 'wb') as f:
                pickle.dump(maps, f)

    print(maps.shape)
