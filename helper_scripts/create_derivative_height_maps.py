import os

os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"
import cv2
import errno
import argparse
import numpy as np
from map_conversion_functions import color2label


def parse_arguments():
    parser = argparse.ArgumentParser(description='Creates derivative height maps with elevated obstacles based on a '
                                                 'base height map and cost maps.')
    parser.add_argument('height_map', type=str,
                        help='input file (.exr)')
    parser.add_argument('--maps', dest="maps", nargs="*", type=str,
                        help='map files (.png)')
    parser.add_argument('--maps-folder', dest='maps_folder', type=str, required=False,
                        help='optional base folder for maps files')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where new maps shall be stored')
    parser.add_argument('--offset', dest='obstacle_offset', type=float, default=1.0,
                        help='height by which obstacles are offset')
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('-s', dest='store', action='store_true',
                        help='store results')
    parser.add_argument('--ids', dest='obstacle_ids', nargs="*", type=int,
                        help='IDs to be considered obstacles', required=False)
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)

    args = parser.parse_args()
    return args


def create_derivative_height_map(base_height_map_path, cost_map_path, obstacle_offset, output_folder, store=False,
                                 override_all = False, obstacle_ids=None, num_classes=15):
    if obstacle_ids is None:
        obstacle_ids = [0, 1]

    input_file_name = cost_map_path.split("/")[-1].replace(".png", "")

    output_file_name = "height_" + input_file_name
    if "map_" in input_file_name:
        output_file_name = input_file_name.replace("map_", "height_")

    output_file = os.path.join(output_folder, output_file_name + ".exr")
    print("Output file will be written to: ", output_file)

    if override_all == False and os.path.isfile(output_file):
        print("The file ", output_file, "already exists. To override it please rerun with te override all flag.")
        return

    cost_map_img = cv2.imread(cost_map_path, cv2.IMREAD_UNCHANGED)
    cost_map = color2label(cost_map_img, num_classes)
    width_cost_map = cost_map.shape[1]
    height_cost_map = cost_map.shape[0]

    base_height_map = cv2.imread(base_height_map_path, cv2.IMREAD_ANYDEPTH)
    width_base_height_map = base_height_map.shape[1]
    height_base_height_map = base_height_map.shape[0]

    if width_base_height_map != width_cost_map or height_base_height_map != height_cost_map:
        print("The dimension of the cost map (", width_cost_map, "x", height_cost_map, ") and the base height map (",
              width_base_height_map, "x", height_base_height_map, ") do not match!")
        exit(1)

    width = width_base_height_map
    height = height_base_height_map

    height_map = base_height_map.copy()

    for i in range(width):
        for j in range(height):
            if cost_map[j, i] in obstacle_ids:
                height_map[j, i] += obstacle_offset

    if store:
            cv2.imwrite(output_file, height_map)
    else:
        min_val = np.min(base_height_map)
        max_val = np.max(base_height_map)

        height_map -= min_val
        height_map /= max_val-min_val
        cv2.imshow(output_file_name, height_map)
        cv2.waitKey(0)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    height_map = arguments.height_map
    try:
        if os.path.isfile(height_map) and ".exr" in height_map:
            pass
        else:
            print("The specified height map can not be found or used: " + height_map)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during base height map checks.")
        exit(1)

    results_folder = arguments.destination_folder
    if not os.path.isdir(results_folder):
        print("The specified height map can not be found or used: " + height_map)
        results_folder = "/tmp/"

    obstacle_maps = []
    for source in arguments.maps:
        map_file_path = source
        if arguments.maps_folder is not None:
            map_file_path = os.path.join(arguments.maps_folder, source)

        if ".png" not in map_file_path:
            map_file_path += ".png"

        if os.path.isfile(map_file_path):
            obstacle_maps.append(map_file_path)
        else:
            print("The specified map file can not be found or used: " + map_file_path)

    if len(obstacle_maps) == 0:
        print("No map files left to process!")
        exit(1)

    obstacle_maps.sort()

    for obstacle_map in obstacle_maps:
        create_derivative_height_map(height_map, obstacle_map, arguments.obstacle_offset, results_folder,
                                     store=arguments.store, obstacle_ids=arguments.obstacle_ids,
                                     num_classes=arguments.num_classes)
