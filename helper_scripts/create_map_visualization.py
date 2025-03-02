import io
import os
import cv2
import sys
import copy
import errno
import argparse
import numpy as np
import yaml as YAML
from visualization_dirty import draw_path_in_map
from map_conversion_functions import label2color, color2label
sys.path.insert(0, os.path.join(os.path.realpath(os.path.dirname(__file__)), '../data_analysis/'))
from evaluation import run_eval_planner, PRM_STAR_PLANNER_NAME, RRT_STAR_PLANNER_NAME


def parse_arguments():
    parser = argparse.ArgumentParser(description='Creates visualization file for given map.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where data shall be stored')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('-u', dest='unknown_id', type=int, default=None,
                        help='label of unknown area')
    parser.add_argument('--oid', dest='obstacle_ids', type=int, nargs='*',
                        help='IDs to be considered obstacles')
    parser.add_argument('-i', dest='ignore_range_file', action='store_true',
                        help='ignore range file (defined and default)')
    parser.add_argument('-d', dest='draw_optimal_path', action='store_true',
                        help='compute the optimal path and draw it in the image (start^and goal are taken from the map_config file)')
    parser.add_argument('--config', dest='map_config', type=int, default=None,
                        help='path of the map config file')
    parser.add_argument('--range', dest='range_file', type=str, default="",
                        help='range file containing the ranges used for visualization', required=False)

    args = parser.parse_args()
    return args


def compute_explorable_area(image, m_per_px, obstacle_ids_mav, obstacle_ids_rov, start_position=np.array([0.0, 0.0])):
    start_location = np.array([int(start_position[0]/m_per_px), int(start_position[1]/m_per_px)])
    location_queue = [start_location]
    exploration_map = copy.deepcopy(image)
    exploration_map *= 0
    exploration_map[start_location[1], start_location[0]] = 1
    while len(location_queue):
        position = location_queue.pop()
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if 0 <= position[1] + i < image.shape[0] and 0 <= position[0] + j < image.shape[1]:
                    data_point = image[position[1] + i, position[0] + j]
                    #TODO: Handle propperly the explorable freespace for ROV and MAV
                    if data_point not in obstacle_ids_mav and exploration_map[position[1] + i, position[0] + j] == 0:
                        exploration_map[position[1] + i, position[0] + j] = 1
                        location_queue.append(np.array([position[0] + j, position[1] + i]))

    exp_pixels = np.sum(exploration_map)
    return float(exp_pixels * m_per_px * m_per_px)

def convert_path(path, map_size_px, map_size_m, dedup=False): 
    res = map_size_px / map_size_m
    converted_path = [[int(point[0] * res[0]), int(point[1] * res[1])] for point in path]
    if not dedup: 
        return converted_path
    
    dedup_path = [converted_path[0]]

    for point in converted_path: 
        if (point != dedup_path[-1]).any(): 
            dedup_path.append(point)
    return dedup_path
    


def create_map_visualization(map_file, results_folder, num_classes=15, obstacle_ids=None,
                             unknown_id=3599, override_all=False, range_min=None, range_max=None, start_px=None, goal_px=None, draw_path= False, map_config=None):
    if obstacle_ids is None:
        obstacle_ids = []

    try:
        img = cv2.imread(map_file)

    except:
        print("Error while reading file: " + map_file)
        return False

    map_path, map_file_name = os.path.split(map_file)
    map_name = map_file_name.split(".")[0]

    resolution_x = img.shape[1]
    resolution_y = img.shape[0]

    max_label = num_classes**3+num_classes**2-1

    labels = color2label(img, num_classes)

    unique_labels = []

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            label = labels[i, j]
            if label not in unique_labels:
                unique_labels.append(label)

    unique_labels.sort()
    free_labels = copy.deepcopy(unique_labels)
    if unknown_id in free_labels:
        free_labels.remove(unknown_id)
    for ob_id in obstacle_ids:
        if ob_id in free_labels:
            free_labels.remove(ob_id)

    output_file = os.path.join(results_folder, map_name + "_vis.png")
    if not check_override(output_file, override=override_all):
        return False

    if range_min is None:
        range_min = free_labels[0]
    if range_max is None:
        range_max = free_labels[-1]

    vis_img = img
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            current_label = labels[i, j]
            if current_label == unknown_id:
                vis_img[i, j] = (127, 127, 127)
            elif current_label in obstacle_ids:
                vis_img[i, j] = (0, 0, 0)
            else:
                if range_max != range_min:
                    s = (current_label - range_min) / (range_max - range_min)
                    if 0.5 < s <= 1.0:
                        s = (s-0.5) * 2
                        color = (s * np.array([0, 0, 255]) + (1 - s) * np.array([0, 255, 255])).astype(int)
                    elif 0.0 <= s <= 0.5:
                        s = s * 2
                        color = (s * np.array([0, 255, 255]) + (1 - s) * np.array([0, 255, 0])).astype(int)
                    else:
                        color = np.array([255, 0, 0]).astype(int)

                else:
                    color = np.array([0, 255, 255]).astype(int)
                vis_img[i, j] = color
    
    if draw_path: 
        if map_config is None: 
            map_config = os.path.join(map_path, "cfg", map_name + ".yaml") 
            if not os.path.exists(map_config): 
                print("No map config has been provided and the default has not been found: ", map_config)
                return False
        success, cost, path, map_size, start_rrt, goal_rrt = run_eval_planner(planner_type=PRM_STAR_PLANNER_NAME, map_dir=map_path, map_name=map_name,
                                                                        output_name="rrt_output.csv", output_path="/tmp/",
                                                                        map_config_file_path=map_config,
                                                                        path_folder_name="rrt_paths")
        if success: 
            converted_path = convert_path(path, [resolution_x, resolution_y], map_size)
            draw_path_in_map(vis_img, converted_path, color=(255, 0, 0), line_thickness=3)
        

        if start_px is None:
            start_px = start_rrt * [resolution_x, resolution_y] / map_size
            start_px = [int(start_px[0]), int(start_px[1])]
        if goal_px is None: 
            goal_px = goal_rrt * [resolution_x, resolution_y] / map_size
            goal_px = [int(goal_px[0]), int(goal_px[1])]
        else:
            print("The planner was not able to find a valid path, so the drawing is skipped!")

    if not start_px is None: 
        cv2.drawMarker(img, (start_px[0], start_px[1]), color=[255, 255, 255], thickness=3, markerType= cv2.MARKER_DIAMOND, line_type=cv2.LINE_AA, markerSize=18)

    if not goal_px is None: 
        cv2.drawMarker(img, (goal_px[0], goal_px[1]), color=[255, 255, 255], thickness=3, markerType= cv2.MARKER_TILTED_CROSS, line_type=cv2.LINE_AA, markerSize=18)


    cv2.imwrite(output_file, vis_img)

    print("Created file: " + os.path.abspath(output_file))

    return True


def check_override(file, override=False):
    if override:
        return True
    if os.path.isfile(file):
        success = False
        print("Existing output file: " + file)
        while not success:
            input_str = input("Override y/[N]: ")
            input_str = input_str.casefold().replace(" ", "")
            if input_str == "n" or input_str == "":
                return False
            elif input_str == "y":
                success = True
            else:
                success = False
    return True

def read_yaml_config_file(path):
    result = {}
    if path is not None:
        if path != "":
            try:
                if os.path.isfile(path) and ".yaml" in path:
                    with open(path, "r") as stream:
                        output = io.StringIO()
                        for line in stream:
                            if '\t' in line:
                                line = line.replace('\t', '')
                            output.write(line)
                        result = YAML.safe_load(output.getvalue())
                else:
                    print("The specified config file can not be found: " + path)
                    raise FileNotFoundError
            except FileNotFoundError:
                result = {}
            except:
                result = {}
        else:
            result = {}
    else:
        result = {}
    return result


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    source = arguments.source
    results_folder = arguments.destination_folder
    maps = []

    try:
        if os.path.isdir(source):
            files = os.listdir(source)
            for file in files:
                path_to_test = os.path.join(source, file)
                if os.path.isfile(path_to_test) and ".png" in file:
                    maps.append(path_to_test)
            if not maps:
                print("No .png files found in the specified source folder: " + source)
                raise ImportError
        elif os.path.isfile(source) and ".png" in source:
            maps.append(source)
        else:
            print("The specified source can not be found or used: " + source)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during folder setup.")
        exit(1)

    if not os.path.isdir(results_folder):
        results_folder = os.path.curdir

    if arguments.unknown_id is None:
        unknown_id = 0
        set_uid = False
        while not set_uid:
            default_uid = arguments.num_classes**3 + arguments.num_classes**2 - 1
            input_uid = input("Use default unknown id " + str(default_uid) + " [Y]/n: ")
            input_uid = input_uid.casefold().replace(" ", "")
            if input_uid == "" or input_uid == "y":
                unknown_id = default_uid
                set_uid = True
            elif unknown_id == "n":
                while not set_uid:
                    input_uid = input("Use no unknown id [Y]/n: ")
                    input_uid = input_uid.casefold().replace(" ", "")
                    if input_uid == "" or input_uid == "y":
                        unknown_id = -1
                        set_uid = True
                    elif unknown_id == "n":
                        while not set_uid:
                            input_uid = input("What unknown id to use? ")
                            input_uid = input_uid.casefold().replace(" ", "")
                            if input_uid.isnumeric():
                                unknown_id = int(input_uid)
                                set_uid = True
                            else:
                                print("Invalid input!!")
                    else:
                        print("Invalid input!!")
            else:
                print("Invalid input!!")
    else:
        unknown_id = int(arguments.unknown_id)

    ignore_range_file = arguments.ignore_range_file
    range_min = None
    range_max = None
    if not ignore_range_file:
        range_file_path = arguments.range_file
        range_file_path_valid = True
        if range_file_path == "" or not os.path.isfile(range_file_path):
            if os.path.isfile(os.path.join(results_folder, "range.yaml")):
                range_file_path = os.path.join(results_folder, "range.yaml")
            elif os.path.isfile("/home/root/data/maps/visualization/range.yaml"):
                range_file_path = "/home/root/data/maps/visualization/range.yaml"
            else:
                range_file_path_valid = False
        try:
            if range_file_path_valid:
                range_file = read_yaml_config_file(range_file_path)
                range_min = range_file["min"]
                range_max = range_file["max"]
        except:
            range_min = None
            range_max = None

    file_counter = 0
    maps.sort()

    for map_file in maps:
        success = create_map_visualization(map_file, results_folder, arguments.num_classes, arguments.obstacle_ids,
                                           unknown_id, arguments.override_all, range_min, range_max, map_config=arguments.map_config, draw_path=arguments.draw_optimal_path)
        if success:
            file_counter += 1

    final_line = "*" * 3 + " Created " + str(file_counter) + " config files, after finding " + str(
        len(maps)) + " maps. " + "*" * 3
    second_final_line = "*" * 3 + " Process finished!"
    second_final_line += " " * (len(final_line) - (len(second_final_line) + 3)) + "*" * 3
    star_line = "*" * len(final_line)

    print("\n")
    print(star_line)
    print(second_final_line)
    print(final_line)
    print(star_line)
