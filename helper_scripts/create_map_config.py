import os
import cv2
import sys
import copy
import errno
import argparse
import numpy as np
from map_conversion_functions import color2label, label2color
sys.path.insert(0, os.path.join(os.path.realpath(os.path.dirname(__file__)), '../data_analysis/'))
from evaluation import run_eval_planner, PRM_STAR_PLANNER_NAME, RRT_STAR_PLANNER_NAME


def parse_arguments():
    parser = argparse.ArgumentParser(description='Creates configuration file for given map.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where data shall be stored')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('-sp', dest='supress_planner_config', action='store_true',
                        help='supress generation of planner config file')
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('--oid', dest='obstacle_ids', type=int, nargs='*',
                        help='IDs to be considered obstacles')
    parser.add_argument('-c', dest='compute_optimal_path_cost', action='store_true',
                        help='compute optimal path cost')

    args = parser.parse_args()
    return args


def create_yaml_entry_string(name: str, data=None, level=0, comment=None):
    line = "  "*level
    line += name.replace(":", "") + ": "
    data_type = type(data)
    if data_type is str:
        line += "\"" + data + "\""
    elif data_type is int:
        line += str(data)
    elif data_type is float:
        line += str(data)
    elif data_type is np.float64:
        line += str(data)
    elif data_type is list:
        line += "["
        first = True
        for element in data:
            if not first:
                line += ", "
            if type(element) is str:
                line += "\"" + element + "\""
            else:
                line += str(element)
            first = False

        line += "]"

    elif data_type is np.ndarray:
        if data.ndim == 2:
            line += "\n"
            line += "\t" + create_yaml_entry_string("rows", data.shape[0]) + "\n"
            line += "\t" + create_yaml_entry_string("cols", data.shape[1]) + "\n"
            line += "\t" + "data: ["
            first = True
            for i in range(data.shape[0]):
                for j in range(data.shape[1]):
                    if not first:
                        line += ", "
                    line += str(data[i, j])
                    first = False

            line += "]"
        else:
            line += "\"ArrayWithTooManyDimensions\""
    elif data is None:
        pass
    else:
        try:
            converted_data = str(data)
            line += converted_data
        except:
            pass

    if comment is not None:
        line += "\t #" + comment
    return line


def compute_explorable_area(image, m_per_px, obstacle_ids_mav, obstacle_ids_rov, start_position=np.array([0.5, 0.5])):
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


def create_config_file(map_file, results_folder, num_classes=15, supress_planner_config_file=False, override_all=False,
                       obstacle_ids=None, compute_optimal_path_cost=False, start_px=None, goal_px=None):

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

    default_scale = 40.0 / 640

    width = resolution_x * default_scale
    height = resolution_y * default_scale

    if start_px is None: 
        start_x = 0.5
        start_y = 0.5
    else: 
        start_x = width * start_px[0] / resolution_x
        start_y = height * start_px[1] / resolution_y
    
    if goal_px is None: 
        goal_x = 0.95 * width
        goal_y = 0.95 * height

        if width > 10 and height > 10:
            goal_x = width - 0.5
            goal_y = height - 0.5
    else: 
        goal_x = width * goal_px[0] / resolution_x
        goal_y = height * goal_px[1] / resolution_y

    max_label = num_classes**3+num_classes**2-1

    labels = color2label(img, num_classes)

    unique_labels = []

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            label = labels[i, j]
            if label not in unique_labels:
                unique_labels.append(label)

    unique_labels.sort()

    unknown_id = max_label
    unknown_id_set = unknown_id not in unique_labels
    if not unknown_id_set:
        dist_max = 0
        for i in range(1, len(unique_labels)):
            new_id = int((unique_labels[i-1] + unique_labels[i]) / 2.0)
            dist = int(new_id - unique_labels[i-1])
            if dist > dist_max:
                dist_max = dist_max
                unknown_id = new_id
        if dist_max == 0:
            while not unknown_id_set:
                unknown_id = int(np.random.rand() * max_label)
                unknown_id_set = unknown_id not in unique_labels

    obstacle_ids_rov = obstacle_ids
    obstacle_ids_mav = [0]

    free_map = True
    for obstacle_id in obstacle_ids_mav + obstacle_ids_rov:
        free_map &= obstacle_id not in unique_labels

    explorable_free_space = height * width

    if not free_map:
        explorable_free_space = compute_explorable_area(labels, width/resolution_x, obstacle_ids_mav, obstacle_ids_rov,
                                                        start_position=np.array([start_x, start_y]))

    output_file = os.path.join(results_folder, map_name + ".yaml")
    if not check_override(output_file, override=override_all):
        return False

    f = open(output_file, "w")
    f.write(create_yaml_entry_string("name", map_name) + "\n")
    f.write(create_yaml_entry_string("type", "real", comment="artificial/real") + "\n")
    f.write(create_yaml_entry_string("resolution", [resolution_x, resolution_y], comment="[px] width x height") + "\n")
    f.write(create_yaml_entry_string("width", width, comment="[m]") + "\n")
    f.write(create_yaml_entry_string("height", height, comment="[m]") + "\n")
    f.write(create_yaml_entry_string("n_data_classes", num_classes) + "\n")
    f.write(create_yaml_entry_string("present_ids", unique_labels) + "\n")
    f.write(create_yaml_entry_string("obstacle_ids_mav", obstacle_ids_mav) + "\n")
    f.write(create_yaml_entry_string("obstacle_ids_rov", obstacle_ids_rov) + "\n")
    f.write(create_yaml_entry_string("unknown_id", unknown_id) + "\n")
    f.write(create_yaml_entry_string("start_location", [start_x, start_y], comment="[m]") + "\n")
    f.write(create_yaml_entry_string("goal_location", [goal_x, goal_y], comment="[m]") + "\n")
    f.write(create_yaml_entry_string("explorable_free_space", explorable_free_space, comment="[m^2]") + "\n")
    f.close()

    optimal_rrt_rov_path_cost = -1.0
    optimal_prm_rov_path_cost = -1.0
    if compute_optimal_path_cost:
        success_rrt, cost_rrt, path_rrt, map_size_rrt, start_rrt, goal_rrt = run_eval_planner(planner_type=RRT_STAR_PLANNER_NAME, map_dir=map_path, map_name=map_name,
                                                                          output_name="rrt_output.csv", output_path="/tmp/",
                                                                          map_config_file_path=output_file,
                                                                          path_folder_name="rrt_paths", start=np.array([start_x,start_y]), goal=np.array([goal_x,goal_y]))

        if success_rrt:
            optimal_rrt_rov_path_cost = cost_rrt
    
    
        success_prm, cost_prm, path_prm, map_size_prm, start_prm, goal_prm = run_eval_planner(planner_type=PRM_STAR_PLANNER_NAME, map_dir=map_path, map_name=map_name,
                                                                          output_name="rrt_output.csv", output_path="/tmp/",
                                                                          map_config_file_path=output_file,
                                                                          path_folder_name="rrt_paths", start=np.array([start_x,start_y]), goal=np.array([goal_x,goal_y]))

        if success_prm:
            optimal_prm_rov_path_cost = cost_prm

    f = open(output_file, "a")
    f.write(create_yaml_entry_string("optimal_rrt_rov_path_cost", optimal_rrt_rov_path_cost, comment="[1] Integral over map cost") + "\n")
    f.write(create_yaml_entry_string("optimal_prm_rov_path_cost", optimal_prm_rov_path_cost, comment="[1] Integral over map cost") + "\n")
    f.close()

    print("Created file: " + os.path.abspath(output_file))

    if not supress_planner_config_file:
        success = create_bounds_config_file(map_name, results_folder, width, height, override_all)

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


def create_bounds_config_file(map_name, results_folder, width, height, override_all=False):
    output_file = os.path.join(results_folder, map_name + "_bounds.yaml")
    if not check_override(output_file, override=override_all):
        return False

    f = open(output_file, "w")
    f.write("# Bounding boxes (in initialization frame, relative to PlayerStart)" + "\n")
    f.write(create_yaml_entry_string("map_bounding_volume") + "\n")
    f.write(create_yaml_entry_string("x_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("x_max", width, level=1) + "\n")
    f.write(create_yaml_entry_string("y_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("y_max", height, level=1) + "\n")
    f.write(create_yaml_entry_string("z_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("z_max", 0.01, level=1) + "\n")
    f.write(create_yaml_entry_string("target_bounding_volume") + "\n")
    f.write(create_yaml_entry_string("x_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("x_max", width, level=1) + "\n")
    f.write(create_yaml_entry_string("y_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("y_max", height, level=1) + "\n")
    f.write(create_yaml_entry_string("z_min", 0.0, level=1) + "\n")
    f.write(create_yaml_entry_string("z_max", 0.01, level=1) + "\n")
    f.close()

    print("Created file: " + os.path.abspath(output_file))
    return True


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    obstacle_ids = arguments.obstacle_ids
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

    file_counter = 0
    maps.sort()

    if not len(obstacle_ids) > 0:
        while True:
            decision = input("No obstacle ids were entered. Do you want to use '0' as default or None? [0]/N")
            decision = decision.casefold()
            if decision == " " or decision == "" or decision == "0":
                obstacle_ids = [0]
                break
            elif decision == "n" or decision == "none":
                obstacle_ids = []
                break

    for map_file in maps:
        success = create_config_file(map_file, results_folder, arguments.num_classes, arguments.supress_planner_config,
                                     arguments.override_all, obstacle_ids, arguments.compute_optimal_path_cost)
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
