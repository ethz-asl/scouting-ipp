import os
import io
import cv2
import sys
import errno
import argparse
import numpy as np
import pandas as pd
import create_map_visualization
from map_conversion_functions import color2label, label2color
import yaml as YAML


def parse_arguments():
    parser = argparse.ArgumentParser(description='Computes performance metrics from given baseline data files.')
    parser.add_argument('data', type=str,
                        help='input file (csv) or folder')
    parser.add_argument('-c', dest='map_config_file', type=str, default=None,
                        help='map_config_file of the stored map')
    parser.add_argument('-m', dest='map', type=str, default=None,
                        help='Path to the map file')
    parser.add_argument('-d', dest='destination', type=str, default=None,
                        help='Destination where to store results')
    parser.add_argument('-r', '--run_comparison', dest='run_comparison', action='store_true',
                        help='Do a comparison in the known evaluation structure.')

    args = parser.parse_args()
    return args


def analyse_data(data_files, map_file, map_config_file, destination):
    if map_config_file is None or map_config_file == "" or not os.path.isfile(map_config_file):
        obstacle_ids = [0, 1]
        width = 40.0
        height = 30.0
        num_classes = 15
    else:
        try:
            with open(map_config_file, "r") as stream:
                output = io.StringIO()
                for line in stream:
                    if '\t' in line:
                        line = line.replace('\t', '')
                    output.write(line)
                data = YAML.safe_load(output.getvalue())
                obstacle_ids = data["obstacle_ids_rov"]
                width = data["width"]
                height = data["height"]
                num_classes = data["n_data_classes"]
                optimal_rov_path_cost = data["optimal_rov_path_cost"]
        except:
            obstacle_ids = [0, 1]
            width = 40.0
            height = 30.0
            num_classes = 15
            optimal_rov_path_cost = None
    try:
        img = cv2.imread(map_file)
    except:
        print("Error while reading file: " + map_file)
        return False

    map_path, map_file_name = os.path.split(arguments.map)
    map_name = map_file_name.split(".")[0]

    labels = color2label(img, num_classes)

    m_per_pixel = width / img.shape[1]

    def position_to_location(position):
        location = np.floor(position/m_per_pixel)
        return location

    def get_cost(position):
        location = position_to_location(position)
        return int(labels[int(location[1]), int(location[0])])


    time_costs = []
    distance_costs = []
    integral_costs = []

    for data_file in data_files:
        data = pd.read_csv(data_file)
        timestamps = data["Time"].to_numpy(float)
        position_x = data["PositionX"].to_numpy(float)
        position_y = data["PositionY"].to_numpy(float)

        timestamps -= timestamps[0]
        timestamps /= 1.0*1000000000

        integral_cost = 0
        time_cost = timestamps[-1]
        distance_cost = 0

        last_time = timestamps[0]
        last_position = np.array([position_x[0], position_y[0]])

        for i in range(1, len(timestamps)):
            position = np.array([position_x[i], position_y[i]])
            distance = np.linalg.norm(position-last_position)
            distance_cost += distance

            num_sub_samples = int(np.floor(distance / (m_per_pixel/3.0)))
            direction = (position-last_position) / distance

            end_position = last_position
            for j in range(1, num_sub_samples+1):
                start_position = end_position
                end_position = direction * min((distance/(m_per_pixel/3.0)) * j, distance) + last_position
                step_distance = np.linalg.norm(end_position-start_position)
                average_cost = (get_cost(end_position) + get_cost(start_position)) / 2.0
                integral_cost += average_cost * step_distance

            last_position = position

        time_costs.append(time_cost)
        distance_costs.append(distance_cost)
        integral_costs.append(integral_cost)

    if len(data_files) > 1:
        time_costs = np.array(time_costs)
        distance_costs = np.array(distance_costs)
        integral_costs = np.array(integral_costs)

        time_cost = np.nanmean(time_costs)
        distance_cost = np.nanmean(distance_costs)
        integral_cost = np.nanmean(integral_costs)

    else:
        time_cost = time_costs[0]
        distance_cost = distance_costs[0]
        integral_cost = integral_costs[0]

    print("Time cost:      ", time_cost)
    print("Distance cost:  ", distance_cost)
    print("Integral cost:  ", integral_cost)
    if optimal_rov_path_cost is not None:
        print("Relative cost:  ", integral_cost / optimal_rov_path_cost)
    print("Average weight: ", integral_cost/distance_cost)

    if destination is not None:
        with open(os.path.join(destination, "path_analysis.txt"), 'w') as f:
            f.write("Time cost:      " + str(time_cost) + "\n")
            f.write("Distance cost:  " + str(distance_cost) + "\n")
            f.write("Integral cost:  " + str(integral_cost) + "\n")
            if optimal_rov_path_cost is not None:
                f.write("Relative cost:  " + str(integral_cost / optimal_rov_path_cost) + "\n")
            f.write("Average weight: " + str(integral_cost / distance_cost) + "\n")


if __name__ == '__main__':

    arguments = parse_arguments()

    try:
        if not os.path.isfile(arguments.map) or ".png" not in arguments.map:
            print("The specified source can not be found or used: " + arguments.map)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during file read in.")
        exit(1)

    map_path, map_file_name = os.path.split(arguments.map)
    map_name = map_file_name.split(".")[0]

    if arguments.map_config_file is None:
        map_config_file = ""
    else:
        map_config_file = arguments.map_config_file

    try:
        if os.path.isfile(map_config_file) and ".yaml" in map_config_file:
            map_config_file = arguments.map_config_file
        else:
            map_config_file = os.path.join(map_path, "cfg", map_name + ".yaml")
            if os.path.isfile(map_config_file):
                pass
            else:
                print("No suitable .yaml file found: " + arguments.map_config_file)
                raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during map config read in.")
        exit(1)

    data_files = []
    if os.path.isfile(arguments.data) and ".csv" in arguments.data:
        data_files.append(arguments.data)
    elif os.path.isdir(arguments.data):
        content = os.listdir(arguments.data)
        for element in content:
            element_path = os.path.join(arguments.data, element)
            if arguments.run_comparison:
                splitted_element = element.split("_")
                if len(splitted_element) >= 2:
                    if splitted_element[0] == "run" and splitted_element[1].isnumeric():
                        file_path = os.path.join(element_path, "trajectory_file_mav_simulator.csv")
                        if os.path.isfile(file_path):
                            data_files.append(file_path)

            else:
                if os.path.isfile(element_path) and ".csv" in element_path:
                    data_files.append(element_path)

    if not data_files:
        print("Unknown error occurred during data read in.")
        exit(1)

    if arguments.destination is not None:
        if os.path.isdir(arguments.destination):
            destination = arguments.destination
        else:
            destination = None
    else:
        destination = None
        if arguments.run_comparison:
            eval_dir = os.path.join(arguments.data, "evaluation")
            if os.path.isdir(eval_dir):
                destination = eval_dir

    analyse_data(data_files, arguments.map, map_config_file, destination)
