import os
import io
import cv2
import sys
import errno
import shutil
import argparse
import numpy as np
import pandas as pd
import yaml as YAML
from report_functions import *
from map_conversion_functions import color2label, label2color
import create_map_visualization
sys.path.insert(0, os.path.join(os.path.realpath(os.path.dirname(__file__)), '../data_analysis/'))
from evaluation import run_eval_planner, PRM_STAR_PLANNER_NAME, RRT_STAR_PLANNER_NAME

#EVAL_PLANNER_TYPES = [PRM_STAR_PLANNER_NAME, RRT_STAR_PLANNER_NAME]


def parse_arguments():
    parser = argparse.ArgumentParser(description='Analyses given map.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where evaluation data shall be stored')
    parser.add_argument('-m', dest='map_config_file', type=str, default=None,
                        help='map_config_file of the stored map')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('-i', dest='iterations', type=int, default=1,
                        help='number of iterations to find best cost', required=False)
    parser.add_argument('-s', dest='create_statistics_file', action='store_true',
                        help='generate the statistics file')
    parser.add_argument('-p', dest='create_path_file', action='store_true',
                        help='generate the path file')
    parser.add_argument('-pv', dest='create_path_vis_file', action='store_true',
                        help='generate the path visualization file')
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('--planner_type', dest='eval_planner_type', type=str, default=PRM_STAR_PLANNER_NAME,
                        help='type of the evaluation planner ([prm_star], rrt_star)')

    args = parser.parse_args()
    return args


def dump_data_to_files(dest, labels, label_count, relative_occurance, obstacle_ids, obstacle_pixel, obstacle_percentage,
                       colors, vis_colors):
    data = {}
    columns = ['label', 'label_count', 'relative_occurance', 'color', 'vis_color', "is_obstacle"]
    data[" "] = ["'1'", "'1'", "%", "[b g r]", "[b g r]", "True/False"]
    #data["O"] = [np.nan, obstacle_pixel, obstacle_percentage, "",  "", ""]
    for i, label in enumerate(labels):
        data[str(label)] = [int(label), label_count[label], relative_occurance[label], colors[i], vis_colors[i],
                            (label in obstacle_ids)]
    statistics_frame = pd.DataFrame.from_dict(data, orient='index', columns=columns)
    statistics_frame.to_csv(dest, index=False)


def analyse_map(map_file, results_folder, map_config_file, obstacle_ids=None, num_classes=15, override_all=False,
                create_statistics_file=False, create_path_file=False, create_path_vis_file=False, iterations=1, evaluation_planner_type=PRM_STAR_PLANNER_NAME):

    if map_config_file is None or map_config_file == "" or not os.path.isfile(map_config_file):
        obstacle_ids = [0, 1]
        start = np.array([0.5, 0.5])
        goal = np.array([39.5, 29.5])
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
                start = np.array(data["start_location"])
                goal = np.array(data["goal_location"])
        except:
            obstacle_ids = [0, 1]
            start = np.array([0.5, 0.5])
            goal = np.array([39.5, 29.5])
    try:
        img = cv2.imread(map_file)

    except:
        print("Error while reading file: " + map_file)
        return False

    map_path, map_file_name = os.path.split(map_file)
    map_name = map_file_name.split(".")[0]

    create_map_visualization.create_map_visualization(map_file, "/tmp/", num_classes, obstacle_ids, override_all=True)
    #create_map_visualization.create_map_visualization(map_file, "/tmp/", num_classes, obstacle_ids, override_all=True,
    #                                                  range_min=30, range_max=240)
    try:
        img_vis = cv2.imread("/tmp/" + map_name + "_vis.png")
    except:
        print("Error while reading file: " + "/tmp/" + map_name + "_vis.png")
        return False

    resolution_x = img.shape[1]
    resolution_y = img.shape[0]

    max_label = num_classes**3+num_classes**2-1

    labels = color2label(img, num_classes)

    unique_labels = []
    label_count = {}
    label_color = {}
    label_vis_color = {}

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            label = labels[i, j]
            if label not in unique_labels:
                unique_labels.append(label)
                label_count[label] = 0
                label_color[label] = img[i, j]
                label_vis_color[label] = img_vis[i, j]
            label_count[label] += 1

    unique_labels.sort()

    relative_occurance = label_count.copy()

    for label in relative_occurance:
        relative_occurance[label] /= float(resolution_x * resolution_y)

    obstacle_pixel = 0
    obstacle_percentage = 0.0

    for label in relative_occurance:
        if label in obstacle_ids:
            obstacle_pixel += label_count[label]
            obstacle_percentage += relative_occurance[label]

    relative_occurance = sort_dict(relative_occurance)
    label_count = sort_dict(label_count)
    label_color = sort_dict(label_color)
    label_vis_color = sort_dict(label_vis_color)

    colors = []
    vis_colors = []

    for label in label_color.keys():
        colors.append(label_color[label])
        vis_colors.append(label_vis_color[label])

    path_image = "/tmp/rrt_maps/" + map_name + ".png"
    if os.path.exists(path_image):
        os.remove(path_image)

    best_cost = np.inf
    worst_cost = 0.0
    for i in range(iterations):
        success, cost, path, map_size, start_rrt, goal_rrt = run_eval_planner(planner_type=evaluation_planner_type, map_dir=map_path, map_name=map_name,
                                                                          output_name="rrt_output.csv", output_path="/tmp/",
                                                                          start=start,
                                                                          goal=goal,
                                                                          map_config_file_path=map_config_file,
                                                                          path_folder_name="rrt_paths")
        if cost > worst_cost and success:
            worst_cost = cost

        if cost < best_cost and success:
            best_cost = cost
            generate_report(os.path.join(results_folder, map_name + "_report"), map_file, "/tmp/" + map_name + "_vis.png",
                            path_image, cost, unique_labels, label_count, relative_occurance, obstacle_ids, obstacle_pixel,
                            obstacle_percentage, colors, vis_colors)
            if create_path_file and success:
                shutil.copy(os.path.join("/tmp", "rrt_paths", map_name + "_path.csv"),
                            os.path.join(results_folder, map_name + "_path.csv"))
            if create_path_vis_file and success:
                shutil.copy(os.path.join("/tmp", "rrt_maps", map_name + ".png"),
                            os.path.join(results_folder, map_name + "_path_vis.png"))
            if create_statistics_file:
                dump_data_to_files(os.path.join(results_folder, map_name + "_statistics.csv"), unique_labels, label_count,
                                   relative_occurance, obstacle_ids, obstacle_pixel, obstacle_percentage, colors, vis_colors)
    if iterations > 1:
        print(map_name + " delta: " + str(worst_cost-best_cost))


def sort_dict(dict_in):
    return dict(sorted(dict_in.items()))


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    source = arguments.source
    results_folder = arguments.destination_folder
    evaluation_planner_type = arguments.eval_planner_type
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
    
    if evaluation_planner_type == PRM_STAR_PLANNER_NAME: 
        planner_type = PRM_STAR_PLANNER_NAME
    elif evaluation_planner_type == RRT_STAR_PLANNER_NAME:
        planner_type = RRT_STAR_PLANNER_NAME
    else: 
        print("Planner type ", evaluation_planner_type, " is unknown!")
        exit(1)

    if not os.path.isdir(results_folder):
        results_folder = os.path.curdir

    file_counter = 0
    maps.sort()

    for map_file in maps:
        if arguments.map_config_file is None or not os.path.isfile(arguments.map_config_file):
            map_path, map_file_name = os.path.split(map_file)
            path_to_check = os.path.join(map_path, "cfg", map_file_name.split(".")[0] + ".yaml")
            if os.path.isfile(path_to_check):
                map_config_file = path_to_check
            else:
                continue
        else:
            map_config_file = arguments.map_config_file

        success = analyse_map(map_file, results_folder, map_config_file, num_classes=arguments.num_classes,
                              override_all=arguments.override_all,
                              create_statistics_file=arguments.create_statistics_file,
                              create_path_file=arguments.create_path_file,
                              create_path_vis_file=arguments.create_path_vis_file, iterations=arguments.iterations, evaluation_planner_type=evaluation_planner_type)
        if success:
            file_counter += 1