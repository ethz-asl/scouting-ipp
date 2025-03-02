import os
import cv2
import copy
import errno
import random
import argparse
import numpy as np
from map_conversion_functions import color2label, label2color

def parse_arguments():
    parser = argparse.ArgumentParser(description='Extracts obstacles from maps and overlays them on other maps.')
    parser.add_argument('-s', dest='source', type=str,
                        help='obstacle file or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where maps shall be stored')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('--ids', dest='obstacle_ids', nargs="*", type=int,
                        help='IDs to be considered obstacles', required=False)
    parser.add_argument('-b', dest='blur', action="store_true", default=False,
                        help='apply blurring before adding the obstacles', required=False)
    parser.add_argument('--start', dest='start', nargs=2, type=int, default=[8, 8],
                        help='start pixel', required=False)
    parser.add_argument('--goal', dest='goal', nargs=2, type=int, default=[632, 472],
                        help='goal pixel', required=False)
    parser.add_argument('-w', dest='square_width', type=int, default=-1,
                        help='width of the area arround start and goal to keep free of obstacles, if not defined nothing is kept free',
                        required=False)
    parser.add_argument('-d', dest='dilate', type=int, default=None,
                        help='dilate obstacles n times',
                        required=False)
    parser.add_argument('-e', dest='erode', type=int, default=None,
                        help='erode obstacles n times',
                        required=False)
    parser.add_argument('-ir', dest='robot_inflation', action='store_true',
                        help='inflate obstacles by the half size of the Perseverance rover', required=False)
    parser.add_argument('map', type=str, help='input map file')

    args = parser.parse_args()
    return args


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


def extract_obstacles_and_overlay(obstacle_file, map_file, results_folder, obstacle_ids=None, num_classes=15,
                                  override_all=False, blur=False,
                                  start=None, goal=None, square_width=-1, dilate=None, erode=None):
    if obstacle_ids is None:
        obstacle_ids = [0, 1]
    try:
        obstacle = cv2.imread(obstacle_file)

    except:
        print("Error while reading file: " + obstacle_file)
        return False

    if square_width <= 0:
        square_width = 0
        start = None
        goal = None

    if start is None:
        start = [-1, -1]
    if goal is None:
        goal = [-1, -1]
    if start[0] <= 0 or start[1] <= 0:
        start = [-square_width - 1, -square_width - 1]
    if goal[0] <= 0 or goal[1] <= 0:
        goal = [-square_width - 1, -square_width - 1]

    obstacle_path, obstacle_file_name = os.path.split(obstacle_file)
    obstacle_name = obstacle_file_name.split(".")[0]

    try:
        img = cv2.imread(map_file)
    except:
        print("Error while reading file: " + map_file)
        return False

    if blur:
        img = cv2.GaussianBlur(img, (23, 23), 0)

    map_path, map_file_name = os.path.split(map_file)
    map_name = map_file_name.split(".")[0]

    output_file = os.path.join(results_folder, (map_name + "_" + obstacle_name + ".png"))

    if not check_override(output_file, override=override_all):
        return False

    if img.shape != obstacle.shape:
        answer_correct = False
        while not answer_correct:
            reshape = input("Map file and obstacle file do not have the same size. Shall they be reshaped [Y]/n: ")
            reshape = reshape.casefold().replace(" ", "")
            if reshape == "y" or reshape == "":
                answer_correct = True
                cv2.resize(obstacle, img.shape, obstacle, interpolation=cv2.INTER_LINEAR)
            elif reshape == "n":
                answer_correct = True
                return False
            else:
                answer_correct = False

    labels_obstacles = color2label(obstacle, num_classes)

    replace_label = None
    while replace_label is None:
        x = random.randint(0, img.shape[0]-1)
        y = random.randint(0, img.shape[1]-1)
        if labels_obstacles[x, y] not in obstacle_ids:
            replace_label = labels_obstacles[x, y]

    eroded_labels_obstacles = copy.deepcopy(labels_obstacles)
    if erode is not None:
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if labels_obstacles[i, j] in obstacle_ids:
                    is_all_obstacle = True
                    for k in range(-erode, erode):
                        for l in range(-erode, erode):
                            if 0 <= i + k < img.shape[0] and 0 <= j + l < img.shape[1]:
                                if labels_obstacles[i + k, j + l] not in obstacle_ids:
                                    is_all_obstacle = False
                            if not is_all_obstacle:
                                break
                        if not is_all_obstacle:
                            break
                    if not is_all_obstacle:
                        eroded_labels_obstacles[i, j] = replace_label

    dilated_labels_obstacles = replace_label * np.ones(img.shape[:2])
    if dilate is not None:
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if eroded_labels_obstacles[i, j] in obstacle_ids:
                    for k in range(-dilate, dilate):
                        for l in range(-dilate, dilate):
                            if 0 <= i + k < img.shape[0] and 0 <= j + l < img.shape[1]:
                                dilated_labels_obstacles[i + k, j + l] = eroded_labels_obstacles[i, j]
                                obstacle[i + k, j + l] = obstacle[i, j]
    else:
        dilated_labels_obstacles = copy.deepcopy(eroded_labels_obstacles)

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if not ((start[0] - square_width / 2 <= j <= start[0] + square_width / 2 and
                     start[1] - square_width / 2 <= i <= start[1] + square_width / 2) or (
                            goal[0] - square_width / 2 <= j <= goal[0] + square_width / 2 and
                            goal[1] - square_width / 2 <= i <= goal[1] + square_width / 2)):

                if dilated_labels_obstacles[i, j] in obstacle_ids:
                    img[i, j] = obstacle[i, j]

    cv2.imwrite(output_file, img)
    return True


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    source = arguments.source
    results_folder = arguments.destination_folder
    obstacle_files = []

    try:
        if os.path.isdir(source):
            files = os.listdir(source)
            for file in files:
                path_to_test = os.path.join(source, file)
                if os.path.isfile(path_to_test) and ".png" in file:
                    obstacle_files.append(path_to_test)
            if not obstacle_files:
                print("No .png files found in the specified source folder: " + source)
                raise ImportError
        elif os.path.isfile(source) and ".png" in source:
            obstacle_files.append(source)
        else:
            print("The specified source can not be found or used: " + source)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during folder setup.")
        exit(1)

    try:
        if not os.path.isfile(arguments.map) or ".png" not in arguments.map:
            print("No suitable .png file found: " + arguments.map)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during map read in.")
        exit(1)

    if not os.path.isdir(results_folder):
        results_folder = os.path.curdir

    file_counter = 0
    obstacle_files.sort()

    for obstacle_file in obstacle_files:
        success = extract_obstacles_and_overlay(obstacle_file, arguments.map, results_folder, arguments.obstacle_ids,
                                                arguments.num_classes, arguments.override_all, arguments.blur,
                                                arguments.start, arguments.goal, arguments.square_width,
                                                arguments.dilate, arguments.erode)
        if success:
            file_counter += 1

    final_line = "*" * 3 + " Created " + str(file_counter) + " new map files, after finding " + str(
        len(obstacle_files)) + " obstacle files. " + "*" * 3
    second_final_line = "*" * 3 + " Process finished!"
    second_final_line += " " * (len(final_line) - (len(second_final_line) + 3)) + "*" * 3
    star_line = "*" * len(final_line)

    print("\n")
    print(star_line)
    print(second_final_line)
    print(final_line)
    print(star_line)
