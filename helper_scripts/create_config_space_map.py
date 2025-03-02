#TODO: Plese clean me up!!!
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
                                  override_all=False, blur=False, start=None, goal=None, square_width=-1, dilate=None,
                                  erode=None, robot_inflation=False):
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

    img_labels = color2label(img, num_classes)

    if blur:
        blured_labels = cv2.GaussianBlur(img_labels.astype('float64'), (23, 23), 0)
        img_labels = np.round(blured_labels)

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
    obstacle_map = np.zeros(img.shape[:2])
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if labels_obstacles[i, j] in obstacle_ids:
                obstacle_map[i, j] = 1

    if erode is not None:
        if erode >= 1:
            element_size = 2 + erode
            element = cv2.getStructuringElement(cv2.MORPH_CROSS, (element_size, element_size))
            obstacle_map = cv2.erode(obstacle_map, element)
    if dilate is not None:
        if dilate >= 1:
            element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
            obstacle_map = cv2.dilate(obstacle_map, element, iterations=dilate)

    if robot_inflation:
        m_per_pixel = 400.0/640.0
        rover_size = [3.0, 2.7]
        safety_factor = 1.25

        safety_distance = safety_factor * np.sqrt((rover_size[0]/2.0)**2 + (rover_size[1]/2.0)**2) / m_per_pixel
        safety_distance -= 0.5
        safety_distance = int(np.ceil(safety_distance))
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1 + safety_distance * 2, 1 + safety_distance * 2))
        obstacle_map = cv2.dilate(obstacle_map, element, iterations=1)

    x_coordinates = np.array([int(start[0] - int(square_width / 2)), int(start[0] + int(square_width / 2)),
                              int(goal[0] - int(square_width / 2)), int(goal[0] + int(square_width / 2))])
    y_coordinates = np.array([int(start[1] - int(square_width / 2)), int(start[1] + int(square_width / 2)),
                              int(goal[1] - int(square_width / 2)), int(goal[1] + int(square_width / 2))])

    y_coordinates = np.clip(y_coordinates, 0, img.shape[0])
    x_coordinates = np.clip(x_coordinates, 0, img.shape[1])

    cv2.rectangle(obstacle_map, (x_coordinates[0], y_coordinates[0]), (x_coordinates[1], y_coordinates[1]), 0, -1)
    cv2.rectangle(obstacle_map, (x_coordinates[2], y_coordinates[2]), (x_coordinates[3], y_coordinates[3]), 0, -1)

    obstacle_map = np.ones(obstacle_map.shape[:2]) - obstacle_map

    img_labels = img_labels * obstacle_map
    img_labels = label2color(img_labels, 15)

    cv2.imwrite(output_file, img_labels)
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
                                                arguments.dilate, arguments.erode, arguments.robot_inflation)
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
