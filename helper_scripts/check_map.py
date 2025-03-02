import os
import io
import cv2
import sys
import errno
import argparse
import numpy as np
from report_functions import *
import create_map_visualization
from map_conversion_functions import color2label, label2color
sys.path.insert(0, '../data_analysis/')
from evaluation import run_rrt_star
import yaml as YAML


def parse_arguments():
    parser = argparse.ArgumentParser(description='Allows inspection of map at locations and map statistics.')
    parser.add_argument('map', type=str,
                        help='input file (map) or folder')
    parser.add_argument('-m', dest='map_config_file', type=str, default=None,
                        help='map_config_file of the stored map')

    args = parser.parse_args()
    return args


def check_map(map_file, map_config_file):
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
        except:
            obstacle_ids = [0, 1]
            width = 40.0
            height = 30.0
            num_classes = 15
    try:
        img = cv2.imread(map_file)

    except:
        print("Error while reading file: " + map_file)
        return False

    map_path, map_file_name = os.path.split(arguments.map)
    map_name = map_file_name.split(".")[0]

    labels = color2label(img, num_classes)
    min_label = np.Inf
    max_label = -1
    min_location = [-1, -1]
    max_location = [-1, -1]
    unique_labels = []

    for i in range(labels.shape[0]):
        for j in range(labels.shape[1]):
            if labels[i, j] not in unique_labels:
                unique_labels.append(labels[i, j])
            if labels[i, j] < min_label:
                min_label = labels[i, j]
                min_location = [i, j]
            if labels[i, j] > max_label:
                max_label = labels[i, j]
                max_location = [i, j]

    unique_labels.sort()

    while True:
        os.system('clear')
        print("The following operations ar possible for image file " + map_file_name + ": ")
        print(" ")

        print("A ... Abort")
        print("C ... Check pixel by position [m]")
        print("P ... Check pixel by location [px]")
        print("S ... Get statistics")

        print(" ")
        decision = input("What do you want to do: ")

        decision = decision.casefold()

        if decision == "a":
            os.system('clear')
            return True
        elif decision == "c":
            x_coord = "x"
            x_coord_num = -1
            while not 0.0 <= x_coord_num < width:
                os.system('clear')
                x_coord = input("Please enter x coordinate [m]: ")
                try:
                    x_coord_num = float(x_coord)
                except:
                    x_coord_num = -1
            y_coord = "x"
            y_coord_num = -1
            while not 0.0 <= y_coord_num < height:
                os.system('clear')
                y_coord = input("Please enter y coordinate [m]: ")
                try:
                    y_coord_num = float(y_coord)
                except:
                    y_coord_num = -1
            os.system('clear')
            position = [x_coord_num, y_coord_num]
            x_coord_num = int((x_coord_num / width) * img.shape[1])
            y_coord_num = int((y_coord_num / height) * img.shape[0])
            print("Label: \t\t" + str(labels[y_coord_num, x_coord_num]))
            print("Color: \t\t" + str(list(img[y_coord_num, x_coord_num])))
            print("Location[px]: \t" + str([x_coord_num, y_coord_num]))
            print("Position[m]: \t" + str(position))
            print("Is Obstacle: \t" + str(labels[y_coord_num, x_coord_num] in obstacle_ids))
            print_square(position, height / img.shape[0])
            input("Press key to continue!")

        elif decision == "p":
            x_coord = "x"
            x_coord_num = -1
            while not (x_coord.isnumeric() and 0 <= x_coord_num < img.shape[1]):
                os.system('clear')
                x_coord = input("Please enter x coordinate [px]: ")
                if x_coord.isnumeric():
                    x_coord_num = int(x_coord)
            y_coord = "x"
            y_coord_num = -1
            while not (y_coord.isnumeric() and 0 <= y_coord_num < img.shape[0]):
                os.system('clear')
                y_coord = input("Please enter y coordinate [px]: ")
                if y_coord.isnumeric():
                    y_coord_num = int(y_coord)
            os.system('clear')
            position = [width * float(x_coord_num)/img.shape[1], height * float(y_coord_num)/img.shape[0]]
            print("Label: \t\t" + str(labels[y_coord_num, x_coord_num]))
            print("Color: \t\t" + str(list(img[y_coord_num, x_coord_num])))
            print("Location[px]: \t" + str([x_coord_num, y_coord_num]))
            print("Position[m]: \t" + str(position))
            print("Is Obstacle: \t" + str(labels[y_coord_num, x_coord_num] in obstacle_ids))
            input("Press key to continue!")

        elif decision == "s":
            os.system('clear')
            print("Present Labels: \t" + str(unique_labels))
            print("Obstacle IDs: \t\t" + str(obstacle_ids))
            print()
            print("Max. Label: \t\t" + str(max_label))
            print("Max. Label Color: \t" + str(list(img[max_location[0], max_location[1]])))
            print("Max. Label Location: \t" + str(max_location))
            print()
            print("Min. Label: \t\t" + str(min_label))
            print("Min. Label Color: \t" + str(list(img[min_location[0], min_location[1]])))
            print("Min. Label Location: \t" + str(min_location))
            input("Press key to continue!")


def print_square(position, m_per_px):
    n = 15
    position_px = np.asarray([position[0] / m_per_px, position[1] / m_per_px])
    location = np.asarray([int(position[0] / m_per_px), int(position[1] / m_per_px)])
    sub_pixel = position_px - location
    coordinate = np.asarray([int(sub_pixel[0] * n), int(sub_pixel[1] * n)])
    print("Position within Pixel:")
    for i in range(n):
        print("  ", end="")
        for j in range(n):
            if j == coordinate[0] and i == coordinate[1]:
                print("o ", end="")
            elif i == 0 or j == 0 or i == n - 1 or j == n - 1:
                print("x ", end="")
            else:
                print("  ", end="")
        print('')


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


    check_map(arguments.map, map_config_file)