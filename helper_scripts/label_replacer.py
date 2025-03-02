import copy
import os
import io
import cv2
import errno
import argparse
import numpy as np
import yaml as YAML
from map_conversion_functions import label2color, color2label


def parse_arguments():
    parser = argparse.ArgumentParser(description='Replaces labels in a given map file.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where new maps shall be stored')
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('--map', dest='map_yaml', type=str, default=None,
                        help='yaml file used to directly map labels to new ones', required=False)
    parser.add_argument('-s', dest='supress_ending', action='store_true',
                        help='supress post-fix for output file')
    parser.add_argument('-pf', dest='post_fix', type=str, default=None,
                        help='alternative post-fix for file', required=False)

    args = parser.parse_args()
    return args


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


def change_labels(map_file, results_folder, num_classes, override_all=False, map_yaml=None, supress_ending=False,
                  post_fix=None):
    try:
        img = cv2.imread(map_file)

    except:
        print("Error while reading file: " + map_file)
        return False

    map_path, map_file_name = os.path.split(map_file)
    map_name = map_file_name.split(".")[0]

    labels = color2label(img, num_classes)

    unique_labels = []
    label_count = {}
    label_color = {}
    new_label = {}

    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            label = labels[i, j]
            if label not in unique_labels:
                unique_labels.append(label)
                label_count[label] = 0
                label_color[label] = img[i, j]
                new_label[label] = label
            label_count[label] += 1

    unique_labels.sort()

    setup_finished = False
    if map_yaml is not None:
        map_data = read_yaml_config_file(map_yaml)
        if map_data != {}:
            replacement_counter = 0
            for label in new_label:
                if label in map_data.keys():
                    new_label[label] = map_data[label]
                    replacement_counter += 1
            if replacement_counter > 0:
                setup_finished = True
            else:
                return False

    while not setup_finished:
        os.system('clear')
        print("The following colors/labels were found in th image file " + map_file_name + ": ")
        print(" ")
        print("ID" + " " + "Label" + " " + "(b, g, r)" + " ==> " + "New Label")
        for i, label in enumerate(unique_labels):
            if new_label[label] == label:
                print(str(i).zfill(2) + " " + str(label).zfill(4) + " " + str(label2color(label, num_classes)) + " ... " + "unchanged")
            else:
                print(str(i).zfill(2) + " " + str(label).zfill(4) + " " + str(label2color(label, num_classes)) + " ==> " + str(new_label[label]) + " " + str(label2color(new_label[label], num_classes)))

        print(" ")
        print(" ")
        print("Options: ")
        print("A ... Abort")
        print("D ... Done editing, create file!")
        print("S ... Skipp image")
        print("X ... Change label X")
        print(" ")
        decision = input("What do you want to do: ")

        decision = decision.casefold()

        if decision == "s":
            return False
        elif decision == "a":
            exit(1)
        elif decision == "d":
            setup_finished = True
        elif decision.isnumeric():
            if 0 <= int(decision) < len(unique_labels):
                label_to_change = unique_labels[int(decision)]
                correct_label_entered = False
                while not correct_label_entered:
                    os.system('clear')
                    print("Image file  " + map_file_name + ": ")
                    new_label_entered = input("What do you want to change label " + str(label_to_change) + " to? ")
                    if new_label_entered.isnumeric():
                        new_label_entered = int(new_label_entered)
                        if 0 <= new_label_entered <= num_classes**3+num_classes**2-1:
                            new_label[label_to_change] = new_label_entered
                            correct_label_entered = True
                        else:
                            correct_label_entered = False
                    else:
                        correct_label_entered = False
            setup_finished = False

        else:
            setup_finished = False

    new_labels = copy.deepcopy(labels)
    new_img = copy.deepcopy(img)

    for i in range(new_labels.shape[0]):
        for j in range(new_labels.shape[1]):
            new_labels[i, j] = new_label[labels[i, j]]
            new_img[i, j] = label2color(new_label[labels[i, j]], num_classes)

    os.system('clear')
    if post_fix is None:
        post_fix = "_replaced"
    if supress_ending:
        post_fix = ""
    output_file = os.path.join(results_folder, map_name + post_fix + ".png")
    print("New image file for file " + map_file_name + " will be stored at: \n" + output_file)
    if not os.path.isfile(output_file) or override_all:
        cv2.imwrite(output_file, new_img)
    else:
        input_override = input("Do you want to override file [Y]/n: \n" + output_file + "\n")
        input_override = input_override.casefold()
        if input_override == "y" or input_override == "":
            cv2.imwrite(output_file, new_img)
        else:
            return False
    return True


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

    map_yaml = arguments.map_yaml
    if map_yaml is not None:
        if not os.path.isfile(map_yaml):
            text_line = "The specified map file was not found: " + map_yaml
            input(text_line)

    file_counter = 0
    maps.sort()

    for map_file in maps:
        success = change_labels(map_file, results_folder, num_classes=arguments.num_classes,
                                override_all=arguments.override_all, map_yaml=map_yaml,
                                supress_ending=arguments.supress_ending, post_fix=arguments.post_fix)
        if success:
            file_counter += 1