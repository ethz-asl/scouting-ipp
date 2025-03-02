import io
import os
import cv2
import copy
import errno
import argparse
import numpy as np
import pandas as pd
from map_conversion_functions import label2color, color2label
from create_map_visualization import create_map_visualization
from create_map_config import create_config_file
from analyse_map import analyse_map

def parse_arguments():
    parser = argparse.ArgumentParser(description='Creates artificial maps configured in a csv file.')
    parser.add_argument('source', type=str,
                        help='input file ')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="/tmp/",
                        help='output folder where data shall be stored')
    parser.add_argument('-n', dest='num_classes', type=int, default=15,
                        help='number of data classes used for segmentation', required=False)
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('--width', dest='width', type=int, default=640,
                        help='width of the output image', required=False)
    parser.add_argument('--height', dest='height', type=int, default=480,
                        help='height of the output image', required=False)
    parser.add_argument('-d', dest='default_label', type=int, default=0,
                        help='default label that is used to fill all undefined areas of the map', required=False)
    parser.add_argument('-v', dest='create_visualization', action='store_true',
                        help='create map visualization')
    parser.add_argument('-c', dest='create_config', action='store_true',
                        help='create map configuration file')
    parser.add_argument('-a', dest='analyse', action='store_true',
                        help='analyse map')
    parser.add_argument('--name', dest='name', type=str, default="new_map",
                        help='name of the output map')
    parser.add_argument('--oid', dest='obstacle_ids', type=int, nargs='*',
                        help='IDs to be considered obstacles', required=False)

    args = parser.parse_args()
    return args


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    source = arguments.source
    results_folder = arguments.destination_folder
    override_all = arguments.override_all
    num_classes = arguments.num_classes
    name = arguments.name

    #TODO: Read this from config file or arguments 
    start_px=[8, 8]
    goal_px=[632, 472]
    #start_px=[240, 240]
    #goal_px=[425, 240]

    if not os.path.isdir(results_folder): 
        os.mkdir(results_folder)

    if arguments.obstacle_ids is None:
        obstacle_ids = [0]
    else: 
        obstacle_ids = arguments.obstacle_ids

    df = pd.read_csv(source, header=0, quotechar='|', dtype=np.int64)
    areas = df.to_numpy()

    label_map = np.ones((arguments.height, arguments.width), np.int64) * arguments.default_label

    for i in range(areas.shape[0]):
        x = areas[i, 0]
        y = areas[i, 1]
        w = areas[i, 2]
        h = areas[i, 3]
        label = areas[i, 4]

        if not (0 <= label and 0 <= w and 0 <= h and 0 <= x <= arguments.width-w and 0 <= y <= arguments.height-h): 
            print("Encountered unallowed area: id=", i, ", x= ", x, ", y= ", y, ", w= ", w, ", h= ", h, ", label= ", label)
            continue

        label_map[y:y+h, x:x+w] = np.ones((h, w)) * label


    image = label2color(label_map, arguments.num_classes)
    output_file = os.path.join(results_folder, name + ".png")
    if os.path.exists(output_file) and not arguments.override_all: 
        print("Map exists and override was not specified!")
    else: 
        cv2.imwrite(output_file , image)
        print("Map successfully written to: ", output_file)

    vis_folder = os.path.join(results_folder, "visualization")
    if not os.path.isdir(vis_folder):
        os.mkdir(vis_folder)

    if arguments.create_visualization: 
        create_map_visualization(output_file, vis_folder, num_classes, obstacle_ids, 3599, override_all, range_min=None, range_max=None, start_px=start_px, goal_px=goal_px)
    
    config_folder = os.path.join(results_folder, "cfg")
    if not os.path.isdir(config_folder):
        os.mkdir(config_folder)
    
    if arguments.create_config or arguments.analyse: 
        create_config_file(output_file, config_folder, num_classes, False, override_all, obstacle_ids, True, start_px=start_px, goal_px=goal_px)
    map_config_file = os.path.join(config_folder, name + ".yaml")

    report_folder = os.path.join(results_folder, "reports")
    if not os.path.isdir(report_folder):
        os.mkdir(report_folder)

    if arguments.analyse: 
        analyse_map(output_file, report_folder, map_config_file, num_classes = num_classes,
                              override_all=override_all,
                              create_statistics_file=False,
                              create_path_file=False,
                              create_path_vis_file=False, iterations=10)
    

    
    

    



