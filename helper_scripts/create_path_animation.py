import os
import io
import cv2
import sys
import pickle
import errno
import argparse
import numpy as np
sys.path.insert(0, '../data_analysis/')
from plotting_functions import plot_path_evolution


def parse_arguments():
    parser = argparse.ArgumentParser(description='Visualizes rover path evolution as gif.')
    parser.add_argument('source', type=str,
                        help='input file (pickle)')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where evaluation data shall be stored')
    parser.add_argument('-m', dest='map_file', type=str, default="",
                        help='path to the map in which the paths should be plotted')
    parser.add_argument('-n', dest='n_run', type=int, default=0,
                        help='number of the run to be plotted')
    args = parser.parse_args()
    return args


def load_data(file_path):

    try:
        if os.path.isfile(file_path):
            with open(file_path, 'rb') as handle:
                data = pickle.load(handle)
        else:
            raise FileNotFoundError

    except FileNotFoundError:
        print("The specified File was not found: " + file_path)
        exit(errno.ENOENT)

    except:
        print("An undefined error occurred during data loading!!")
        exit(1)

    return data


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    map_file = None
    if arguments.map_file != "":
        if os.path.isfile(arguments.map_file) and arguments.map_file.split(".")[-1] == "png":
            map_file = arguments.map_file

    batch = load_data(arguments.source)

    batch.evaluate_data()

    n_run = 0
    if 0 <= arguments.n_run < len(batch):
        n_run = arguments.n_run

    plot_path_evolution(batch, arguments.destination_folder, map_file, n_run)
