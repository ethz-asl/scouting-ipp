import os
import errno
import pickle
import argparse
import exp_data_classes
import numpy as np
os_version = os.popen("lsb_release -r | cut -f2").read().replace("\n", "")
if os_version == "18.04":
    import matplotlib
    matplotlib.use('agg')
import matplotlib.pyplot as plt
from plotting_functions import *

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['CMU Serif']
plt.rcParams['mathtext.fontset'] = 'cm'


def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot data of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    parser.add_argument('--map', dest='map_path', type=str, default=None,
                        help='path of map visualization file')
    parser.add_argument('--font_size', dest='font_size', type=int, default=12,
                        help='Font size for legend, labels and title')
    parser.add_argument('exp_file', type=str,
                        help='output folder of the path planner containing the logged data')
    # parser.add_argument('--sum', dest='accumulate', action='store_const',
    #                     const=sum, default=max,
    #                     help='sum the integers (default: find the max)')

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


def plot_results(data: exp_data_classes.ExperimentBatch, destination_folder=None, map_path=None):
    plot_success_rate_bars([data], destination_folder)
    plot_success_rate([data], destination_folder)
    plot_travel_costs([data], destination_folder)
    plot_travel_costs([data], destination_folder, use_plotting_data=True)
    plot_travel_costs([data], destination_folder, relative_to_min=True)
    plot_travel_costs([data], destination_folder, use_plotting_data=True, relative_to_min=True)
    plot_coverage([data], destination_folder)
    plot_combined_data(data, destination_folder)
    plot_combined_data(data, destination_folder, use_plotting_data=True)
    plot_combined_data(data, destination_folder, relative_to_min=True)
    plot_combined_data(data, destination_folder, use_plotting_data=True, relative_to_min=True)
    plot_path_overlay(data, destination_folder, map_path)
    plot_path_overlay(data, destination_folder, map_path, use_first_successful=True)
    plot_coverage([data], destination_folder, do_run_comparison=True)
    plot_travel_costs([data], destination_folder, do_run_comparison=True)
    plot_travel_costs([data], destination_folder, relative_to_min=True, do_run_comparison=True)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    plt.rcParams.update({'font.size': arguments.font_size})

    batch = load_data(arguments.exp_file)

    batch.evaluate_data()

    plot_results(batch, arguments.destination_folder, arguments.map_path)

    #plt.show()
