import os
import errno
import pickle
import argparse
import exp_data_classes
import numpy as np
import pandas as pd
from performance_metrics import *


def parse_arguments():
    parser = argparse.ArgumentParser(description='Print performance data of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    #parser.add_argument('--map', dest='map_path', type=str, default=None,
    #                    help='path of map visualization file')
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


def get_results(data: exp_data_classes.ExperimentBatch, destination_folder=None):
    result_dict = {}
    first_path_coverage(data, result_dict=result_dict)
    first_path_cost(data, result_dict=result_dict)
    best_path_coverage(data, 0.01, result_dict=result_dict)
    best_path_coverage(data, 0.05, result_dict=result_dict)
    normalized_path_cost_difference(data, result_dict=result_dict)
    time_to_first_path(data, result_dict=result_dict)
    time_to_best_path(data, 0.01, result_dict=result_dict)
    time_to_best_path(data, 0.05, result_dict=result_dict)
    time_to_termination(data, result_dict=result_dict)
    ipp_cost_to_first_path(data, result_dict=result_dict)
    ipp_cost_to_best_path(data, 0.01, result_dict=result_dict)
    ipp_cost_to_best_path(data, 0.05, result_dict=result_dict)
    ipp_cost_to_termination(data, result_dict=result_dict)
    coverage_at_termination(data, result_dict=result_dict)
    dataframe = pd.DataFrame(result_dict)
    dataframe.to_csv(os.path.join(destination_folder, data.batch_name + "_run_performance_metrics.csv")
                     , index_label="run")
    batch_performance = {}
    for key in result_dict:
        try:
            data_array = np.array(result_dict[key])
            batch_performance[key] = np.nanmean(data_array)
        except:
            pass
    dataframe = pd.DataFrame(batch_performance, index=[0])
    dataframe.to_csv(os.path.join(destination_folder, data.batch_name + "_performance_metrics.csv"), index=False)




# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    batch = load_data(arguments.exp_file)

    batch.evaluate_data()

    get_results(batch, arguments.destination_folder)
