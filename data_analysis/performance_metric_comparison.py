import copy
import os
import errno
import pickle
import argparse
import exp_data_classes
from performance_metrics import *
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot data of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    parser.add_argument('--names', dest='names', nargs="*", type=str, default=None,
                        help='')
    parser.add_argument('--title', dest='title', type=str, default=None,
                        help='')
    parser.add_argument('--files', dest='exp_files', nargs="*", type=str,
                        help='output folder of the path planner containing the logged data')
    parser.add_argument('-m', dest='compute_min_max', action='store_true',
                        help='compute minimum and maximum for each metric')

    args = parser.parse_args()
    return args


def compute_metrics(data: exp_data_classes.ExperimentBatch, compute_min_max=False):
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
    batch_performance = {}
    for key in result_dict:
        try:
            data_array = np.array(result_dict[key])
            batch_performance[key] = np.nanmean(data_array)
            if compute_min_max: 
                batch_performance["min_" + key] = np.nanmin(data_array)
                batch_performance["max_" + key] = np.nanmax(data_array)
                batch_performance["stddev_" + key] = np.nanstd(data_array)
        except:
            pass
    dataframe = pd.DataFrame(batch_performance, index=[0])
    runs_df=pd.DataFrame(result_dict)
    runs_df.index.name = "Run"
    return dataframe, runs_df


def load_data(file_path):

    try:
        if os.path.isfile(file_path):
            with open(file_path, 'rb') as handle:
                data = pickle.load(handle)
        else:
            raise FileNotFoundError

    except FileNotFoundError:
        print("The specified File was not found: " + file_path)
        return None

    except:
        print("An undefined error occurred during data loading!!")
        return None

    return data


def check_folder_path(folder_path):

    try:
        if os.path.isdir(folder_path):
            return folder_path
        else:
            raise NotADirectoryError

    except NotADirectoryError:
        print("The specified destination directory was not found: " + folder_path)
        exit(1)

    except:
        print("An undefined error occurred during folder check!!")
        exit(1)


def store_run_performance(performance_runs, names, destination_folder): 
    for i in range(len(performance_runs)): 
        performance_runs[i]["name"] = names[i]
    
    experiments_runs = pd.concat(performance_runs)

    name_conversion = get_naming_conversion(experiments_runs.keys())
    experiments_runs = experiments_runs.rename(columns=name_conversion)

    with open(os.path.join(destination_folder, "performance_runs_table.csv"), 'w') as f:
        f.write(experiments_runs.to_csv())


def get_naming_conversion(keys): 
    name_conversion = {}
    for key in keys:
        key_components = key.casefold().split("_")
        new_key = key
        if "best" in key and len(key_components) >= 2:
            new_key = new_key.replace("_", " ", len(key_components) - 2)
            new_key = new_key.replace("_", ".")
            new_key += "\%"
        new_key = new_key.replace("_", " ")
        new_key = new_key.title().replace("Ipp", "IPP").replace("Rrt", "RRT")
        name_conversion[key] = new_key
    return name_conversion


def make_comparison(experiments, names, destination_folder, title=None):
    results = pd.concat(experiments, keys=names)
    name_conversion = get_naming_conversion(results.keys())
    results = results.rename(columns=name_conversion)
    with open(os.path.join(destination_folder, "performance_result_table.csv"), 'w') as f:
        f.write(results.to_csv())

    s = results.style.highlight_min(props='bfseries:;')
    s.format(precision=2).hide(level=1, axis=0)
    s.applymap_index(lambda v: "rotatebox:{90}--rwrap;", level=0, axis=1)
    col_format = "l"
    for _ in results.keys():
        col_format += "|c"
    with open(os.path.join(destination_folder, "performance_result_table.tex"), 'w') as f:
        f.write(s.to_latex(column_format=col_format, hrules=True))



# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()
    experiments = []
    experiments_runs = []
    names = []
    title = arguments.title
    compute_min_max = arguments.compute_min_max

    use_names = False
    if arguments.names is not None:
        if len(arguments.names) == len(arguments.exp_files):
            use_names = True
        else:
            print("Number of names and number of experiments does not match. Using default names instead!")
            use_names = False
    else:
        use_names = False

    map_names = []
    evaluator_names = []

    for i, exp_file_path in enumerate(arguments.exp_files):
        batch = load_data(exp_file_path)
        if batch is not None:
            batch.evaluate_data()
            performance_metrics, runs_performance = compute_metrics(batch, compute_min_max=compute_min_max)
            experiments.append(performance_metrics)
            experiments_runs.append(runs_performance)
            if use_names:
                names.append(arguments.names[i])
            else:
                names.append(batch.batch_name)
            map_names.append(batch.map_name)
            evaluator_names.append(batch.ipp_gain_evaluator)

    if not use_names:
        if "" not in map_names and len(set(map_names)) == len(map_names) and len(set(evaluator_names)) == 1:
            names = map_names
            if title is None and evaluator_names[0] != "":
                title = "Comparison " + evaluator_names[0]
        elif "" not in evaluator_names and len(set(evaluator_names)) == len(evaluator_names) and len(set(map_names)) == 1:
            names = evaluator_names
            if title is None and map_names[0] != "":
                title = "Comparison " + map_names[0]

    store_run_performance(experiments_runs, names, check_folder_path(arguments.destination_folder))
    make_comparison(experiments, names, check_folder_path(arguments.destination_folder), title=title)
