import os
import errno
import pickle
import argparse
import exp_data_classes
from plotting_functions import *
import numpy as np
os_version = os.popen("lsb_release -r | cut -f2").read().replace("\n", "")
if os_version == "18.04":
    import matplotlib
    matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['CMU Serif']
plt.rcParams['mathtext.fontset'] = 'cm'

def parse_arguments():
    parser = argparse.ArgumentParser(description='Plot data of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where plots shall be stored')
    parser.add_argument('--names', dest='names', nargs="*", type=str, default=None,
                        help='')
    parser.add_argument('--file_name', dest='file_name', type=str, default=None,
                        help='prefix for all file names')
    parser.add_argument('--title', dest='title', type=str, default=None,
                        help='')
    parser.add_argument('--baseline', dest='baseline', type=float, default=None,
                        help='')
    parser.add_argument('--y_lim', dest='y_lim', type=float, default=None,
                        help='')
    parser.add_argument('--x_lim', dest='x_lim', type=float, default=None,
                        help='')
    parser.add_argument('--font_size', dest='font_size', type=float, default=9.0,
                        help='Font size for legend, labels and title')
    parser.add_argument('--no_label', dest='no_label', action='store_true',
                        help='')
    parser.add_argument('--mav_comp', dest='mav_comp', action='store_true',
                        help='only use for comparing normal amd mav runs')
    parser.add_argument('--no_title', dest='suppress_title', action='store_true',
                        help='suppresses titles for all plots')
    parser.add_argument('--suppress_x_label_cost', dest='suppress_x_label_cost', action='store_true',
                        help='suppresses x-label for cost plots')
    parser.add_argument('--suppress_y_label_cost', dest='suppress_y_label_cost', action='store_true',
                        help='suppresses y-label for cost plots')
    parser.add_argument('--suppress_x_label_coverage', dest='suppress_x_label_coverage', action='store_true',
                        help='suppresses x-label for coverage plots')
    parser.add_argument('--suppress_y_label_coverage', dest='suppress_y_label_coverage', action='store_true',
                        help='suppresses y-label for coverage plots')
    parser.add_argument('--suppress_x_label_success', dest='suppress_x_label_success', action='store_true',
                        help='suppresses x-label for success plots')
    parser.add_argument('--suppress_y_label_success', dest='suppress_y_label_success', action='store_true',
                        help='suppresses y-label for success plots')
    parser.add_argument('--files', dest='exp_files', nargs="*", type=str,
                        help='output folder of the path planner containing the logged data')

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


def plot_comparison(experiments, names, destination_folder, title=None, colors=None, baseline=None, y_lim=None, x_lim=None,
                    mav_comp=False, suppress_title=False, file_name=None, suppress_x_label_cost=False, suppress_y_label_cost=False, 
                    suppress_x_label_coverage=False, suppress_y_label_coverage=False, suppress_x_label_success=False, suppress_y_label_success=False):
    new_colors = []
    if colors is None:
        for i in range(len(experiments)):
            new_colors.append(list(mcolors.TABLEAU_COLORS)[np.mod(i, len(mcolors.TABLEAU_COLORS))])

    plot_success_rate_bars(experiments, destination_folder, labels=names, colors=colors, title=title,
                           suppress_title=suppress_title, file_name=file_name, x_lim=x_lim, suppress_x_label=suppress_x_label_success, suppress_y_label=suppress_y_label_success)
    plot_success_rate(experiments, destination_folder, labels=names, colors=colors, title=title, mav_comp=mav_comp, x_lim=x_lim,
                      suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_success, suppress_y_label=suppress_y_label_success)
    plot_travel_costs(experiments, destination_folder, labels=names, colors=colors, title=title, baseline=baseline,
                      y_lim=y_lim, x_lim=x_lim, mav_comp=mav_comp, suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_cost, suppress_y_label=suppress_y_label_cost)
    plot_travel_costs(experiments, destination_folder, labels=names, colors=colors, title=title, use_plotting_data=True,
                      baseline=baseline, y_lim=y_lim, x_lim=x_lim, mav_comp=mav_comp, suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_cost, suppress_y_label=suppress_y_label_cost)
    plot_coverage(experiments, destination_folder, labels=names, colors=colors, title=title, mav_comp=mav_comp, x_lim=x_lim,
                  suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_coverage, suppress_y_label=suppress_y_label_coverage)
    plot_travel_costs(experiments, destination_folder, labels=names, colors=colors, title=title, relative_to_min=True,
                      baseline=baseline, y_lim=y_lim, x_lim=x_lim, mav_comp=mav_comp, suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_cost, suppress_y_label=suppress_y_label_cost)
    plot_travel_costs(experiments, destination_folder, labels=names, colors=colors, title=title, use_plotting_data=True,
                      relative_to_min=True, baseline=baseline, y_lim=y_lim, x_lim=x_lim, mav_comp=mav_comp,
                      suppress_title=suppress_title, file_name=file_name, suppress_x_label=suppress_x_label_cost, suppress_y_label=suppress_y_label_cost)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    plt.rcParams.update({'font.size': arguments.font_size})

    experiments = []
    names = []
    title = arguments.title
    file_name = arguments.file_name

    if file_name == "": 
        file_name = None

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
            experiments.append(batch)
            if use_names:
                names.append(arguments.names[i].replace("^", " "))
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

    if arguments.no_label == True:
        names=None

    plot_comparison(experiments, names, check_folder_path(arguments.destination_folder), title=title,
                    baseline=arguments.baseline, y_lim=arguments.y_lim, x_lim=arguments.x_lim, mav_comp=arguments.mav_comp,
                    suppress_title=arguments.suppress_title, file_name=file_name, 
                    suppress_x_label_cost=arguments.suppress_x_label_cost, suppress_y_label_cost=arguments.suppress_y_label_cost, 
                    suppress_x_label_coverage=arguments.suppress_x_label_coverage, suppress_y_label_coverage=arguments.suppress_y_label_coverage, 
                    suppress_x_label_success=arguments.suppress_x_label_success, suppress_y_label_success=arguments.suppress_y_label_success)