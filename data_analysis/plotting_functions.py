import os
import cv2
import copy
import exp_data_classes
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.lines as mlines
import matplotlib.animation as animation

LINE_STYLES_DEF = list(filter(lambda x: (x != 'None' and x != ' ' and x != ''), mlines.lineStyles.keys()))
LINE_STYLES_PARAM = [
     ('loosely dotted',        (0, (1, 10))),
     ('dotted',                (0, (1, 1))),
     ('densely dotted',        (0, (1, 1))),
     ('long dash with offset', (5, (10, 3))),
     ('loosely dashed',        (0, (5, 10))),
     ('dashed',                (0, (5, 5))),
     ('densely dashed',        (0, (5, 1))),

     ('loosely dashdotted',    (0, (3, 10, 1, 10))),
     ('dashdotted',            (0, (3, 5, 1, 5))),
     ('densely dashdotted',    (0, (3, 1, 1, 1))),

     ('dashdotdotted',         (0, (3, 5, 1, 5, 1, 5))),
     ('loosely dashdotdotted', (0, (3, 10, 1, 10, 1, 10))),
     ('densely dashdotdotted', (0, (3, 1, 1, 1, 1, 1)))]

LINE_STYLES = ["-"]
LINE_STYLES.extend([ls for (_, ls) in LINE_STYLES_PARAM])

Y_LABEL_COST = "Follower Cost $Q_F/Q_F^\star$ [-]"

def get_colors(n): 
    if n <= 3: 
        return ["b", "g", "r"][0:n]
    if n == 4: 
        return ["b", "k", "g", "r"]
    elif n == 5: 
        return ["b", "tab:purple", "k", "g", "r"]
    return [list(mcolors.TABLEAU_COLORS)[np.mod(i, len(mcolors.TABLEAU_COLORS))] for i in range(n)]

def get_linestyle(n): 
    if n <= 3: 
        return ["-", "--", ":"][0:n]
    if n == 4: 
        return ["--", "-.", ":", "-"]
    if n == 5: 
        return ["--", "-.", (0, (3, 5, 1, 5, 1, 5)), ":", "-"]
    return [LINE_STYLES[np.mod(i, len(LINE_STYLES))] for i in range(n)]

def plot_success_rate_bars(data_list: list[exp_data_classes.ExperimentBatch], destination_folder=None, labels=None,
                           title=None, colors=None, suppress_title=False, file_name=None, x_lim=None, suppress_x_label=False, 
                           suppress_y_label=False):
    do_comparison = len(data_list) > 1
    if do_comparison:
        if title is None:
            title = "Comparison"
        top_name = title
        batch_name = title
    else:
        batch_name = data_list[0].batch_name
        top_name = check_title_validity(data_list)

    trivial_path_cost = check_trivial_path_cost_validity(data_list)
    trivial_path_cost = None #TODO: Remove again

    show_labels = True
    if labels is None:
        show_labels = False
        labels = ["" for _ in data_list]

    if colors is None:
        colors = get_colors(len(data_list))

    max_travel_cost_ipp = data_list[0].max_travel_cost_ipp
    min_travel_cost_ipp = data_list[0].min_travel_cost_ipp
    num_travel_costs = len(data_list[0].travel_costs_ipp)

    for data in data_list:
        if data.max_travel_cost_ipp > max_travel_cost_ipp:
            max_travel_cost_ipp = data.max_travel_cost_ipp
        if data.min_travel_cost_ipp < min_travel_cost_ipp:
            min_travel_cost_ipp = data.min_travel_cost_ipp

    if do_comparison:
        bar_width = 0.8 * (max_travel_cost_ipp - min_travel_cost_ipp) / (num_travel_costs * len(data_list))
    else:
        bar_width = 0.5 * (max_travel_cost_ipp - min_travel_cost_ipp) / num_travel_costs

    fig, ax_success = plt.subplots(figsize=(5, 2.7), layout='constrained')
    if trivial_path_cost is not None:
        ax_success.axvline(trivial_path_cost, color='grey', linestyle='--', linewidth=0.8, alpha=0.8)
    if not suppress_x_label:
        ax_success.set_xlabel('Scout Cost $Q_{S}$ [' + data_list[0].get_cost_unit() + ']')
    if not suppress_y_label:
        ax_success.set_ylabel('Success Rate [%]')
    ax_success.set_title('Success Rate ' + top_name)

    for i, data in enumerate(data_list):
        ax_success.bar(np.round(data.travel_costs_ipp, 0), np.round(data.success_rate * 100, 0), width=bar_width,
                       label=labels[i], color=colors[i])
    if show_labels:
        ax_success.legend()
    ax_success.set_ylim(0, 100)
    if x_lim is None: 
        ax_success.set_xlim(left=0)
    else: 
        ax_success.set_xlim(left=0, right=x_lim)
    if suppress_title:
        ax_success.set_title("")
    if file_name is None: 
        file_name = batch_name.replace(' ', '_') 
    file_name  += "_success_rate_bars"
    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)


def plot_travel_costs(data_list: list[exp_data_classes.ExperimentBatch], destination_folder=None,
                      use_plotting_data=False, labels=None, title=None, colors=None, relative_to_min=False,
                      do_run_comparison=False, baseline=None, y_lim=None, x_lim=None, mav_comp=False, suppress_title=False, 
                      file_name=None, suppress_x_label=False, suppress_y_label=False):
    if do_run_comparison:
        labels = None
        if len(data_list) > 1:
            for experiment in data_list:
                plot_travel_costs([experiment], destination_folder, use_plotting_data=use_plotting_data, title=title,
                                  relative_to_min=relative_to_min, do_run_comparison=True)
            return
        else:
            data_list = [data_list[0] for _ in range(len(data_list[0]))]
            if title is None:
                title = check_title_validity(data_list)
    do_comparison = len(data_list) > 1
    if do_comparison:
        if title is None:
            title = "Comparison"
        top_name = title
        batch_name = data_list[0].batch_name
    else:
        batch_name = data_list[0].batch_name
        top_name = check_title_validity(data_list)

    trivial_path_cost = check_trivial_path_cost_validity(data_list)
    optimal_path_cost = check_optimal_path_cost_validity(data_list)
    trivial_path_cost = None  #TODO: Remove again

    show_labels = True
    if labels is None:
        if do_run_comparison:
            show_labels = True
            labels = ["Run " + str(i) for i in range(len(data_list))]
        else:
            show_labels = False
            labels = ["" for _ in data_list]

    markers = get_linestyle(len(data_list))
    if mav_comp:
        markers = ['-', '-', '-.', '-.']
        colors = [list(mcolors.TABLEAU_COLORS)[0], list(mcolors.TABLEAU_COLORS)[1], list(mcolors.TABLEAU_COLORS)[0],
                  list(mcolors.TABLEAU_COLORS)[1]]
    if colors is None:
        colors = get_colors(len(data_list))
    fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')
    if trivial_path_cost is not None:
        ax.axvline(trivial_path_cost, color='k', linestyle='--', linewidth=0.8, alpha=0.8)
    if not relative_to_min and optimal_path_cost is not None:
        ax.axhline(optimal_path_cost, color='grey', linestyle="-.", linewidth=1.0, alpha=1.0)
    elif relative_to_min: 
        ax.axhline(1.0, color='grey', linestyle="-", linewidth=0.8, alpha=1.0)
    for i, data in enumerate(data_list):
        if do_run_comparison:
            if use_plotting_data:
                print("This has not yet been implemented, because the necessary data is not stored in the experiemnt "
                      "data class.")
                return
            else:
                mean_rrt_travel_cost = copy.deepcopy(data.interpolated_travel_cost_rrt[i, :])
                stddev_rrt_travel_cost = np.array([0.0 for _ in mean_rrt_travel_cost])
        else:
            if use_plotting_data:
                mean_rrt_travel_cost = copy.deepcopy(data.plotting_mean_rrt_travel_cost)
                stddev_rrt_travel_cost = copy.deepcopy(data.plotting_stddev_rrt_travel_cost)
            else:
                mean_rrt_travel_cost = copy.deepcopy(data.mean_rrt_travel_cost)
                stddev_rrt_travel_cost = copy.deepcopy(data.stddev_rrt_travel_cost)
        if relative_to_min:
            # The following line was added to allow the comparison of different evaluation planners 
            optimal_path_cost = data.optimal_rov_path_cost
            mean_rrt_travel_cost /= optimal_path_cost
            # Uncomment the following line to shift the optimal cost line to 0 
            # mean_rrt_travel_cost -= 1.0
            stddev_rrt_travel_cost /= optimal_path_cost
        if x_lim is None:
            max_id = -1 
        else: 
            max_id = np.searchsorted(data.travel_costs_ipp, x_lim * 0.95, side='right')
        ax.plot(data.travel_costs_ipp[0:max_id], mean_rrt_travel_cost[0:max_id], label=labels[i], color=colors[i], linestyle=markers[i])
        ax.fill_between(data.travel_costs_ipp[0:max_id], mean_rrt_travel_cost[0:max_id] - stddev_rrt_travel_cost[0:max_id],
                        mean_rrt_travel_cost[0:max_id] + stddev_rrt_travel_cost[0:max_id], color=colors[i], alpha=0.2)
    if show_labels:
        ax.legend(loc='upper right')
    if not suppress_x_label:
        ax.set_xlabel('Scout Cost $Q_{S}$ [' + data_list[0].get_cost_unit() + ']')
    if not suppress_y_label:
        ax.set_ylabel(Y_LABEL_COST)
    ax.set_title('Evaluation ' + top_name)
    if x_lim is None: 
        ax.set_xlim(left=0)
    else: 
        ax.set_xlim(left=0, right=x_lim)
    if relative_to_min:
        if y_lim is not None:
            # Set the following to 0 when schifting the optimal cost line to 0
            ax.set_ylim(bottom=0.95, top=y_lim)
        else:
            # Set the following to 0 when schifting the optimal cost line to 0
            ax.set_ylim(bottom=0.95)
    if suppress_title:
        ax.set_title("")
    if file_name is None:
        file_name = batch_name.replace(' ', '_')
    file_name += "_travel_cost"
    if use_plotting_data:
        file_name += "_plotting"
    if relative_to_min:
        file_name += "_relative"
    if do_run_comparison:
        file_name += "_run_comparison"
    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)
    if baseline is not None:
        if relative_to_min and optimal_path_cost is not None:
            baseline /= optimal_path_cost
            # Uncomment the following line to shift the optimal cost line to 0
            # baseline -= 1.0
            ax.set_ylim(bottom=0, top=baseline+0.1)

        ax.axhline(baseline, color='grey', linestyle="-", linewidth=0.8, alpha=1.0, label="Baseline", in_layout=True)
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles, labels, loc='upper right')
        file_name += "_baseline"
        plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                    format="png", dpi=400)
        plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                    format="pdf", dpi=400)


def plot_coverage(data_list: list[exp_data_classes.ExperimentBatch], destination_folder=None, labels=None, title=None,
                  colors=None, do_run_comparison=False, mav_comp=False, x_lim=None, suppress_title=False, file_name=None,
                  suppress_x_label=False, suppress_y_label=False):
    if do_run_comparison:
        labels = None
        if len(data_list) > 1:
            for experiment in data_list:
                plot_coverage([experiment], destination_folder, title=title, do_run_comparison=True)
            return
        else:
            data_list = [data_list[0] for _ in range(len(data_list[0]))]
            if title is None:
                title = check_title_validity(data_list)
    do_comparison = len(data_list) > 1
    if do_comparison:
        if title is None:
            title = "Comparison"
        top_name = title
        batch_name = data_list[0].batch_name
    else:
        batch_name = data_list[0].batch_name
        top_name = check_title_validity(data_list)
    trivial_path_cost = check_trivial_path_cost_validity(data_list)
    trivial_path_cost = None  #TODO: Remove again

    show_labels = True
    if labels is None:
        if do_run_comparison:
            show_labels = True
            labels = ["Run " + str(i) for i in range(len(data_list))]
        else:
            show_labels = False
            labels = ["" for _ in data_list]

    markers = get_linestyle(len(data_list))
    if mav_comp:
        markers = ['-', '-', '-.', '-.']
        colors = [list(mcolors.TABLEAU_COLORS)[0], list(mcolors.TABLEAU_COLORS)[1], list(mcolors.TABLEAU_COLORS)[0],
                  list(mcolors.TABLEAU_COLORS)[1]]
    if colors is None:
        colors = get_colors(len(data_list))
    fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')
    if trivial_path_cost is not None:
        ax.axvline(trivial_path_cost, color='grey', linestyle='--', linewidth=0.8, alpha=0.8)
    for i, data in enumerate(data_list):
        if do_run_comparison:
            ax.plot(data.travel_costs_ipp, data.interpolated_coverage_ipp[i, :], label=labels[i], color=colors[i])
        else:
            ax.plot(data.travel_costs_ipp, data.mean_ipp_coverage, label=labels[i], color=colors[i], linestyle=markers[i])
            ax.fill_between(data.travel_costs_ipp, data.mean_ipp_coverage - data.stddev_ipp_coverage,
                            data.mean_ipp_coverage + data.stddev_ipp_coverage, color=colors[i], alpha=0.2)
    if show_labels:
        ax.legend(loc='lower right')
    if not suppress_x_label:
        ax.set_xlabel('Scout Cost $Q_{S}$ [' + data_list[0].get_cost_unit() + ']')
    if not suppress_y_label:
        ax.set_ylabel('Coverage [%]')
    ax.set_title('Evaluation ' + top_name)
    if x_lim is None: 
        ax.set_xlim(left=0)
    else: 
        ax.set_xlim(left=0, right=x_lim)
    ax.set_ylim(0, 100)
    if suppress_title:
        ax.set_title("")
    if file_name is None: 
        file_name = batch_name.replace(' ', '_') 
    file_name += "_coverage"
    if do_run_comparison:
        file_name += "_run_comparison"
    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)


def plot_success_rate(data_list: list[exp_data_classes.ExperimentBatch], destination_folder=None, labels=None,
                      title=None, colors=None, mav_comp=False, x_lim=None, suppress_title=False, file_name=None,
                      suppress_x_label=False, suppress_y_label=False):
    do_comparison = len(data_list) > 1
    if do_comparison:
        if title is None:
            title = "Comparison"
        top_name = title
        batch_name = title
    else:
        batch_name = data_list[0].batch_name
        top_name = check_title_validity(data_list)
        if colors is None:
            colors = ["red"]

    trivial_path_cost = check_trivial_path_cost_validity(data_list)
    trivial_path_cost = None  #TODO: Remove again

    show_labels = True
    if labels is None:
        show_labels = False
        labels = ["" for _ in data_list]

    markers = get_linestyle(len(data_list))
    if mav_comp:
        markers = ['-', '-', '-.', '-.']
        colors = [list(mcolors.TABLEAU_COLORS)[0], list(mcolors.TABLEAU_COLORS)[1], list(mcolors.TABLEAU_COLORS)[0],
                  list(mcolors.TABLEAU_COLORS)[1]]
    if colors is None:
        colors = get_colors(len(data_list))
    fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')
    if trivial_path_cost is not None:
        ax.axvline(trivial_path_cost, color='grey', linestyle='--', linewidth=0.8, alpha=0.8)
    for i, data in enumerate(data_list):
        ax.plot(np.round(data.travel_costs_ipp, 0), np.round(data.success_rate * 100, 0), label=labels[i],
                color=colors[i], linestyle=markers[i])
    if show_labels:
        ax.legend(loc='lower right')
    if not suppress_x_label: 
        ax.set_xlabel('Scout Cost $Q_{S}$ [' + data_list[0].get_cost_unit() + ']')
    if not suppress_y_label: 
        ax.set_ylabel('Success Rate [%]')
    ax.set_title('Evaluation ' + top_name)
    if x_lim is None: 
        ax.set_xlim(left=0)
    else: 
        ax.set_xlim(left=0, right=x_lim)
    ax.set_ylim(0, 100)
    if suppress_title:
        ax.set_title("")
    if file_name is None: 
        file_name = batch_name.replace(' ', '_')
    file_name += "_success_rate"
    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)


def plot_combined_data(data: exp_data_classes.ExperimentBatch, destination_folder=None, use_plotting_data=False,
                       relative_to_min=False, suppress_title=False, file_name=None, x_lim=None,
                       suppress_x_label=False, suppress_y_label=False):

    if file_name is None: 
        file_name = data.batch_name 
    file_name += "_combined"
    mean_rrt_travel_cost = copy.deepcopy(data.mean_rrt_travel_cost)
    stddev_rrt_travel_cost = copy.deepcopy(data.stddev_rrt_travel_cost)
    trivial_path_cost = check_trivial_path_cost_validity([data])
    trivial_path_cost = None  #TODO: Remove again
    if use_plotting_data:
        file_name += "_plotting"
        mean_rrt_travel_cost = copy.deepcopy(data.plotting_mean_rrt_travel_cost)
        stddev_rrt_travel_cost = copy.deepcopy(data.plotting_stddev_rrt_travel_cost)

    if data.optimal_rov_path_cost > 0.0:
        optimal_path_cost = data.optimal_rov_path_cost
        optimal_path_cost_alpha = 1.0
    else:
        optimal_path_cost = np.nanmin(mean_rrt_travel_cost)
        optimal_path_cost_alpha = 0.6

    if relative_to_min and optimal_path_cost is not None:
        file_name += "_relative"
        mean_rrt_travel_cost /= optimal_path_cost
        mean_rrt_travel_cost -= 1.0
        stddev_rrt_travel_cost /= optimal_path_cost

    top_name = check_title_validity([data])

    fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')
    ax.set_title('Evaluation ' + top_name)
    ax.plot(data.travel_costs_ipp, mean_rrt_travel_cost, color='blue')
    ax.fill_between(data.travel_costs_ipp, mean_rrt_travel_cost - stddev_rrt_travel_cost,
                    mean_rrt_travel_cost + stddev_rrt_travel_cost, color='blue', alpha=0.2)
    if not relative_to_min:
        ax.axhline(optimal_path_cost, color='blue', linestyle="-.", linewidth=1.0, alpha=optimal_path_cost_alpha)
    if not suppress_x_label:
        ax.set_xlabel('Scout Cost $Q_{S}$ [' + data.get_cost_unit() + ']')
    if not suppress_y_label:
        ax.set_ylabel(Y_LABEL_COST)
    if x_lim is None: 
        ax.set_xlim(left=0)
    else: 
        ax.set_xlim(left=0, right=x_lim)
    ax.spines['left'].set_color('blue')
    ax.yaxis.label.set_color('blue')
    ax.tick_params(axis='y', colors='blue')
    if trivial_path_cost is not None:
        ax.axvline(trivial_path_cost, color='grey', linestyle='--', linewidth=0.8, alpha=0.8)

    ax_success = ax.twinx()
    # make a plot with different y-axis using second axis object
    ax_success.set_ylim(0, 100)
    ax_success.plot(np.round(data.travel_costs_ipp, 0), np.round(data.success_rate * 100, 0), color='red')
    if not suppress_y_label:
        ax_success.set_ylabel('Success Rate [%]')
    # right, left, top, bottom
    ax_success.spines['right'].set_position(('outward', 60))
    ax_success.spines['right'].set_color('red')
    ax_success.yaxis.label.set_color('red')
    ax_success.tick_params(axis='y', colors='red')

    ax_coverage = ax.twinx()
    ax_coverage.set_ylim(0, 100)
    ax_coverage.plot(data.travel_costs_ipp, data.mean_ipp_coverage, color='orange')
    ax_coverage.fill_between(data.travel_costs_ipp, data.mean_ipp_coverage - data.stddev_ipp_coverage,
                             data.mean_ipp_coverage + data.stddev_ipp_coverage, color='orange', alpha=0.2)
    if not suppress_y_label:
        ax_coverage.set_ylabel('Coverage [%]')
    ax_coverage.spines['right'].set_color('orange')
    ax_coverage.yaxis.label.set_color('orange')
    ax_coverage.tick_params(axis='y', colors='orange')

    if suppress_title:
        ax.set_title("")

    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)


def plot_path_overlay(data: exp_data_classes.ExperimentBatch, destination_folder=None, map_path=None,
                      use_first_successful=False, file_name=None):
    if file_name is None: 
        file_name = data.batch_name
    if use_first_successful:
        file_name += "_first_paths"
    else:
        file_name += "_best_paths"
    colors = get_colors(len(data.experiments))
    fig, ax = plt.subplots(layout='constrained')
    if map_path is not None and os.path.isfile(map_path) and map_path.split(".")[-1] == "png":
        img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.flip(img, 0)
        ax.imshow(img, extent=[0, data.map_size[0], 0, data.map_size[1]])
    for i, run in enumerate(data.experiments):
        if use_first_successful:
            idx = -1
            for j, success in enumerate(run.rrt_experiment.success):
                if success:
                    idx = j
                    break
        else:
            idx = -1
        if run.rrt_experiment.paths[idx].size != 0:
            ax.plot(run.rrt_experiment.paths[idx][:, 0], run.rrt_experiment.paths[idx][:, 1], label=("Run " + str(i)),
                    color=colors[i])

    start_circle = plt.Circle((data.experiments[0].rrt_experiment.starts[0][0],
                               data.experiments[0].rrt_experiment.starts[0][1]),
                              data.map_size[1] / 50, facecolor='g', edgecolor='w')
    goal_circle = plt.Circle((data.experiments[0].rrt_experiment.goals[0][0],
                              data.experiments[0].rrt_experiment.goals[0][1]),
                             data.map_size[1] / 50, facecolor='r', edgecolor='w')
    ax.add_patch(start_circle)
    ax.add_patch(goal_circle)

    ax.legend(loc='lower right')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    if use_first_successful:
        ax.set_title('First Successful Path Overlay ' + check_title_validity([data]))
    else:
        ax.set_title('Last Path Overlay ' + check_title_validity([data]))
    ax.set_xlim(0, data.map_size[0])
    ax.set_ylim(0, data.map_size[1])
    if destination_folder is not None:
        if os.path.isdir(destination_folder):
            plt.savefig(os.path.join(destination_folder, file_name + ".png"),
                        format="png", dpi=400)
            plt.savefig(os.path.join(destination_folder, file_name + ".pdf"),
                        format="pdf", dpi=400)


def plot_path_evolution(data: exp_data_classes.ExperimentBatch, destination_folder=None, map_path=None, n_run=0, file_name=None):
    if destination_folder == "" or destination_folder is None:
        destination_folder = "/tmp/"
    if file_name is None: 
        data.batch_name
    start_circle = plt.Circle((data.experiments[n_run].rrt_experiment.starts[0][0],
                               data.experiments[n_run].rrt_experiment.starts[0][1]),
                              data.map_size[1] / 50, facecolor='g', edgecolor='w')
    goal_circle = plt.Circle((data.experiments[n_run].rrt_experiment.goals[0][0],
                              data.experiments[n_run].rrt_experiment.goals[0][1]),
                             data.map_size[1] / 50, facecolor='r', edgecolor='w')
    fig, ax = plt.subplots()
    if map_path is not None and os.path.isfile(map_path) and map_path.split(".")[-1] == "png":
        img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.flip(img, 0)
        ax.imshow(img, extent=[0, data.map_size[0], 0, data.map_size[1]])
    ax.add_patch(start_circle)
    ax.add_patch(goal_circle)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_xlim(0, data.map_size[0])
    ax.set_ylim(0, data.map_size[1])

    line, = ax.plot([], [])

    idxs = [i for i, path in enumerate(data.experiments[n_run].rrt_experiment.paths) if path.size != 0]

    def init_function():
        line.set_data([], [])
        return line,

    def animation_function(i):
        idx = idxs[i]
        line.set_data(data.experiments[n_run].rrt_experiment.paths[idx][:, 0], data.experiments[n_run].rrt_experiment.paths[idx][:, 1])
        return line,

    ani = animation.FuncAnimation(fig, animation_function, init_func=init_function, frames=len(idxs), interval=500, blit=True, repeat_delay=5000)
    ani.save(os.path.join(destination_folder, file_name + "_" + str(n_run) + ".gif"))


def check_trivial_path_cost_validity(data_list: list[exp_data_classes.ExperimentBatch]):
    trivial_path_cost = None
    do_comparison = len(data_list) > 1
    if do_comparison:
        trivial_path_cost = data_list[0].trivial_path_cost
        for experiment in data_list:
            if experiment.trivial_path_cost != trivial_path_cost:
                trivial_path_cost = None
                break
    else:
        if data_list[0].trivial_path_cost > 0.0:
            trivial_path_cost = data_list[0].trivial_path_cost

    return trivial_path_cost


def check_optimal_path_cost_validity(data_list: list[exp_data_classes.ExperimentBatch]):
    optimal_path_cost = None
    do_comparison = len(data_list) > 1

    if data_list[0].optimal_rov_path_cost > 0.0:
        optimal_path_cost = data_list[0].optimal_rov_path_cost
    else:
        optimal_path_cost = np.nanmin(data_list[0].mean_rrt_travel_cost)

    if do_comparison:
        for experiment in data_list:
            if experiment.optimal_rov_path_cost > 0.0:
                experiment_optimal_path_cost = experiment.optimal_rov_path_cost
            else:
                experiment_optimal_path_cost = np.nanmin(experiment.mean_rrt_travel_cost)

            if experiment_optimal_path_cost != optimal_path_cost:
                optimal_path_cost = None
                break

    return optimal_path_cost


def check_title_validity(data_list: list[exp_data_classes.ExperimentBatch]):
    evaluator_name = data_list[0].ipp_gain_evaluator
    batch_name = data_list[0].batch_name
    if evaluator_name == "":
        return batch_name
    title = evaluator_name
    for experiment in data_list:
        if experiment.ipp_gain_evaluator != evaluator_name:
            title = batch_name
    return title
