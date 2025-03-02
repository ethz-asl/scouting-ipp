import os
import errno
import pickle
import exp_data_classes
import numpy as np


def data_to_dict(name: str, data):
    data_dict = {name: data}
    return data_dict


def compute_optimal_travel_cost(data: exp_data_classes.ExperimentBatch):
    optimal_travel_cost = data.optimal_rov_path_cost
    if not optimal_travel_cost > 0.0:
        optimal_travel_cost = data.min_travel_cost_rrt
    return optimal_travel_cost


def compute_first_path_indices(data: exp_data_classes.ExperimentBatch):
    indices = []
    rrt_travel_costs = data.interpolated_travel_cost_rrt
    for i in range(rrt_travel_costs.shape[0]):
        for j, cost in enumerate(rrt_travel_costs[i, :]):
            if cost is not None:
                if not np.isnan(cost):
                    indices.append(j)
                    break
        if len(indices) < i + 1:
            indices.append(np.nan)
    return indices


def compute_best_path_indices(data: exp_data_classes.ExperimentBatch, percentage: float):
    optimal_travel_cost = compute_optimal_travel_cost(data)
    threshold = optimal_travel_cost * (1.0 + percentage)
    indices = []
    rrt_travel_costs = data.interpolated_travel_cost_rrt
    for i in range(rrt_travel_costs.shape[0]):
        for j, cost in enumerate(rrt_travel_costs[i, :]):
            if cost is not None:
                if not np.isnan(cost) and cost <= threshold:
                    indices.append(j)
                    break
        if len(indices) < i + 1:
            indices.append(np.nan)
    return indices


def first_path_cost(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    indices = compute_first_path_indices(data)
    first_path_costs = []
    rrt_travel_costs = data.interpolated_travel_cost_rrt
    for i, index in enumerate(indices):
        if np.isnan(index):
            first_path_costs.append(np.nan)
        else:
            first_path_costs.append(rrt_travel_costs[i, index])
    if result_dict is not None:
        result_dict["first_path_cost"] = first_path_costs
    return first_path_costs


def normalized_path_cost_difference(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    optimal_travel_cost = compute_optimal_travel_cost(data)
    first_path_costs = first_path_cost(data)
    normalized_path_cost_differences = []
    for cost in first_path_costs:
        normalized_path_cost_differences.append((cost / optimal_travel_cost) - 1.0)
    if result_dict is not None:
        result_dict["normalized_path_cost_difference"] = normalized_path_cost_differences
    return normalized_path_cost_differences


def first_path_coverage(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    indices = compute_first_path_indices(data)
    coverages = []
    ipp_coverages = data.interpolated_coverage_ipp
    for i, index in enumerate(indices):
        if np.isnan(index):
            coverages.append(np.nan)
        else:
            coverages.append(ipp_coverages[i, index])
    if result_dict is not None:
        result_dict["first_path_coverage"] = coverages
    return coverages


def best_path_coverage(data: exp_data_classes.ExperimentBatch, percentage: float, result_dict: dict = None):
    indices = compute_best_path_indices(data, percentage)
    coverages = []
    ipp_coverages = data.interpolated_coverage_ipp
    for i, index in enumerate(indices):
        if np.isnan(index):
            coverages.append(np.nan)
        else:
            coverages.append(ipp_coverages[i, index])
    name_str = "best_path_coverage_" + str(int(percentage * 100)) + "_" + \
               str(int(percentage * 1000) - 10 * int(percentage * 100))
    if result_dict is not None:
        result_dict[name_str] = coverages
    return coverages


def time_to_first_path(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    indices = compute_first_path_indices(data)
    times = []
    wall_times = data.interpolated_time
    for i, index in enumerate(indices):
        if np.isnan(index):
            times.append(np.nan)
        else:
            times.append(wall_times[i, index])
    if result_dict is not None:
        result_dict["time_to_first_path"] = times
    return times


def time_to_best_path(data: exp_data_classes.ExperimentBatch, percentage: float, result_dict: dict = None):
    indices = compute_best_path_indices(data, percentage)
    times = []
    wall_times = data.interpolated_time
    for i, index in enumerate(indices):
        if np.isnan(index):
            times.append(np.nan)
        else:
            times.append(wall_times[i, index])
    name_str = "time_to_best_path_" + str(int(percentage * 100)) + "_" + \
               str(int(percentage * 1000) - 10 * int(percentage * 100))
    if result_dict is not None:
        result_dict[name_str] = times
    return times

def time_to_termination(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    times = []
    wall_times = data.interpolated_time
    for i in range(len(data.experiments)):
        position = len(wall_times[i, :])-1
        if np.isnan(data.interpolated_travel_cost_rrt[i, position]): 
            times.append(np.nan)
        else: 
            times.append(wall_times[i, position])
    if result_dict is not None:
        result_dict["time_to_termination"] = times
    return times


def ipp_cost_to_best_path(data: exp_data_classes.ExperimentBatch, percentage: float, result_dict: dict = None):
    indices = compute_best_path_indices(data, percentage)
    costs = []
    ipp_costs = data.travel_costs_ipp
    for i, index in enumerate(indices):
        if np.isnan(index):
            costs.append(np.nan)
        else:
            costs.append(ipp_costs[index])
    name_str = "ipp_cost_to_best_path_" + str(int(percentage * 100)) + "_" + \
               str(int(percentage * 1000) - 10 * int(percentage * 100))
    if result_dict is not None:
        result_dict[name_str] = costs
    return costs


def ipp_cost_to_first_path(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    indices = compute_first_path_indices(data)
    costs = []
    ipp_costs = data.travel_costs_ipp
    for i, index in enumerate(indices):
        if np.isnan(index):
            costs.append(np.nan)
        else:
            costs.append(ipp_costs[index])
    if result_dict is not None:
        result_dict["ipp_cost_to_first_path"] = costs
    return costs

def ipp_cost_to_termination(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    ipp_costs = [np.nan if np.isnan(exp.min_travel_cost_rrt) else exp.max_travel_cost_ipp for exp in data.experiments]
    if result_dict is not None:
        result_dict["ipp_cost_to_termination"] = ipp_costs
    return ipp_costs

def coverage_at_termination(data: exp_data_classes.ExperimentBatch, result_dict: dict = None):
    terminal_coverages = []
    coverages = data.interpolated_coverage_ipp
    for i in range(len(data.experiments)):
        position = len(coverages[i, :])-1
        if np.isnan(data.interpolated_travel_cost_rrt[i, position]): 
            terminal_coverages.append(np.nan)
        else: 
            terminal_coverages.append(coverages[i, position])
    if result_dict is not None:
        result_dict["coverage_at_termination"] = terminal_coverages
    return terminal_coverages