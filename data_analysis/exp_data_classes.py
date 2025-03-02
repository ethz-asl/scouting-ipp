import io
import os
import re
import copy
import pandas as pd
import numpy as np
import yaml as YAML
from scipy.interpolate import interp1d
from dataclasses import dataclass, field

PRM_STAR_PLANNER_NAME="prm_star"
RRT_STAR_PLANNER_NAME="rrt_star"

@dataclass
class RRTExperimentData:
    map_name: list[str] = field(default_factory=list)
    travel_cost_rrt: list[int] = field(default_factory=list)
    map_directory: str = field(default_factory=str)
    success: list[bool] = field(default_factory=list)
    starts: list[np.ndarray] = field(default_factory=list)
    goals: list[np.ndarray] = field(default_factory=list)
    paths: list[np.ndarray] = field(default_factory=list)
    map_size: np.ndarray = field(default_factory=np.ndarray)

    def __getitem__(self, item):
        return dict({'MapName': self.map_name[item],
                     'TravelCost': self.travel_cost_rrt[item],
                     'Success': self.success[item],
                     'Start': self.starts[item],
                     'Goal': self.goals[item],
                     'Path': self.paths[item],
                     'MapSize': self.map_size})

    def __len__(self):
        return len(self.map_name)


@dataclass
class IPPExperimentData:
    map_name: list[str] = field(default_factory=list)
    ros_time: list[int] = field(default_factory=list)
    wall_time: list[int] = field(default_factory=list)
    n_map_data: list[int] = field(default_factory=list)
    cpu_time: list[int] = field(default_factory=list)
    travel_cost_ipp: list[int] = field(default_factory=list)
    travel_cost_type_ipp: str = field(default_factory=str)
    explored_area_ipp: list[int] = field(default_factory=list)
    exp_path: str = field(default_factory=str)
    map_directory: str = field(default_factory=str)
    data_file_path: str = field(default_factory=str)
    perf_log_file_path: str = field(default_factory=str)
    perf_log_found: bool = field(default=False)
    max_explored_area: float = field(default_factory=float)
    ipp_planner_config: dict = field(default_factory=dict)
    ipp_planner_bounds_config: dict = field(default_factory=dict)
    map_config: dict = field(default_factory=dict)
    trivial_path_cost: float = field(default_factory=float)
    ipp_gain_evaluator: str = field(default_factory=str)
    map_name_str: str = field(default_factory=str)

    def __init__(self, exp_path, map_data_file="map_data.csv", map_dir_name="maps", perf_log_dir="",
                 perf_log_file_name="performance_log.csv", ipp_planner_config_path="",
                 ipp_planner_bounds_config_path="", map_config_file_path="", evaluator_config_path=""):

        self.map_name = []
        self.ros_time = []
        self.wall_time = []
        self.n_map_data = []
        self.cpu_time = []
        self.travel_cost_ipp = []
        self.travel_cost_type_ipp = ""
        self.explored_area_ipp = []
        self.exp_path = ""
        self.map_directory = ""
        self.data_file_path = ""
        self.perf_log_file_path = ""
        self.perf_log_found = False
        self.max_explored_area = 0.0
        self.trivial_path_cost = 0.0
        self.ipp_gain_evaluator = ""
        self.map_name_str = ""

        self.exp_path = exp_path
        if not os.path.isdir(self.exp_path):
            print("Directory not found: " + self.exp_path)
            raise NotADirectoryError

        self.data_file_path = os.path.join(self.exp_path, map_data_file)
        if not os.path.isfile(self.data_file_path):
            print("File not found: " + self.data_file_path)
            raise FileNotFoundError
        try:
            data_frame = pd.read_csv(self.data_file_path, header=0, quotechar='|', skiprows=[1])
        except Exception as inst:
            print("File not readable: " + self.data_file_path)
            raise inst

        self.map_directory = os.path.join(self.exp_path, map_dir_name)
        if not os.path.isdir(self.map_directory):
            print("Directory not found: " + self.map_directory)
            raise NotADirectoryError

        try:
            self.perf_log_file_path = os.path.join(self.exp_path, perf_log_dir)
            if not os.path.isdir(self.perf_log_file_path):
                print("Directory not found: " + self.perf_log_file_path)
                raise NotADirectoryError
            self.perf_log_file_path = os.path.join(self.perf_log_file_path, perf_log_file_name)
            if not os.path.isfile(self.perf_log_file_path):
                print("File not found: " + self.perf_log_file_path)
                raise FileNotFoundError
            self.perf_log_found = True
        except:
            print("Continuing without performance log file!!")

        self.map_name = list(data_frame['MapName'])
        self.ros_time = list(data_frame['RosTime'])
        self.wall_time = list(data_frame['WallTime'])
        self.n_map_data = list(data_frame['NMapData'])
        self.cpu_time = list(data_frame['CPUTime'])
        self.travel_cost_ipp = list(data_frame['TravelCost'])
        self.travel_cost_type_ipp = list(data_frame['CostType'])[-1]
        self.explored_area_ipp = list(data_frame['ExploredArea'])

        max_exp_areas = list(data_frame['ExplorableArea'])
        if not (max_exp_areas.count(max_exp_areas[0]) == len(max_exp_areas)):
            print("Something seems odd. Not all explorable areas are equal!")
        self.max_explored_area = max_exp_areas[len(max_exp_areas) - 1]

        self.ipp_planner_config = self.read_yaml_config_file(ipp_planner_config_path)
        self.ipp_planner_bounds_config = self.read_yaml_config_file(ipp_planner_bounds_config_path)
        self.map_config = self.read_yaml_config_file(map_config_file_path)
        evaluator_config = self.read_yaml_config_file(evaluator_config_path)
        if evaluator_config != {}:
            self.ipp_planner_config = self.merge_configs(self.ipp_planner_config, [evaluator_config])

        self.compute_trivial_path_cost()
        self.ipp_gain_evaluator = self.convert_evaluator_name(
            self.get_config_element(self.ipp_planner_config, ["trajectory_evaluator", "following_evaluator", "type"]))
        self.map_name_str = self.convert_map_name(self.get_config_element(self.map_config, ["name"]))
        if self.map_name_str == "" and len(self.map_name) > 0:
            self.map_name_str = self.convert_map_name(self.map_name[0])

    def __getitem__(self, item):
        return dict({'MapName': self.map_name[item],
                     'RosTime': self.ros_time[item],
                     'WallTime': self.wall_time[item],
                     'NMapData': self.n_map_data[item],
                     'CPUTime': self.cpu_time[item],
                     'TravelCost': self.travel_cost_ipp[item],
                     'CostType': self.travel_cost_type_ipp,
                     'ExploredArea': self.explored_area_ipp[item]})

    def __len__(self):
        return len(self.map_name)

    def read_yaml_config_file(self, path):
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

    def merge_configs(self, main_config: dict, configs: list, prefixes=None, prioritize_new: bool = True):
        if prefixes is None:
            prefixes = ["" for _ in configs]
        if len(prefixes) != len(configs):
            return {}
        new_config = copy.deepcopy(main_config)
        for config, prefix in zip(configs, prefixes):
            if prefix == "":
                for key in config.keys():
                    if key in new_config.keys():
                        if type(new_config[key]) is dict:
                            new_config[key] = self.merge_configs(new_config[key], [config[key]])
                        else:
                            if prioritize_new:
                                new_config[key] = config[key]
                            else:
                                pass
                    else:
                        new_config[key] = config[key]
            else:
                if prefix in new_config.keys():
                    new_config[prefix] = self.merge_configs(new_config[prefix], [config])
                else:
                    new_config[prefix] = config

        return new_config

    def get_config_element(self, config: dict, keys: list[str]):

        if config != {}:
            try:
                result = config
                for key in keys:
                    result = result[key]
                return result
            except:
                return ""
        else:
            return ""

    def compute_trivial_path_cost(self):
        try:
            a_x = self.ipp_planner_config["trajectory_evaluator"]["following_evaluator"]["sensor_model"]["resolution_x"]
            a_y = self.ipp_planner_config["trajectory_evaluator"]["following_evaluator"]["sensor_model"]["resolution_y"]
            if a_x != a_y or a_x <= 0 or a_y <= 0:
                print("The config file seems invalid, because the field of view of the camera is not a square.")
                raise ValueError

            width = self.map_config["width"]
            height = self.map_config["height"]
            resolution = self.map_config["resolution"]
            a = width * a_x / resolution[0]

            if "map_bounding_volume" in self.ipp_planner_bounds_config:
                x_min = self.ipp_planner_bounds_config["map_bounding_volume"]["x_min"]
                x_max = self.ipp_planner_bounds_config["map_bounding_volume"]["x_max"]
                y_min = self.ipp_planner_bounds_config["map_bounding_volume"]["y_min"]
                y_max = self.ipp_planner_bounds_config["map_bounding_volume"]["y_max"]
            else:
                x_min = self.ipp_planner_config["map_bounding_volume"]["x_min"]
                x_max = self.ipp_planner_config["map_bounding_volume"]["x_max"]
                y_min = self.ipp_planner_config["map_bounding_volume"]["y_min"]
                y_max = self.ipp_planner_config["map_bounding_volume"]["y_max"]

            x = x_max - x_min
            y = y_max - y_min

            if x <= 0 or y <= 0:
                print("The config file seems invalid, because the map area appears to be invalid.")
                raise ValueError
            s1 = (x - a) * np.ceil(y / a) + (y - a)
            s2 = (y - a) * np.ceil(x / a) + (x - a)

            trivial_path_length = min(s1, s2)
        except:
            trivial_path_length = 0.0

        if self.travel_cost_type_ipp == "time":
            try:
                v_max = self.ipp_planner_bounds_config["planner"]["system_constraints"]["v_max"]
                self.trivial_path_cost = trivial_path_length * v_max
            except:
                try:
                    v_max = self.ipp_planner_config["system_constraints"]["v_max"]
                    self.trivial_path_cost = trivial_path_length * v_max
                except:
                    self.trivial_path_cost = 0.0
        elif self.travel_cost_type_ipp == "distance":
            self.trivial_path_cost = trivial_path_length
        elif self.travel_cost_type_ipp == "map":
            self.trivial_path_cost = 0.0
        else:
            print("Unknown cost type encountered!")
            self.trivial_path_cost = 0.0

    def convert_evaluator_name(self, name):
        if name == "RRTStarEvaluator":
            return "RRT*"
        elif name == "ExpandReachableAreaEvaluator":
            return "Expand Reachable Area"
        elif name == "AverageViewCostEvaluator":
            return "Average View Cost"
        elif name == "ExpandLowCostAreaEvaluator":
            return "Expand Low Cost Area"
        elif name == "GoalDistanceEvaluator":
            return "Goal Distance"
        elif name == "NaiveEvaluator":
            return "New Pixel Count"
        name = re.sub(r'\B([a-z](?=[A-Z])|[A-Z](?=[A-Z][a-z]))', r'\1 ', name)
        return name.replace(" Evaluator", "")

    def convert_map_name(self, name):
        if type(name) is list:
            return self.convert_map_name(name[0])
        if "." in name:
            name = name.split(".")[0]
        if re.match(r"map_[0-9]+", name):
            number = int(name.split("_")[-1])
            return "Map " + str(number)
        if type(name) is int:
            return "Map " + str(name)
        return name


class PlanningExperiment:
    ipp_experiment: IPPExperimentData
    rrt_experiment: RRTExperimentData
    min_travel_cost_ipp: float
    max_travel_cost_ipp: float
    min_travel_cost_rrt: float
    max_travel_cost_rrt: float

    def __init__(self, ipp_exp: IPPExperimentData, rrt_exp: RRTExperimentData):
        self.ipp_experiment = ipp_exp
        self.rrt_experiment = rrt_exp
        self.min_travel_cost_ipp = float(np.nanmin(self.ipp_experiment.travel_cost_ipp))
        self.max_travel_cost_ipp = float(np.nanmax(self.ipp_experiment.travel_cost_ipp))
        self.min_travel_cost_rrt = float(np.nanmin(self.rrt_experiment.travel_cost_rrt))
        self.max_travel_cost_rrt = float(np.nanmax(self.rrt_experiment.travel_cost_rrt))

    def __len__(self):
        return len(self.ipp_experiment)


class ExperimentBatch:
    batch_name: str = ""
    evaluation_planner: str = ""
    map_config_file: str = ""
    ipp_gain_evaluator: str = ""
    map_name: str = ""
    map_size: np.ndarray = np.ndarray([])
    min_travel_cost_ipp: float = 0.0
    max_travel_cost_ipp: float = 0.0
    min_travel_cost_rrt: float = 0.0
    max_travel_cost_rrt: float = 0.0
    trivial_path_cost: float = 0.0
    optimal_rov_path_cost: float = 0.0
    experiments: list[PlanningExperiment] = []
    travel_costs_ipp: np.ndarray = np.ndarray([])
    interpolated_travel_cost_rrt: np.ndarray = np.ndarray([])
    interpolated_coverage_ipp: np.ndarray = np.ndarray([])
    interpolated_time: np.ndarray = np.ndarray([])
    mean_rrt_travel_cost: np.ndarray = np.ndarray([])
    stddev_rrt_travel_cost: np.ndarray = np.ndarray([])
    mean_ipp_coverage: np.ndarray = np.ndarray([])
    stddev_ipp_coverage: np.ndarray = np.ndarray([])
    success_rate: np.ndarray = np.ndarray([])
    plotting_mean_rrt_travel_cost: np.ndarray = np.ndarray([])
    plotting_stddev_rrt_travel_cost: np.ndarray = np.ndarray([])

    def __init__(self, experiment=None, batch_name=None, map_config_file=None, ipp_planner_config=None, planner_type=PRM_STAR_PLANNER_NAME):

        self.experiments = []
        self.travel_costs_ipp = np.ndarray([])
        self.interpolated_travel_cost_rrt = np.ndarray([])
        self.interpolated_coverage_ipp = np.ndarray([])
        self.interpolated_time = np.ndarray([])
        self.mean_rrt_travel_cost = np.ndarray([])
        self.stddev_rrt_travel_cost = np.ndarray([])
        self.mean_ipp_coverage = np.ndarray([])
        self.stddev_ipp_coverage = np.ndarray([])
        self.success_rate = np.ndarray([])
        self.map_size = np.ndarray([])

        self.min_travel_cost_ipp = np.inf
        self.max_travel_cost_ipp = -np.inf
        self.min_travel_cost_rrt = np.inf
        self.max_travel_cost_rrt = -np.inf
        self.trivial_path_cost = 0.0
        self.optimal_rov_path_cost = -1.0

        if experiment is not None and type(experiment) is PlanningExperiment:
            self.add_experiment(experiment)

        if batch_name is not None and type(batch_name) is str:
            self.batch_name = batch_name
        
        if planner_type is not None and type(planner_type) is str:
            self.evaluation_planner = planner_type

        if map_config_file is not None and type(map_config_file) is str:
            self.map_config_file = map_config_file

    def __len__(self):
        return len(self.experiments)

    def add_experiment(self, experiment: PlanningExperiment):
        self.experiments.append(experiment)
        self.map_size = experiment.rrt_experiment.map_size

        try:
            if self.evaluation_planner == PRM_STAR_PLANNER_NAME:
                optimal_rov_path_cost = experiment.ipp_experiment.map_config["optimal_prm_rov_path_cost"]
            elif self.evaluation_planner == RRT_STAR_PLANNER_NAME:
                optimal_rov_path_cost = experiment.ipp_experiment.map_config["optimal_rrt_rov_path_cost"]
            else:
                raise KeyError("")
        except KeyError:
            optimal_rov_path_cost = -1.0

        if len(self) > 1:
            if self.trivial_path_cost != experiment.ipp_experiment.trivial_path_cost:
                self.trivial_path_cost = 0.0
            if self.ipp_gain_evaluator != experiment.ipp_experiment.ipp_gain_evaluator:
                self.ipp_gain_evaluator = ""
            if self.map_name != experiment.ipp_experiment.map_name_str:
                self.ipp_gain_evaluator = ""
            if self.optimal_rov_path_cost != optimal_rov_path_cost:
                self.optimal_rov_path_cost = -1.0
        else:
            self.trivial_path_cost = experiment.ipp_experiment.trivial_path_cost
            self.ipp_gain_evaluator = experiment.ipp_experiment.ipp_gain_evaluator
            self.map_name = experiment.ipp_experiment.map_name_str
            self.optimal_rov_path_cost = optimal_rov_path_cost

    def compute_min_max_travel_cost_ipp(self):
        self.min_travel_cost_ipp = np.inf
        self.max_travel_cost_ipp = -np.inf
        self.min_travel_cost_rrt = np.inf
        self.max_travel_cost_rrt = -np.inf
        for experiment in self.experiments:
            if experiment.min_travel_cost_ipp < self.min_travel_cost_ipp:
                self.min_travel_cost_ipp = experiment.min_travel_cost_ipp
            if experiment.max_travel_cost_ipp > self.max_travel_cost_ipp:
                self.max_travel_cost_ipp = experiment.max_travel_cost_ipp
            if experiment.min_travel_cost_rrt < self.min_travel_cost_rrt:
                self.min_travel_cost_rrt = experiment.min_travel_cost_rrt
            if experiment.max_travel_cost_rrt > self.max_travel_cost_rrt:
                self.max_travel_cost_rrt = experiment.max_travel_cost_rrt

    def set_batch_name(self, batch_name):
        self.batch_name = batch_name

    def set_map_config_file(self, map_config_file):
        self.map_config_file = map_config_file

    def get_cost_unit(self):
        cost_type = self.experiments[0].ipp_experiment.travel_cost_type_ipp
        consistent = True
        for experiment in self.experiments:
            consistent &= cost_type == experiment.ipp_experiment.travel_cost_type_ipp

        if consistent:
            if cost_type == "time":
                return "s"
            elif cost_type == "distance":
                return "m"
            elif cost_type == "map":
                return "1"
            else:
                print("Cost type is unknown!")
                return "1"
        else:
            print("Cost type is inconsistent!")
            return "1"

    def __getitem__(self, item):
        return self.experiments[item]

    def compute_spacing_and_ranges(self):
        num_el = 200

        self.compute_min_max_travel_cost_ipp()

        delta = (self.max_travel_cost_ipp - self.min_travel_cost_ipp) / (num_el + 1)
        data_points = [self.min_travel_cost_ipp + delta * (i + 0.5) for i in range(num_el)]

        self.travel_costs_ipp = np.array(data_points)

    def interpolate_data(self):
        self.compute_spacing_and_ranges()
        for i, experiment in enumerate(self.experiments):
            if len(experiment.ipp_experiment.travel_cost_ipp) > 1:
                interp_func = interp1d(experiment.ipp_experiment.travel_cost_ipp,
                                       experiment.rrt_experiment.travel_cost_rrt,
                                       kind="linear", bounds_error=False,
                                       fill_value=(np.nan, np.nanmin(experiment.rrt_experiment.travel_cost_rrt)))
                interpolated_data = interp_func(self.travel_costs_ipp)
            else:
                interpolated_data = [np.nan for _ in range(len(self.travel_costs_ipp))]
            if self.interpolated_travel_cost_rrt.size == 1:
                self.interpolated_travel_cost_rrt = np.array(interpolated_data)
            else:
                self.interpolated_travel_cost_rrt = np.vstack(
                    [self.interpolated_travel_cost_rrt, np.array(interpolated_data)])

    def interpolate_time(self):
        self.compute_spacing_and_ranges()
        for i, experiment in enumerate(self.experiments):
            if len(experiment.ipp_experiment.travel_cost_ipp) > 1:
                interp_func = interp1d(experiment.ipp_experiment.travel_cost_ipp,
                                       experiment.ipp_experiment.wall_time,
                                       kind="linear", bounds_error=False,
                                       fill_value=(np.nan, np.nanmax(experiment.ipp_experiment.wall_time)))
                interpolated_data = interp_func(self.travel_costs_ipp)
            else:
                interpolated_data = [np.nan for _ in range(len(self.travel_costs_ipp))]
            if self.interpolated_time.size == 1:
                self.interpolated_time = np.array(interpolated_data)
            else:
                self.interpolated_time = np.vstack(
                    [self.interpolated_time, np.array(interpolated_data)])

    def compute_mean_and_stddev(self):
        self.mean_rrt_travel_cost = np.nanmean(self.interpolated_travel_cost_rrt, axis=0)
        self.stddev_rrt_travel_cost = np.nanstd(self.interpolated_travel_cost_rrt, axis=0)

    def compute_success_rate(self):
        self.success_rate = (len(self) - np.count_nonzero(np.isnan(self.interpolated_travel_cost_rrt), axis=0)) / len(
            self)

    def compute_coverage(self):
        self.compute_spacing_and_ranges()
        for i, experiment in enumerate(self.experiments):
            if len(experiment.ipp_experiment.travel_cost_ipp) > 1:
                interp_func = interp1d(experiment.ipp_experiment.travel_cost_ipp,
                                       experiment.ipp_experiment.explored_area_ipp,
                                       kind="linear", bounds_error=False,
                                       fill_value=(np.nan, np.nanmax(experiment.ipp_experiment.explored_area_ipp)))
                interpolated_data = interp_func(self.travel_costs_ipp)
            else:
                interpolated_data = [np.nan for _ in range(len(self.travel_costs_ipp))]

            interpolated_data = np.array(interpolated_data)

            if experiment.ipp_experiment.max_explored_area > 0:
                interpolated_data = 100 * interpolated_data / experiment.ipp_experiment.max_explored_area

            if self.interpolated_coverage_ipp.size == 1:
                self.interpolated_coverage_ipp = interpolated_data
            else:
                self.interpolated_coverage_ipp = np.vstack(
                    [self.interpolated_coverage_ipp, interpolated_data])
        self.mean_ipp_coverage = np.nanmean(self.interpolated_coverage_ipp, axis=0)
        self.stddev_ipp_coverage = np.nanstd(self.interpolated_coverage_ipp, axis=0)

    def compute_plotting_mean_and_stddev(self):
        data = np.array([])
        if len(self.interpolated_travel_cost_rrt.shape) >= 2:
            num_iterations = self.interpolated_travel_cost_rrt.shape[0]
        else:
            num_iterations = 1
        for i in range(num_iterations):
            if num_iterations == 1:
                data_line = copy.deepcopy(self.interpolated_travel_cost_rrt)
            else:
                data_line = copy.deepcopy(self.interpolated_travel_cost_rrt[i, :])
            max_data_line = np.nanmax(data_line)
            for j in range(len(data_line)):
                if self.success_rate[j] == 0:
                    continue
                if np.isnan(data_line[j]):
                    data_line[j] = max_data_line  #TODO: Check which max to use for filling.
                else:
                    break
            if data.size == 0:
                data = data_line
            else:
                data = np.vstack(
                    [data, data_line])
        self.plotting_mean_rrt_travel_cost = np.nanmean(data, axis=0)
        self.plotting_stddev_rrt_travel_cost = np.nanstd(data, axis=0)

    def evaluate_data(self):
        #TODO: Find better way to handle one run experiments.
        duplicated = False
        if len(self) == 1:
            self.add_experiment(self.experiments[0])
            duplicated = True
        self.interpolate_data()
        self.interpolate_time()
        self.compute_mean_and_stddev()
        self.compute_success_rate()
        self.compute_coverage()
        self.compute_plotting_mean_and_stddev()
        if duplicated:
            experiment = self.experiments[0]
            self.experiments.clear()
            self.add_experiment(experiment)

