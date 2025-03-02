import os
import errno
import pickle
import argparse
import subprocess
import exp_data_classes
import numpy as np
import pandas as pd


RRT_STAR_PLANNER_NAME="rrt_star"
PRM_STAR_PLANNER_NAME="prm_star"

def parse_arguments():
    parser = argparse.ArgumentParser(description='Process output of path planner for the MT of Friedrich M. Rockenbauer.')
    parser.add_argument('data_folder', type=str,
                        help='output folder of the path planner containing the logged data')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None,
                        help='output folder where evaluation data shall be stored', required=True)
    parser.add_argument('--name', dest='name', type=str, default="UNKNOWN",
                        help='name of the experiment batch', required=False)
    parser.add_argument('--map_config', dest='map_config', type=str, default="",
                        help='name of the used map configuration file', required=False)
    parser.add_argument('--planner_config', dest='ipp_planner_config', type=str, default="",
                        help='path to the configuration file for the used planner', required=False)
    parser.add_argument('--evaluator_config', dest='evaluator_config', type=str, default="",
                        help='path to the configuration file for the used evaluator', required=False)
    parser.add_argument('--planner_bounds_config', dest='ipp_planner_bounds_config', type=str, default="",
                        help='path to the bounds configuration file for the used planner', required=False)
    parser.add_argument('-l', dest='use_precomputed_data', action='store_true',
                        help='only load rrt star results and do not recompute them')
    parser.add_argument('--eval_planner', dest='evaluation_planner_type', type=str,
                        help='type of planner used as follower planner and for evaluation')

    args = parser.parse_args()
    return args


def run_eval_planner(planner_type=PRM_STAR_PLANNER_NAME, map_dir="~/data/maps", map_name="test_map_art", output_name="rrt_output.csv",
                 output_path="/tmp/test_out/", start=np.array([0.0,0.0]), goal=np.array([0.0,0.0]),
                 map_config_file_path="", path_folder_name="rrt_paths", use_precomputed_data=False):
    if type(map_name) is int:
        map_path = os.path.join(map_dir, str(map_name).zfill(5) + ".png")
    else:
        map_path = os.path.join(map_dir, str(map_name) + ".png")
    if not use_precomputed_data:
        ros_ws_dir = os.getenv("ROS_WS_DIR")
        if not map_config_file_path == "" or (goal == start).all():
            if (goal == start).all() and map_config_file_path == "":
                print("Goal equals start: " + str(start[0]) + ", " + str(start[1]))
            subprocess.call([ros_ws_dir + "/devel/lib/planexp_base_planners/planexp_eval_planner", "-m", map_path, "-o",
                             output_path, "-f", output_name, "-cm", map_config_file_path, "-t", planner_type], stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT)
        else:
            subprocess.call([ros_ws_dir + "/devel/lib/planexp_base_planners/planexp_eval_planner", "-m", map_path, "-o",
                             output_path, "-f", output_name, "-sx", str(start[0]), "-sy", str(start[1]), "-gx", str(goal[0]), "-gy", str(goal[1]), "-t", 
                             planner_type], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    successes, travel_costs, path, map_sizes, starts_rrt, goals_rrt = get_eval_planner_result(output_path, output_name, map_name, path_folder_name)
    for success, travel_cost, map_size, start_rrt, goal_rrt in zip(np.flip(successes), np.flip(travel_costs), np.flip(map_sizes, axis=0), np.flip(starts_rrt, axis=0), np.flip(goals_rrt, axis=0)):
        if success:
            return True, travel_cost, path, map_size, start_rrt, goal_rrt
    return False, np.nan, path, map_sizes[-1], starts_rrt[-1], goals_rrt[-1]


def get_eval_planner_result(output_path, output_file_name, map_name, path_folder_name):
    try:
        output_file = os.path.join(output_path, output_file_name)
        data = pd.read_csv(output_file, header=0, quotechar='|', skiprows=[1])
        map_names = list(data['MapName'])
        widths = list(data['Width'])
        heights = list(data['Height'])
        starts_x = list(data['StartX'])
        starts_y = list(data['StartY'])
        goals_x = list(data['GoalX'])
        goals_y = list(data['GoalY'])
        success = list(data['Success'])
        travel_costs = list(data['TravelCost'])

        if type(map_name) is int:
            map_file_name = str(map_name).zfill(5) + ".png"
        else:
            map_file_name = str(map_name) + ".png"

        indices = [i for i, x in enumerate(map_names) if x == map_file_name]
        matching_successes = [success[i] for i in indices]
        matching_travel_costs = [travel_costs[i] for i in indices]
        start = [np.array([starts_x[i], starts_y[i]]) for i in indices]
        goal =[np.array([goals_x[i], goals_y[i]])  for i in indices]
        width = [widths[i] for i in indices]
        height = [heights[i] for i in indices]

    except:
        print("Error while reading file: " + output_file)
        matching_successes = []
        matching_travel_costs = []
        width = [0.0]
        height = [0.0]
        start = [np.array([0.0, 0.0])]
        goal = [np.array([0.0, 0.0])]

    try:
        if type(map_name) is int:
            path_file_name = str(map_name).zfill(5) + "_path.csv"
        else:
            path_file_name = str(map_name) + "_path.csv"

        path_file = os.path.join(output_path, path_folder_name, path_file_name)
        if os.path.isfile(path_file):
            data = pd.read_csv(path_file, header=0, quotechar='|', skiprows=[1])
            position_x = list(data['PositionX'])
            position_y = list(data['PositionY'])
            path = np.array([position_x, position_y]).transpose()
            #location_x = list(data['LocationX'])
            #location_y = list(data['LocationY'])
        else:
            path = np.array([])

    except:
        print("Error while reading file: " + path_file)
        path = np.array([])
    return np.array(matching_successes), np.array(matching_travel_costs), path, [np.array([width[i], height[i]]) for i in range(len(width))], start, goal


def evaluate_single_experiment(exp_folder, map_folder_name="maps", start=np.array([0.0, 0.0]), goal=np.array([0.0, 0.0]),
                               map_config_file_path="", ipp_planner_config_path="", ipp_planner_bounds_config_path="",
                               evaluator_config_path="", use_precomputed_data=False, planner_type=PRM_STAR_PLANNER_NAME):
    travel_costs_rrt = []
    successes_rrt = []
    starts_rrt = []
    goals_rrt = []
    paths_rrt = []
    map_size_rrt = np.array([0.0, 0.0])

    ipp_experiment = exp_data_classes.IPPExperimentData(exp_folder, map_dir_name=map_folder_name,
                                                        ipp_planner_config_path=ipp_planner_config_path,
                                                        ipp_planner_bounds_config_path=ipp_planner_bounds_config_path,
                                                        map_config_file_path=map_config_file_path,
                                                        evaluator_config_path=evaluator_config_path)
    for map_name in ipp_experiment.map_name:
        single_success, single_travel_cost_rrt, path, map_size_rrt, start_rrt, goal_rrt = run_eval_planner(planner_type,
            ipp_experiment.map_directory, map_name,
            output_path=exp_folder, start=start, goal=goal,
            map_config_file_path=map_config_file_path,
            use_precomputed_data=use_precomputed_data)

        travel_costs_rrt.append(single_travel_cost_rrt)
        successes_rrt.append(single_success)
        starts_rrt.append(start_rrt)
        goals_rrt.append(goal_rrt)
        paths_rrt.append(path)

    rrt_experiment = exp_data_classes.RRTExperimentData(map_name=ipp_experiment.map_name,
                                                        travel_cost_rrt=travel_costs_rrt, success=successes_rrt,
                                                        starts=starts_rrt, goals=goals_rrt, paths=paths_rrt,
                                                        map_size=map_size_rrt)

    return exp_data_classes.PlanningExperiment(ipp_exp=ipp_experiment, rrt_exp=rrt_experiment)


def evaluate_experiment_batch(batch_name: str, results_folder: str, experiment_folders: list, start=np.array([0.0, 0.0]),
                              goal=np.array([0.0, 0.0]), map_config_file_path="", ipp_planner_config_path="",
                              ipp_planner_bounds_config_path="", evaluator_config_path="", use_precomputed_data=False, planner_type=PRM_STAR_PLANNER_NAME):

    batch = exp_data_classes.ExperimentBatch(batch_name=batch_name, planner_type=planner_type)
    if map_config_file_path != "":
        batch.set_map_config_file(map_config_file_path)

    for i, experiment_folder in enumerate(experiment_folders):
        print("Evaluation of run " + str(i+1) + " of " + str(len(experiment_folders)))
        experiment = evaluate_single_experiment(experiment_folder, start=start, goal=goal,
                                                map_config_file_path=map_config_file_path,
                                                ipp_planner_config_path=ipp_planner_config_path,
                                                ipp_planner_bounds_config_path=ipp_planner_bounds_config_path,
                                                evaluator_config_path=evaluator_config_path,
                                                use_precomputed_data=use_precomputed_data, 
                                                planner_type=planner_type)

        batch.add_experiment(experiment)

    with open(os.path.join(results_folder, batch_name + ".pickle"), 'wb') as handle:
        pickle.dump(batch, handle, protocol=pickle.HIGHEST_PROTOCOL)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    data_folder = arguments.data_folder
    batch_name = arguments.name
    results_folder = arguments.destination_folder
    map_config_path = arguments.map_config
    ipp_planner_config_path = arguments.ipp_planner_config
    ipp_planner_bounds_config_path = arguments.ipp_planner_bounds_config
    evaluator_config_path = arguments.evaluator_config
    evaluation_planner_type = arguments.evaluation_planner_type.lower()
    experiment_folders = []

    try:
        if os.path.isdir(data_folder):
            folders = os.listdir(data_folder)
            for folder in folders:
                path_to_test = os.path.join(data_folder, folder)
                if os.path.isdir(path_to_test) and "run_" in folder:
                    experiment_folders.append(path_to_test)
            if not experiment_folders:
                experiment_folders.append(data_folder)
        else:
            print("The specified data directory can not be found: " + data_folder)
            raise NotADirectoryError
        if not os.path.isdir(results_folder):
            print("The specified results directory can not be found: " + results_folder)
            raise NotADirectoryError
    except NotADirectoryError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during folder setup.")
        exit(1)

    use_map_config_file = False
    if map_config_path != "":
        try:
            if os.path.isfile(map_config_path) and ".yaml" in map_config_path:
                use_map_config_file = True
            else:
                print("The specified map config file can not be found: " + map_config_path)
                raise FileNotFoundError
        except FileNotFoundError:
            use_map_config_file = False
        except:
            print("Unknown error occurred during read setup.")
            exit(1)
    
    if evaluation_planner_type == PRM_STAR_PLANNER_NAME: 
        planner_type = PRM_STAR_PLANNER_NAME
    elif evaluation_planner_type == RRT_STAR_PLANNER_NAME:
        planner_type = RRT_STAR_PLANNER_NAME
    else: 
        print("Planner type ", evaluation_planner_type, " is unknown!")
        exit(1)
        

    if use_map_config_file:
        evaluate_experiment_batch(batch_name, results_folder, experiment_folders, map_config_file_path=map_config_path,
                                  ipp_planner_config_path=ipp_planner_config_path,
                                  ipp_planner_bounds_config_path=ipp_planner_bounds_config_path,
                                  evaluator_config_path=evaluator_config_path,
                                  use_precomputed_data=arguments.use_precomputed_data, planner_type=planner_type)
    else:
        evaluate_experiment_batch(batch_name, results_folder, experiment_folders, start=np.array([0.5, 0.5]),
                                  goal=np.array([39.5, 29.5]), ipp_planner_config_path=ipp_planner_config_path,
                                  ipp_planner_bounds_config_path=ipp_planner_bounds_config_path,
                                  evaluator_config_path=evaluator_config_path,
                                  use_precomputed_data=arguments.use_precomputed_data, planner_type=planner_type)
