import os
import copy
import cv2
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation


def parse_arguments():
    parser = argparse.ArgumentParser(description='Visualizes the rover path estimate evolution.')
    parser.add_argument('--dest', dest='destination_folder', type=str, default=None, required=False,
                        help='output folder where plots shall be stored')
    parser.add_argument('--size', dest='map_size', nargs=2, type=float,
                        help='size of map in meters [width x height]', default=[40.0, 30.0])
    parser.add_argument('--map', dest='map_path', type=str, default=None,
                       help='path of map visualization file')
    parser.add_argument('data_folder', type=str,
                        help='output folder of the logged data')

    args = parser.parse_args()
    return args


def plot_path_evolution(data_folder, destination_folder=None, map_size=None, map_path=None, use_evaluator_data=False):
    if destination_folder == "" or destination_folder is None:
        destination_folder = "/tmp/"
    if map_size is None:
        map_size = [40.0, 30.0]

    x_coordinates, y_coordinates = load_paths_from_folder(data_folder)

    start_circle = plt.Circle((x_coordinates[0][0],
                               y_coordinates[0][0]),
                              map_size[1] / 50, facecolor='g', edgecolor='w')
    goal_circle = plt.Circle((x_coordinates[0][-1],
                              y_coordinates[0][-1]),
                             map_size[1] / 50, facecolor='r', edgecolor='w')

    fig, ax = plt.subplots()
    if map_path is not None and os.path.isfile(map_path) and map_path.split(".")[-1] == "png":
        img = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.flip(img, 0)
        ax.imshow(img, extent=[0, map_size[0], 0, map_size[1]])
    ax.add_patch(start_circle)
    ax.add_patch(goal_circle)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])

    line, = ax.plot([], [])

    #idxs = [i for i, path in enumerate(data.experiments[n_run].rrt_experiment.paths) if path.size != 0]
    idxs = [i for i in range(len(x_coordinates))]

    def init_function():
        line.set_data([], [])
        return line,

    def animation_function(i):
        idx = idxs[i]
        #line.set_data(data.experiments[n_run].rrt_experiment.paths[idx][:, 0], data.experiments[n_run].rrt_experiment.paths[idx][:, 1])
        line.set_data(x_coordinates[idx], y_coordinates[idx])
        return line,

    ani = animation.FuncAnimation(fig, animation_function, init_func=init_function, frames=len(idxs), interval=500,
                                  blit=True, repeat_delay=5000)
    #ani.save(os.path.join(destination_folder, data.batch_name + "_" + str(n_run) + ".gif"))
    ani.save(os.path.join(destination_folder, "test" + ".gif"))


def load_paths_from_folder(path):
    if not os.path.isdir(path):
        print("Could not find folder: " + path)
        exit(1)

    folder_content = os.listdir(path)
    path_files = []
    for file in folder_content:
        if os.path.isfile(os.path.join(path, file)) and ".csv" in file:
            path_files.append(file)

    path_files.sort()
    x_coordinates = []
    y_coordinates = []
    for path_file in path_files:
        new_data = pd.read_csv(os.path.join(path, path_file), header=0, skiprows=[1])
        if not ("PositionX" in new_data.columns and "PositionY" in new_data.columns):
            continue
        x_coordinates.append(new_data["PositionX"].to_numpy())
        y_coordinates.append(new_data["PositionY"].to_numpy())

    return x_coordinates, y_coordinates


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    plot_path_evolution(arguments.data_folder, arguments.destination_folder, arguments.map_size, arguments.map_path)