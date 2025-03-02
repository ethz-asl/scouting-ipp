#TODO: Plese clean me up!!!
import os
import cv2
import argparse
import pandas as pd
import numpy as np
import copy
from tqdm import tqdm
os_version = os.popen("lsb_release -r | cut -f2").read().replace("\n", "")
if os_version == "18.04":
    import matplotlib
    matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from map_conversion_functions import label2color, color2label


def parse_arguments():
    parser = argparse.ArgumentParser(description='Replace labels in a given image.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('-o', dest="occupancy", type=str,
                        help='input file (map) or folder')

    args = parser.parse_args()
    return args

def read_path_file(file_path):
    data = pd.read_csv(file_path, header=0, quotechar='|', skiprows=[1])
    try:
        timestamps = data["Time"].to_numpy(float)
        timestamps -= timestamps[0]
        timestamps /= 1.0*1000000000
    except:
        timestamps = np.array([])
    try:
        position_x = data["PositionX"].to_numpy(float)
    except:
        position_x = np.array([])
    try:
        position_y = data["PositionY"].to_numpy(float)
    except:
        position_y = np.array([])
    try:
        location_x = data["LocationX"].to_numpy(int)
    except:
        location_x = np.array([])
    try:
        location_y = data["LocationY"].to_numpy(int)
    except:
        location_y = np.array([])

    return timestamps, position_x, position_y, location_x, location_y


def paste_occupancy_map(occupancy_map, image, color=(128, 128, 128)):
    if len(occupancy_map.shape) == 3:
        new_image = copy.deepcopy(image)
        for i in range(new_image.shape[1]):
            for j in range(new_image.shape[0]):
                if (occupancy_map[j, i] == (254, 254, 254)).all():
                    new_image[j, i] = color
    else:
        new_image = copy.deepcopy(image)
        for i in range(new_image.shape[1]):
            for j in range(new_image.shape[0]):
                if occupancy_map[j, i] == 1:
                    new_image[j, i] = color
    return new_image


def read_map_data(file_path):
    data_frame = pd.read_csv(file_path, header=0, quotechar='|', skiprows=[1])
    map_name = list(data_frame['MapName'])
    ros_time = list(data_frame['RosTime'])
    wall_time = list(data_frame['WallTime'])
    n_map_data = list(data_frame['NMapData'])
    cpu_time = list(data_frame['CPUTime'])
    travel_cost_ipp = list(data_frame['TravelCost'])
    travel_cost_type_ipp = list(data_frame['CostType'])[-1]
    explored_area_ipp = list(data_frame['ExploredArea'])
    return map_name, ros_time


def read_optimal_path(file_path):
    data_frame = pd.read_csv(file_path, header=0, quotechar='|', skiprows=[1])
    try:
        location_x = data_frame["LocationX"].to_numpy(int)
    except:
        location_x = np.array([])
    try:
        location_y = data_frame["LocationY"].to_numpy(int)
    except:
        location_y = np.array([])

    locations = []
    for loc_x, loc_y in zip(location_x, location_y):
        locations.append(np.array([loc_x, loc_y]))
    return locations


def position_to_location(position):
    m_per_pixel = 40.0 / 640.0
    location = np.floor(position/m_per_pixel)
    return location.astype('int')


def position_arrays_to_location_vector(file_name):
    timestamps, position_x, position_y, location_x, location_y = read_path_file(file_name)
    locations = []
    for i in range(len(position_x)):
        locations.append(position_to_location(np.array([position_x[i], position_y[i]])))
    return locations, timestamps


def find_closest_smaller_time(data, value):
    for i in range(len(data)):
        if value < data[i]:
            if i > 0:
                return i-1
            else:
                return i
    return len(data) - 1


def draw_path_in_map(map, path, color=(0, 255, 0), line_thickness=1):
    for i in range(1, len(path)):
        cv2.line(map, tuple(path[i-1]), tuple(path[i]), color, thickness=line_thickness)


def animate_frames(frames, save_gif=True, path='/tmp/test/animation.gif'):
    """ Animate frames from array (with ipython HTML extension for jupyter) and optionally save as gif.
    @param frames: list of frames
    @param jupyter: (bool), for using jupyter notebook extension to display animation
    @param save_gif: (bool), indicator if frames should be saved as gif
    @param path: path to save gif to
    """

    legend_elements = [Line2D([0], [0], color='w', lw=2, label='Optimal Rover Path'),
                       Line2D([0], [0], color=(1, 0, 1), lw=2, label='Rover Path Estimate'),
                       Line2D([0], [0], color=(0, 0, 1), lw=2, label='Scout Path'),
                       Patch(facecolor=(0.5,0.5,0.5), edgecolor=(0,0,0), label='Unknown Area')]

    fig, ax = plt.subplots(figsize=(12.8, 4.8), layout='constrained')
    plt.axis('off')
    cmap = None if len(frames[0].shape) == 3 else 'Greys'
    patch = plt.imshow(frames[0], cmap=cmap)
    ax.legend(handles=legend_elements, loc='lower left')
    anim = animation.FuncAnimation(plt.gcf(),
                                   lambda x: patch.set_data(frames[x]), frames=len(frames), interval=30)

    if save_gif:
        writer = animation.PillowWriter(fps=25)
        anim.save(path, writer=writer)

    plt.close()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    #arguments = parse_arguments()

    trajectory_file = "/home/rockenbf/data/automated_tests/ipp_2d/2022-11-28-06-56-19_e54717b537bc98ebd40fb1b41c5cae33ebb3781e/map_111/run_1/trajectory_file_mav_simulator.csv"
    vis_image_file = "/home/rockenbf/data/maps/visualization/map_121_vis.png"
    optimal_path_file = "/home/rockenbf/Downloads/map_data/rrt_paths/map_111_path.csv"
    evaluator_paths_dir = "/home/rockenbf/data/automated_tests/ipp_2d/2022-11-28-06-56-19_e54717b537bc98ebd40fb1b41c5cae33ebb3781e/map_111/run_1/evaluator_rrt_star_paths/"
    rrt_paths = "/home/rockenbf/data/automated_tests/ipp_2d/2022-11-28-06-56-19_e54717b537bc98ebd40fb1b41c5cae33ebb3781e/map_111/run_1/rrt_paths/"
    map_data_file = "/home/rockenbf/data/automated_tests/ipp_2d/2022-11-28-06-56-19_e54717b537bc98ebd40fb1b41c5cae33ebb3781e/map_111/run_1/map_data.csv"
    map_dir = "/home/rockenbf/data/automated_tests/ipp_2d/2022-11-28-06-56-19_e54717b537bc98ebd40fb1b41c5cae33ebb3781e/map_111/run_1/maps/"
    dest_folder = "/tmp/test/"

    map_data_names, map_data_times = read_map_data(map_data_file)
    trajectory_locations, trajectory_times = position_arrays_to_location_vector(trajectory_file)
    vis_image = cv2.imread(vis_image_file)

    evaluator_path_files = os.listdir(evaluator_paths_dir)
    evaluator_path_files.sort()

    evaluator_path_time_spacing = (map_data_times[-1] - map_data_times[0]) / len(evaluator_path_files)
    evaluator_path_times = [evaluator_path_time_spacing*j for j in range(len(evaluator_path_files) + 1)]

    rrt_path_files = os.listdir(rrt_paths)
    rrt_path_files.sort()

    optimal_path = read_optimal_path(optimal_path_file)
    occupancy_map = np.ones(vis_image.shape[:2])
    compute_occupancy_map = True

    images = []

    for i in tqdm(range(len(trajectory_times))):

        #trajectory_time_idx = find_closest_smaller_time(trajectory_times, map_data_time)
        #trajectory_time = trajectory_times[trajectory_time_idx]
        #trajectory = trajectory_locations[0:trajectory_time_idx]
        trajectory_time = trajectory_times[i]
        trajectory = trajectory_locations[0:i]
        #evaluator_path_time = evaluator_path_times[i]
        evaluator_path_time_idx = find_closest_smaller_time(evaluator_path_times, trajectory_time)
        evaluator_path_time = evaluator_path_times[evaluator_path_time_idx]
        if evaluator_path_time_idx != 0:
            evaluator_path, _ = position_arrays_to_location_vector(os.path.join(evaluator_paths_dir, evaluator_path_files[evaluator_path_time_idx-1]))
        else:
            evaluator_path = [np.array([8, 8]), np.array([632, 472])]
        map_data_time_idx = find_closest_smaller_time(map_data_times, trajectory_time)
        map_data_time = map_data_times[map_data_time_idx]
        map_data_name = map_data_names[map_data_time_idx]
        if compute_occupancy_map:
            if len(trajectory) > 0:
                square_width = 65
                x_coordinates = np.array([int(trajectory[-1][0] - int(square_width / 2)), int(trajectory[-1][0] + int(square_width / 2))])
                y_coordinates = np.array([int(trajectory[-1][1] - int(square_width / 2)), int(trajectory[-1][1] + int(square_width / 2))])

                y_coordinates = np.clip(y_coordinates, 0, vis_image.shape[0])
                x_coordinates = np.clip(x_coordinates, 0, vis_image.shape[1])

                cv2.rectangle(occupancy_map, (x_coordinates[0], y_coordinates[0]), (x_coordinates[1], y_coordinates[1]), 0,
                              -1)
            else:
                cv2.rectangle(occupancy_map, (0, 0), (100, 100), 0, -1)
        else:
            occupancy_map_file_format = "{index:05d}_occupancy.png"
            occupancy_map_file = os.path.join(map_dir, occupancy_map_file_format.format(index=map_data_name))
            occupancy_map = cv2.imread(occupancy_map_file)
        map_img = paste_occupancy_map(occupancy_map, vis_image)
        evaluator_img = paste_occupancy_map(occupancy_map, vis_image, color=(128, 128, 128))
        draw_path_in_map(evaluator_img, optimal_path, color=(208, 224, 64), line_thickness=2)
        draw_path_in_map(evaluator_img, optimal_path, color=(255, 255, 255), line_thickness=2)
        draw_path_in_map(evaluator_img, evaluator_path, color=(255, 0, 255), line_thickness=2)
        draw_path_in_map(map_img, trajectory, color=(255, 0, 0), line_thickness=2)
        frame_name_format = "{index:05d}.png"
        frame_name = os.path.join(dest_folder, frame_name_format.format(index=i))
        #cv2.imwrite(frame_name, cv2.hconcat([map_img, evaluator_img]))
        #cv2.putText(img=map_img, text=str(i), org=(30, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=3, color=(255, 255, 255),thickness=3)
        dist = 12
        cv2.circle( evaluator_img, (dist,dist), 10, (0, 255, 0), cv2.FILLED, cv2.LINE_8 )
        cv2.circle( evaluator_img, (640-dist,480-dist), 10, (0, 0, 255), cv2.FILLED, cv2.LINE_8 )
        cv2.circle( map_img, (dist,dist), 10, (0, 255, 0), cv2.FILLED, cv2.LINE_8 )
        cv2.circle( map_img, (640-dist,480-dist), 10, (0, 0, 255), cv2.FILLED, cv2.LINE_8 )
        cv2.rectangle(map_img, (0, 0), (640-1, 480-1), 0, 0)
        cv2.rectangle(evaluator_img, (0, 0), (640-1, 480-1), 0, 0)
        images.append(cv2.cvtColor(cv2.hconcat([map_img, evaluator_img]), cv2.COLOR_BGR2RGB))

        i_max = 650
        if i >= i_max:
            break


    animate_frames(images)