#TODO: Plese clean me up!!!
import os
import time
import errno
import shutil
import pickle
import signal
import argparse
import subprocess
import numpy as np
import pandas as pd


def handler(signum, frame, call_exit=True):
    print("Processes killed")
    try:
        blennder_processes = map(int, subprocess.check_output(["pidof", "blender"]).split())
        for process in blennder_processes:
            try:
                pid = int(process)
                if pid > 0:
                    os.kill(pid, signal.SIGKILL)
            except:
                pass
    except:
        pass
    try:
        oaisys_client_processes = map(int, subprocess.check_output(["pidof", "oaisys_client_node"]).split())
        for process in oaisys_client_processes:
            try:
                pid = int(process)
                if pid > 0:
                    os.kill(pid, signal.SIGKILL)
            except:
                pass
    except:
        pass

    if call_exit:
        exit(1)


if __name__ == '__main__':
    verbose = False
    mountain = True
    ros_ws_dir = os.getenv("ROS_WS_DIR")
    working_dir = os.getcwd()
    os.chdir(os.path.join(ros_ws_dir, "src", "scouting-ipp", "helper_scripts"))
    signal.signal(signal.SIGINT, handler)
    # Start process
    #TODO: Clean me!!
    output_dir = "/tmp/test/"
    basic_maps_dir = "/home/rockenbf/Desktop/maps_final/basic_maps/"
    post_fixes = ["_smaller", "_normal", "_bigger"]
    config_dir = "/home/rockenbf/Desktop/maps_final/"
    configs = ["map_smaller.yaml", "map_normal.yaml", "map_bigger.yaml"]
    obstacle_map_folder = "/home/rockenbf/data/maps/"
    obstacle_maps = ["map_10", "map_11", "map_12"]
    if mountain:
        basic_maps_dir = "/home/rockenbf/Desktop/maps_final/basic_maps/"
        obstacle_map_folder = "/home/rockenbf/Desktop/maps_final/obstacle_files/"
        obstacle_maps = ["bitmap"]

    content = os.listdir(basic_maps_dir)
    basic_maps = []
    for file_name in content:
        if os.path.isfile(os.path.join(basic_maps_dir, file_name)):
            basic_maps.append(os.path.join(basic_maps_dir, file_name))

    for i, config, post_fix in zip(range(len(configs)), configs, post_fixes):
        for basic_map in basic_maps:
            basic_map_name = (basic_map.split(".")[0]).split("/")[-1]
            subprocess.call(["python", "label_replacer.py", "--dest", output_dir, "--map",
                             os.path.join(config_dir, config), "-pf", post_fix, basic_map])
            shifted_file = os.path.join(output_dir, basic_map_name + post_fix + ".png")
            subprocess.call(["python", "create_config_space_map.py", "--dest", output_dir, "-s",
                             shifted_file, "-b", shifted_file])
            blured_file = os.path.join(output_dir, basic_map_name + post_fix + "_blured.png")
            if "map_" in basic_map_name:
                basic_map_str = basic_map_name.split("map_")
                if basic_map_str[0] == "s":
                    basic_map_str = "map_" + basic_map_str[-1] + str(i)
                    blured_file = os.path.join(output_dir, basic_map_str + ".png")
            os.rename(os.path.join(output_dir, basic_map_name + post_fix + "_" + basic_map_name + post_fix + ".png"),
                      blured_file)
            subprocess.call(["python", "create_map_visualization.py", "--dest", output_dir, "-u", "3599", shifted_file])

            for j, obstacle_map in enumerate(obstacle_maps):
                obstacle_map_file = os.path.join(obstacle_map_folder, obstacle_map + ".png")
                subprocess.call(["python", "create_config_space_map.py", "--dest", output_dir, "-s", obstacle_map_file,
                                 "--ids", "0", "-w", "20", "-ir", blured_file])
                blured_file_path, blured_file_name = os.path.split(blured_file)
                blured_file_name = blured_file_name.split(".")[0]
                obstacle_nr = j+1
                if mountain:
                    obstacle_nr = 4
                final_map = os.path.join(output_dir, blured_file_name + str(obstacle_nr) + ".png")
                os.rename(os.path.join(output_dir, blured_file_name + "_" + obstacle_map + ".png"), final_map)
                subprocess.call(["python", "create_map_visualization.py", "--oid", "0", "--dest", output_dir, "-u",
                                 "3599", final_map])
            new_blured_file = os.path.join(output_dir, blured_file_name + str(0) + ".png")
            os.rename(blured_file, new_blured_file)
            subprocess.call(["python", "create_map_visualization.py", "--dest", output_dir, "-u", "3599", new_blured_file])

    # time.sleep(2)
    # client_process = subprocess.Popen(["/home/rockenbf/ros_ws/devel/lib/oaisys_client_ros/oaisys_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    # time.sleep(2)
    # os.system("rostopic pub /reference/command/trajectory geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "", seq: 0}, pose: {position: {x: 40.0, y: 40.0, z: 100.0}, orientation: {w: 1.0}}}' --once")
    # new_folder = None
    # while new_folder is None:
    #     if verbose:
    #         print("Waiting for results!")
    #     time.sleep(0.3)
    #     content = os.listdir(oaisys_tmp_dir)
    #     for folder in content:
    #         if os.path.isdir(os.path.join(oaisys_tmp_dir, folder)) and folder not in old_folders:
    #             new_folder = folder
    #             image_path = os.path.join(oaisys_tmp_dir, new_folder, "batch_0001", "sensor_1")
    #
    # while not os.path.isdir(image_path):
    #     if verbose:
    #         print("Waiting for folders!")
    #     time.sleep(1)
    #
    # image_file = None
    # while image_file is None:
    #     if verbose:
    #         print("Waiting for results!")
    #     time.sleep(0.3)
    #     content = os.listdir(image_path)
    #     for file in content:
    #         if "sensor_1_semantic_label_00.png" in file:
    #             image_file = os.path.join(image_path, file)
    #
    # new_image_file = os.path.join("/tmp/test", new_folder+".png")
    # shutil.move(image_file, new_image_file)
    #
    # os.chdir(working_dir)
    # analyse_process = subprocess.call(["python", "analyse_map.py", "--dest", "/tmp/test/", "-m", "/home/rockenbf/data/maps/cfg/map_2.yaml", "-o", new_image_file], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    os.chdir(working_dir)
    handler(None, None)

