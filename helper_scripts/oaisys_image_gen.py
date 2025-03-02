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
        oaisys_process.kill()
    except:
        pass
    try:
        client_process.kill()
    except:
        pass
    try:
        analyse_process.kill()
    except:
        pass

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
    ros_ws_dir = os.getenv("ROS_WS_DIR")
    working_dir = os.getcwd()
    os.chdir(os.path.join(ros_ws_dir, "src", "oaisys_rockenbf"))
    signal.signal(signal.SIGINT, handler)
    # Start process
    oaisys_tmp_dir = "/home/root/ros_ws/src/oaisys_rockenbf/oaisys_tmp/"
    content = os.listdir(oaisys_tmp_dir)
    old_folders = []
    for folder in content:
        if os.path.isdir(os.path.join(oaisys_tmp_dir, folder)):
            old_folders.append(folder)
    oaisys_process = subprocess.Popen(["python", "run_oaisys_online.py", "--config-file", "/home/root/ros_ws/src/oaisys_rockenbf/cfgExamples/OAISYS_MT_cfg.json"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(2)
    client_process = subprocess.Popen(["/home/root/ros_ws/devel/lib/oaisys_client_ros/oaisys_client_node"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(2)
    os.system("rostopic pub /reference/command/trajectory geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "", seq: 0}, pose: {position: {x: 40.0, y: 40.0, z: 100.0}, orientation: {w: 1.0}}}' --once")
    new_folder = None
    while new_folder is None:
        if verbose:
            print("Waiting for results!")
        time.sleep(0.3)
        content = os.listdir(oaisys_tmp_dir)
        for folder in content:
            if os.path.isdir(os.path.join(oaisys_tmp_dir, folder)) and folder not in old_folders:
                new_folder = folder
                image_path = os.path.join(oaisys_tmp_dir, new_folder, "batch_0001", "sensor_1")

    while not os.path.isdir(image_path):
        if verbose:
            print("Waiting for folders!")
        time.sleep(1)

    image_file = None
    while image_file is None:
        if verbose:
            print("Waiting for results!")
        time.sleep(0.3)
        content = os.listdir(image_path)
        for file in content:
            if "sensor_1_semantic_label_00.png" in file:
                image_file = os.path.join(image_path, file)

    new_image_file = os.path.join("/tmp/test", new_folder+".png")
    shutil.move(image_file, new_image_file)

    os.chdir(working_dir)
    analyse_process = subprocess.call(["python", "analyse_map.py", "--dest", "/tmp/test/", "-m", "/home/root/data/maps/cfg/map_2.yaml", "-o", new_image_file], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    os.chdir(working_dir)
    handler(None, None)

