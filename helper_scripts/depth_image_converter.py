import copy
import os
import io
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import cv2
import errno
import argparse
import numpy as np
import yaml as YAML
from map_conversion_functions import label2color, color2label


def parse_arguments():
    parser = argparse.ArgumentParser(description='Converts depth images as created by OAISYS and outputs height map.')
    parser.add_argument('source', type=str,
                        help='input file (map) or folder')
    parser.add_argument('--dest', dest='destination_folder', type=str, default="",
                        help='output folder where new maps shall be stored')
    parser.add_argument('-o', dest='override_all', action='store_true',
                        help='override all files')
    parser.add_argument('-s', dest='supress_ending', action='store_true',
                        help='supress post-fix for output file')
    parser.add_argument('-pf', dest='post_fix', type=str, default=None,
                        help='alternative post-fix for file', required=False)
    parser.add_argument('-f', dest='focal_length', type=float, default=541.14,
                        help='focal length of camera model', required=False)
    # parser.add_argument('-a', dest='focal_length', type=float, default=100.0,
    #                     help='focal length of camera model', required=False)

    args = parser.parse_args()
    return args


def convert_depth_image_2_map(depth_image_path, focal_length, result_folder, override_all, supress_ending, post_fix):

    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)

    width = depth_image.shape[1]
    height = depth_image.shape[0]

    angle_image = np.zeros((height, width))

    for i in range(width):
        for j in range(height):
            dpx = i - width / 2.0
            dpy = j - height / 2.0
            alpha = np.arctan(np.sqrt(dpx**2 + dpy**2)/focal_length)
            angle_image[j, i] = alpha

    depth_image = depth_image * np.cos(angle_image)
    depth_image = 100-depth_image

    d_max = np.max(depth_image)
    d_min = np.min(depth_image)

    print(d_max)
    print(d_min)

    depth_image -= d_min
    depth_image *= 1.0/(d_max-d_min)

    d_max = np.max(depth_image)
    d_min = np.min(depth_image)

    print(d_max)
    print(d_min)

    cv2.imshow('image', depth_image)
    cv2.waitKey(0)


    return True



# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    arguments = parse_arguments()

    source = arguments.source
    results_folder = arguments.destination_folder
    images = []

    try:
        if os.path.isdir(source):
            files = os.listdir(source)
            for file in files:
                path_to_test = os.path.join(source, file)
                if os.path.isfile(path_to_test) and ".exr" in file:
                    images.append(path_to_test)
            if not images:
                print("No .png files found in the specified source folder: " + source)
                raise ImportError
        elif os.path.isfile(source) and ".exr" in source:
            images.append(source)
        else:
            print("The specified source can not be found or used: " + source)
            raise ImportError
    except ImportError:
        exit(errno.ENOENT)
    except:
        print("Unknown error occurred during folder setup.")
        exit(1)

    if not os.path.isdir(results_folder):
        results_folder = os.path.curdir

    file_counter = 0
    images.sort()

    for image_file in images:
        success = convert_depth_image_2_map(image_file, arguments.focal_length, results_folder, override_all=arguments.override_all,
                                supress_ending=arguments.supress_ending, post_fix=arguments.post_fix)
        if success:
            file_counter += 1