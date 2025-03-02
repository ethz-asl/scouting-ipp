## Installation
Installation instructions to run the planner in your local ros workspace.

**Prerequisites**

1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).

2. If not already done so, create a catkin workspace with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/):

```shell script
sudo apt-get install python-catkin-tools
mkdir -p ~/ros_ws/src
cd ~/ros_ws
catkin init
catkin config --extend /opt/ros/melodic  # exchange melodic for your ros distro
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Installation**

1. Move to your catkin workspace:
```shell script
cd ~/ros_ws/src
```

2. Install system dependencies:
```shell script
sudo apt-get install python-wstool python-catkin-tools
```

3. Install the [mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning) package following its installation instructions ([https://github.com/ethz-asl/mav_active_3d_planning](https://github.com/ethz-asl/mav_active_3d_planning)) and apply the patch file ```mav_active_3d_planner.patch```:
```shell script 
roscd mav_active_3d_planning/..
git apply ../scouting-ipp/mav_active_3d_planning.patch
```

4. Download repo using an SSH key or via HTTPS:
```shell script
git clone git@github.com:ethz-asl/scouting-ipp.git # SSH
git clone https://github.com/ethz-asl/scouting-ipp.git # HTTPS
```

5. Download and install the dependencies of the packages you intend to use.

```shell script
# system dependencies, replace melodic with your ros distro if necessary:
sudo apt-get install ros-melodic-cmake-modules \
    ros-melodic-control-toolbox \
    yaml_cpp_catkin \
    fonts-cmu \
    autoconf \
    libyaml-cpp-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    liblapacke-dev \
    git \
    libeigen3-dev

# If you already intialized ws tool use 'wstool merge -t'
wstool init . ./scouting-ipp/dependencies_ssh.rosinstall # SSH
wstool init . ./scouting-ipp/dependencies_https.rosinstall # HTTPS
wstool update
```

6. Deactivate yaml_cpp_catkin package:
```shell script
# We use the libyaml-cpp-dev system package instead
touch src/yaml_cpp_catkin/CATKIN_IGNORE
```

7. Source and compile:
```shell script
source ../devel/setup.bash
catkin build catkin build ros_planner_planexp mav_simulator map_server_planexp # Builds this package only
catkin build # Builds entire workspace, recommended for full install.
```

8. For the evaluation scripts the following python packages are required:
```shell script 
pip3 install pandas \
    pyyaml \
    matplotlib \
    scipy \
    dataclasses \
    opencv \
    pylatex \
    pylatex \
    tqdm 
```
**Note:** The minimum python version for those dependencies is 3.9.12 (a virtual environment cann be used). 
**Ubuntu 20.04:** For Ubuntu 20.04 it is not recommended to update the system python to the above requested version. The use of a conda environment is recommended. The name of this environment is by default ``scouting_ipp_venv``. If the environment is named differently the name has to be set in the ``experiment.sh`` file.

## Data Repository
The recommended folder structure including an example is located in the `docker/data` directory.

**Note:** It is recommended to place the folder directly into the home directory or create a symbolic link from there to the storage location.

## Example
A example of how planner configurations are specified is given in `cfg/planners/example_config.yaml`.
The example planner uses local motion primitives to expand new segments and the goal driven path aware evaluator as gain formulation.
To see the planner in action, run
```bash
roslaunch ros_planner_planexp example.launch
```
The planner will be built from the config file and visualized in RVIZ.
A useful parameter to set is `verbose_modules: true`, as all available params of all built modules will be printed to console.

A local motion primitive based planner starting exploration.

## Experiment Example
In order for the experiment script to work the path to the catkin workspace hast to be exported as environment variable:
```bash 
export ROS_WS_DIR=/home/${USER}/ros_ws/
```
The following set of commands will execute an experiment run (10 iterations, 15 minutes, logger interval 10 seconds) on one map using the goal driven and the goal driven path aware evaluator.
```bash
roscd ros_planner_planexp
cd scripts/
source run_experiment.sh
```