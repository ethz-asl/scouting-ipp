FROM ros:melodic

# Install required packages
RUN apt-get update && apt-get install -y python-wstool \
    python-catkin-tools \
    build-essential \
    ros-melodic-cmake-modules \
    ros-melodic-control-toolbox \
    ros-melodic-cv-bridge \
    ros-melodic-filters \
    ros-melodic-eigen-conversions \
    ros-melodic-image-transport \
    ros-melodic-tf \
    ros-melodic-grid-map \
    fonts-cmu \
    autoconf \
    libyaml-cpp-dev \
    protobuf-compiler \
    libgoogle-glog-dev \
    liblapacke-dev \
    git \
    libeigen3-dev \
    python3-venv \
    wget \
    nano && \
    apt clean

# Setup ros workspace
RUN mkdir -p /home/root/ros_ws/src
WORKDIR /home/root/ros_ws
RUN catkin init
RUN catkin config --extend /opt/ros/melodic 
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup scouting-ipp repository 
WORKDIR /home/root/ros_ws/src
RUN catkin build
RUN git clone https://github.com/ethz-asl/scouting-ipp.git

# Setup mav_active_3d_planning repository and apply patch
RUN git clone https://github.com/ethz-asl/mav_active_3d_planning.git
WORKDIR /home/root/ros_ws/src/mav_active_3d_planning/
RUN git checkout 4c139dcb5837c0aa71b1f3055cac8208f806c33e
RUN git apply ../scouting-ipp/mav_active_3d_planning.patch

# Fetch dependencies
WORKDIR /home/root/ros_ws
RUN wstool init src src/mav_active_3d_planning/mav_active_3d_planning_https.rosinstall
RUN wstool merge -t src src/scouting-ipp/dependencies_https.rosinstall
RUN wstool update -t src 

WORKDIR /home/root/ros_ws/src/ompl
RUN git checkout 1.5.2

WORKDIR /home/root/ros_ws
# We use the libyaml-cpp-dev system package instead
RUN touch src/yaml_cpp_catkin/CATKIN_IGNORE

# Build required packages 
RUN catkin build ros_planner_planexp mav_simulator map_server_planexp
RUN [ "/bin/bash", "-c", "source /home/root/ros_ws/devel/setup.bash" ]

# Setup ros workspace on start up
RUN echo "source /home/root/ros_ws/devel/setup.bash" >> /root/.bashrc

# Prepare mountpoint for data directory
RUN mkdir -p /home/root/data/
# It is not recommended to copy the data directory. Better is to mount it into the running container. 
#COPY data /home/root/data

# Setup environment
ENV ROS_WS_DIR="/home/root/ros_ws/"
ENV USER="root"

# Install python environment required for evaulation scripts using Miniconda 
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
ENV PATH="/root/miniconda3/bin:$PATH"
RUN mkdir /root/.conda && bash Miniconda3-latest-Linux-x86_64.sh -b
RUN cat ~/.bashrc
RUN conda init bash \
    && . ~/.bashrc \
    && conda create --name scouting_ipp_venv python=3.9.12 \
    && conda activate scouting_ipp_venv \
    && conda install pandas pyyaml=6.0 matplotlib scipy dataclasses opencv pylatex conda-forge::pylatex tqdm
# Set conda environment on start up
RUN echo "conda activate scouting_ipp_venv" >> /root/.bashrc