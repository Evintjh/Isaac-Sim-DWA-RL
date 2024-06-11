FROM nvcr.io/nvidia/isaac-sim:2022.2.1

ARG DEBIAN_FRONTEND=noninteractive
ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN rm /etc/apt/sources.list.d/cuda.list
RUN apt-key del 7fa2af80
RUN apt-get update && apt-get install -y curl lsb-release ca-certificates dirmngr
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64/7fa2af80.pub

# # ROS


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y -qq ros-noetic-desktop-full


RUN apt-get update && apt-get install -y \
  libopencv-dev \
  liblua5.2-dev \
  virtualenv \
  screen \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-navigation \
  ros-noetic-rviz \
  ros-noetic-pcl-ros \
  ros-noetic-gazebo-ros \
  vim \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential \
  python3-rospkg-modules \
  libarmadillo-dev \
  blender \
  libeigen3-dev \
  libassimp-dev \
  cmake \
  libmagick++-dev \
  libboost-all-dev \
  libgl-dev \
  libfontconfig1 \
  libfreetype6 \
  libxcb1 \
  libxkbcommon* \
  libxkbcommon-x11-* \
  software-properties-common \
  ros-noetic-pointcloud-to-laserscan
RUN apt-get install  --reinstall ca-certificates
RUN ./python.sh -m pip install rospkg tensorboard stable_baselines3 
RUN ./python.sh -m pip install omegaconf

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
COPY ./utils/differential_controller.py /isaac-sim/exts/omni.isaac.wheeled_robots/omni/isaac/wheeled_robots/controllers/differential_controller.py
COPY ./utils/rotating_lidar_physX.py /isaac-sim/exts/omni.isaac.sensor/omni/isaac/sensor/scripts/rotating_lidar_physX.py

WORKDIR /isaac-sim
