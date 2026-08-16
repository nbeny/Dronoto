# Image de base : ROS 2 Jazzy + outillage de construction commun.
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      build-essential \
      cmake \
      git \
      libeigen3-dev \
      ros-jazzy-tf2-ros \
      ros-jazzy-tf2-eigen \
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages --no-cache-dir pyyaml pytest

WORKDIR /ws
