#!/bin/bash
sudo apt install \
    libeigen3-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    libyaml-cpp-dev \
    libqt5serialport5-dev \ 
    libsuitesparse-dev \
    libceres-dev \
    ros-humble-slam-toolbox 
git submodule update --init --recursive