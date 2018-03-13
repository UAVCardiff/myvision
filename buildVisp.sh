#!/bin/bash
cd $HOME
sudo apt-get install -y \
    build-essential \
    cmake-curses-gui \
    libopencv-dev \
    liblapack-dev \
    libeigen3-dev \
    libv4l-dev \
    libxml2-dev \
    cmake \
    pkg-config

# Python 2.7
sudo apt-get install -y python-dev python-numpy python-py python-pytest
# GStreamer support
#sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 

export VISP_WS=$HOME/visp-ws
mkdir -p $VISP_WS
cd $VISP_WS
git clone https://github.com/lagadic/visp.git

mkdir $VISP_WS/visp-build
cd $VISP_WS
cmake \

cd $VISP_WS/visp-build
# Consider running jetson_clocks.sh before compiling
make -j4
