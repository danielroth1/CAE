#!/bin/bash

# Install required dependencies for CAE project

# Update package list
sudo apt-get update

# Install build tools and libraries
sudo apt-get install -y \
    build-essential \
    cmake \
    libglew-dev \
    freeglut3-dev \
    libboost-all-dev \
    libmpfr-dev \
    libgmp3-dev \
    libxmu-dev \
    libxi-dev \
    qtbase5-dev \
    qt5-qmake \
    qttools5-dev \
    qttools5-dev-tools

# Clone CGAL library
if [ ! -d "cgal" ]; then
    git clone https://github.com/CGAL/cgal.git
    echo "CGAL cloned successfully"
else
    echo "CGAL directory already exists, skipping clone"
fi
