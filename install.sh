#!/bin/bash
#
sudo apt-get update && sudo apt-get upgrade && sudo apt-get -y install build-essential
sudo apt-get -y install cmake
sudo apt-get -y install git
sudo apt-get -y install qtbase5-dev
sudo apt-get -y install qtdeclarative5-dev

# OpenGL
sudo apt-get -y install libglfw3
sudo apt-get -y install libglfw3-dev

# OpenCV 3.4
cd ~/git/
sudo apt-get -y install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

cd opencv
git checkout 3.4

cd ..
cd opencv_contrib
git checkout 3.4
cd ..

cd opencv
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -DBUILD_opencv_cudacodec=OFF \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..

make -j8 # runs 7 jobs in parallel
sudo make install

export OpenCV_DIR=~/git/opencv/build	# or change to where you install /opencv/build

# RealSense
cd ../..
sudo apt-get update && sudo apt-get upgrade #&& sudo apt-get dist-upgrade
git clone https://github.com/IntelRealSense/librealsense.git

cd librealsense
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev 

./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh		
#if the above step fail:kernel isn't supported, try: ./scripts/patch-ubuntu-kernel-4.16.sh

echo 'hid_sensor_custom' | sudo tee -a /etc/modules

mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_CV_EXAMPLES=true
sudo make uninstall && make clean && make -j8 && sudo make install

# Mapper
cd ../..
sudo apt-get -y install libncurses5-dev

cd voxelmap
mkdir build && cd build
cmake ..
make
