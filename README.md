# Essential dependencies
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install git
sudo apt-get install qtbase5-dev
sudo apt-get install qtdeclarative5-dev
```
# OpenGL
```
sudo apt-get install libglfw3
sudo apt-get install libglfw3-dev
```
# OpenCV 3.4
```
mkdir git && cd git
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
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
      -D CMAKE_INSTALL_PREFIX=$cwd/installation/OpenCV-3.4 \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..

make -j8 # runs 7 jobs in parallel
sudo make install

export OpenCV_DIR=~/git/opencv/build	# or change to where you install /opencv/build
```
# RealSense
```
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
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
```
# Mapper
```
sudo apt-get install libncurses5-dev

cd mapper
mkdir build && cd build
cmake ..
make
```
# run code
```
cd src/
```
# mapper:
```
./onboard "your.ip.address"
```
# ground station
```
./read_voxel
```
