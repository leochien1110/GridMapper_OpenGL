#
`sudo apt-get update && sudo apt-get upgrade && sudo apt-get install build-essential`

`sudo apt-get install cmake`

`sudo apt-get install git`

`sudo apt-get install qtbase5-dev`

`sudo apt-get install qtdeclarative5-dev`

# OpenGL
Install OpenGL3
`sudo apt-get install libglfw3`

`sudo apt-get install libglfw3-dev`

# OpenCV 3.4
Build `git` folder
```bash
mkdir git && cd git
```
OpenCV source code:
```bash
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

git clone https://github.com/opencv/opencv.git

git clone https://github.com/opencv/opencv_contrib.git

cd opencv
```
Checkout to version 3.4 or newer:
```bash
git checkout 3.4
```
> Make sure to checkout the same versionin `opencv_contrib`

OpenCV Contribution:
```bash
cd ..

cd opencv_contrib

git checkout 3.4
```
Go back to main source code:
```bash
cd ..

cd opencv

mkdir build && cd build
```

Build the Project:
```bash
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
      -D BUILD_EXAMPLES=ON ..`

make -j4`

sudo make install`

export OpenCV_DIR=~/git/opencv/build
```
> :bulb: or change to where you install `/opencv/build`

# RealSense
```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

git clone https://github.com/IntelRealSense/librealsense.git

cd librealsense

sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

./scripts/setup_udev_rules.sh

./scripts/patch-realsense-ubuntu-lts.sh
```
if the above step fail:kernel isn't supported, try: 
```bash
./scripts/patch-ubuntu-kernel-4.16.sh
```

```bash
echo 'hid_sensor_custom' | sudo tee -a /etc/modules

mkdir build && cd build

cmake ../ -DBUILD_EXAMPLES=true -DBUILD_CV_EXAMPLES=true

sudo make uninstall && make clean && make -j8 && sudo make install
```

# Mapper
```bash
sudo apt-get install libncurses5-dev

cd mapper

mkdir build && cd build

cmake ..  # [1]

make

```

# run code
```bash
cd src/
```

# mapper:
```bash
./onboard Ground.Station.IP.address
````

# ground station
```bash
./read_voxel
```

## Knwon Issue
[1] Known OpenCV error while building mapper
**If you see the error message while cmaking the project in the build folder:**

Could not find a package configuration file provided by "OpenCV" with any
  of the following names:

  OpenCVConfig.cmake
  
  opencv-config.cmake

  Add the installation prefix of "OpenCV" to CMAKE_PREFIX_PATH or set
  "OpenCV_DIR" to a directory containing one of the above files.  If "OpenCV"
  provides a separate development package or SDK, be sure it has been
  installed.



**Run `export OpenCV_DIR=~/git/opencv/build` and try again.**

**Note:** or change to where you install `/opencv/build`
