#!/bin/bash

PACKAGE='libgoogle-glog-dev'
dpkg -s $PACKAGE &> /dev/null
if [ $? -eq 0 ]; then
    echo "$PACKAGE is installed"
else
    echo "$PACKAGE is NOT installed"
    sudo apt install libgoogle-glog-dev
fi

rm -rf ~/catkin_slam

unzip catkin_slam.zip -d ~/
cd ~/catkin_slam
sudo chmod 777 -R *
cd Livox-SDK2-master && mkdir build && cd build
cmake ..
sudo make install

cd ~/catkin_slam/src/livox_ros_driver2
./build.sh ROS1