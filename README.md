# Fly项目，使用激光雷达定位+ego planner规划+MPC控制

## 激光雷达定位
* Livox-SDK
安装Livox驱动，用于mid360雷达点云接收
```
cd Livox-SDK
mkdir build
cd build && cmake ..
make
sudo make install
```

* Faster Lio
用于雷达定位，对源码进行了修改，在`/Odometry`话题中可以输出三轴速度信息，来源于IMU的EKF积分

## ego planner规划
Waiting...

## MPC控制
Waiting...