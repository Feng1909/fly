
# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import math
import ReqCopterSim
import RflyRosStart
import sys

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True

req = ReqCopterSim.ReqCopterSim() # 获取局域网内所有CopterSim程序的电脑IP列表
StartCopterID = 1 # 初始飞机的ID号
TargetIP = req.getSimIpID(StartCopterID) # 获取CopterSim的1号程序所在电脑的IP，作为目标IP
# 注意：如果是本电脑运行的话，那TargetIP是127.0.0.1的本机地址；如果是远程访问，则是192打头的局域网地址。
# 因此本程序能同时在本机运行，也能在其他电脑运行。

print('Request CopterSim Send data.')
req.sendReSimIP(StartCopterID) # 请求回传数据到本电脑

print(RflyRosStart.isLinux,RflyRosStart.isRosOk)

# 自动开启mavros
if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
    print('This demo can only run on with Ros')
    sys.exit(0)

# 自动开启RosCore
ros = RflyRosStart.RflyRosStart(StartCopterID,TargetIP)


vis = VisionCaptureApi.VisionCaptureApi(TargetIP)

# VisionCaptureApi 中的配置函数
vis.jsonLoad() # 加载Config.json中的传感器配置文件
isSuss = vis.sendReqToUE4(
    0, TargetIP
)
vis.startImgCap() # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')
vis.sendImuReqCopterSim(StartCopterID,TargetIP) # 发送请求，从目标飞机CopterSim读取IMU数据,回传地址为127.0.0.1，默认频率为200Hz
# 执行本语句之后，会自动开启数据监听，已经可以通过vis.imu读取到IMU数据了。

#ros.EndRosLoop()
