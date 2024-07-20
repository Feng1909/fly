import PX4MavCtrlV4ROS as PX4MavCtrl
import time


# 创建一个ROS控制实例，针对飞机1
mav = PX4MavCtrl.PX4MavCtrler(CopterID=1)
time.sleep(1)

print('进入offboard并解锁')
mav.initOffboard()
time.sleep(1)


# 飞机起飞到0.1米前，0.8米高
print('发送起飞命令')
mav.SendPosNED(0.1,0,-0.8)
time.sleep(10)

print('PosE',mav.uavPosNED)
print('VelE',mav.uavVelNED)
print('Euler',mav.uavAngEular)
print('Quaternion',mav.uavAngQuatern)
print('Rate',mav.uavAngRate)

time.sleep(1)
print('Start control.')

# 下面开始你的控制算法
# mav.SendPosNED(x,y,z,yaw) 发送期望位置点，NED地球坐标系
# mav.SendVelNED(vx,vy,vz,yawrate) 发送速度，NED地球坐标系
# mav.SendVelFRD(vx,vy,vz,yawrate) 发送速度，FRD机体坐标系，通常而言，设定vx和yawrate即可，前进速度和偏航速度。
# mav.SendPosVelNED(PosE=[0,0,0],VelE=[0,0,0],yaw,yawrate) 同时控制飞机位置和速度


# 下面仅展示使用SendVelFRD控制的例子，会撞墙，为正常现象

print('开始速度控制')

# 向前0.1m/s飞，并缓慢调转方向
mav.SendVelFRD(0.1,0,0,0.05)

