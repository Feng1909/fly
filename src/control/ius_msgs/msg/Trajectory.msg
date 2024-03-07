Header header

# 轨迹的id，用来标记不同轨迹
uint32 traj_id

geometry_msgs/Point[] pos # 轨迹位置点
float64[] yaw # 位置点对应的偏航角
float64[] time # 位置点对应的时间
