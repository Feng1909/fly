import rospy
# from mavros_msgs.msg import CommandBool,
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest
 
def arm_and_takeoff(altitude):
    rospy.init_node('takeoff_node', anonymous=True)
    
    # 等待连接
    rospy.wait_for_service('mavros/cmd/arming')
    rospy.wait_for_service('mavros/set_mode')
    rospy.wait_for_service('mavros/cmd/takeoff')
    
    arming_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    # set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)
    # takeoff_service = rospy.ServiceProxy('mavros/cmd/takeoff', CommandBool)
    
    # 解锁和解锁无人机
    arm_command = CommandBoolRequest(value=False)
    arming_service.call(arm_command)
    
    # # 设置飞行模式为OFFBOARD
    # set_mode_service.call(custom_mode='OFFBOARD')
    
    # # 发送升起命令
    # takeoff_command = CommandBoolRequest(value=True)
    # takeoff_service.call(takeoff_command)
 
if __name__ == '__main__':
    try:
        arm_and_takeoff(altitude=10)  # 起飞并指定高度（以米为单位）
    except rospy.ROSInterruptException:
        pass