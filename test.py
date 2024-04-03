import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest
 
def arm():
    rospy.init_node('takeoff_node', anonymous=True)
    
    # 等待连接
    rospy.wait_for_service('mavros/cmd/arming')
    
    arming_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    
    # 解锁和解锁无人机
    arm_command = CommandBoolRequest(value=False)
    arming_service.call(arm_command)
 
if __name__ == '__main__':
    try:
        arm() 
    except rospy.ROSInterruptException:
        pass
