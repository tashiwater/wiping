import rospy

def get_param(name, retry_num=50, retry_delay_sec=0.1):
    ret = None
    while (True):
        ret = rospy.get_param(name, None)
        retry_num -= 1
        if((ret is not None) or (retry_num <= 0)):
            break
        rospy.sleep(retry_delay_sec)
    return ret
