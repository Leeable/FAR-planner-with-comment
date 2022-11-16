'''
    这部分代码是用来记录小车运行时候的odom、waypoint、goal、connection的数据的
'''
import rospy
from std_msgs.msg import Bool
if __name__ == '__main__':
    rospy.init_node("Record_data")
    pub = rospy.Publisher("/record_data", Bool,queue_size=5)
    while True:
        value = input("输入1开始记录，输入0停止记录，输入e退出\n")
        if value == "e":
            break
        else:
            pub.publish(bool(int(value)))
            exit = input("已发送，开始记录,按e回到上级菜单\n")
            if exit == "e":
                continue
    print("结束了")
    