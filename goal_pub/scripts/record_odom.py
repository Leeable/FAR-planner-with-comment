'''
    这部分代码是用来记录小车行驶的odom的
'''

import rospy
import os
from nav_msgs.msg import Odometry
if __name__ == '__main__':
    rospy.init_node("Record_Odom")
    odom_list = []
    while True:
        flag = input("存数据么？ y 存数据, e 退出\n")
        if flag == "y":
            odom = rospy.wait_for_message("/state_estimation", Odometry)
            odom_pos = odom.pose.pose.position
            f = open("./travel_odom/odom.txt", 'a+')
            f.writelines(str((odom_pos.x, odom_pos.y, odom_pos.z))+"\n")
            f.close()
            os.system('cls' if os.name == 'nt' else 'clear')
            print("已加入到当前目录的odom.txt文件")
            print("--------------------------")
        if flag == "e":
            break
        else:
            continue
