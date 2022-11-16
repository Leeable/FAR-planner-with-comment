'''
    这部分代码是用来记录laser的时间的
'''
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import os


global flag 

def send_callback(Bool):
    global flag
    flag = bool(Bool.data)

if __name__ == '__main__':
    time_list = []
    flag = False
    rospy.init_node("receive_time")
    print("wait for goal_pub work")
    work = rospy.wait_for_message("/goal_pub_work", Bool)
    print("program launched")
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/send_done",Bool,send_callback,queue_size=5)
        except:
            flag = False
        if(flag!=True ):
            time = rospy.wait_for_message("/laser_time", Float32)
            time = str(time)[6:]
            time_list.append(str(time))
            time_list.append(",")
            os.system('cls' if os.name == 'nt' else 'clear')
            print("add time")
            f = open("./laser_time_modify_17r.txt","w")
            f.writelines(time_list)
            f.close()
        if flag == True:
            break
    print("recording already done")