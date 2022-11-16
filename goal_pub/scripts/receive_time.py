'''
    这部分代码是用来记录vgraph的时间的
'''
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32


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
            time = rospy.wait_for_message("/runtime", Float32)
            time = str(time)[6:]
            time_list.append(str(time))
            time_list.append(",")
            print("add time")
        else:
            f = open("./time.txt","w")
            f.writelines(time_list)
            f.close()
            break
    print("recording already done")