'''记录到达一个waypoint的时间'''


import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
global flag, dflag

def send_callback(Bool):
    global flag
    flag = bool(Bool.data)

def send_done_callback(Bool):
    global dflag
    dflag = bool(Bool.data)

def rec_time(Float32):
    time = Float32().time()


if __name__ == '__main__':
    time_list = []
    distance_list = []
    dist_ls = []
    time_ls = []
    flag = False
    dflag = False
    rospy.init_node("receive_travel_time")
    print("wait for goal")
    work = rospy.wait_for_message("/goal_pub_work", Bool)
    print("start work\n")
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/far_reach_goal_status",Bool,send_callback,queue_size=5)
            rospy.Subscriber("/send_done",Bool,send_done_callback,queue_size=5)
        except:
            flag = False
        try:
            time = rospy.wait_for_message("/time_duration", Float32)
            time = str(time)[6:]
        except:
            time = 0
        try:
            distance = rospy.wait_for_message("/traveling_distance",Float32)
            distance = str(distance)[6:]
        except:
            distance = 0
        # 如果到了目标点，记录时间
        if flag == True :
            print("add time")
            time_list.append(str(time))
            try:
                value_t =float(time_list[-2])
            except:
                value_t = 0.0
            timer = float(time_list[-1]) - value_t
            time_ls.append(str(timer))
            time_ls.append(",")   
            ft = open("./travel_odom/tunnel/travel_time.txt","w")
            ft.writelines(time_ls)
            ft.close()

            print("add_distance")
            distance_list.append(str(distance))
            try:
                value =float(distance_list[-2])
            except:
                value = 0.0
            dist = float(distance_list[-1]) - value
            dist_ls.append(str(dist))
            dist_ls.append(",")
            fd = open("./travel_odom/tunnel/travel_distance.txt","w")
            fd.writelines(dist_ls)
            fd.close()
            
            rospy.sleep(4)
            flag = False
            
        # 如果都发布完了，退出
        if dflag == True:
            break
        else:
            continue
    print("recording already done")