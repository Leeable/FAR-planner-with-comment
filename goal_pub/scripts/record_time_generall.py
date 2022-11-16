'''
    这部分代码是用来记录任何时间的，运行时加入话题名称和保存名称  
    如    /laser_time     laser_time_document
'''

import rospy
import sys
from std_msgs.msg import Float32
global time
def send_callback(Float32):
    global time
    time = Float32.data

topic = sys.argv[1]
file_name = sys.argv[2]
# topic = "/runtime"
# file_name = "test"
print(type(topic))
if __name__ == '__main__':
    rospy.init_node('listener')
    time_list = []
    time = 0
    while not rospy.is_shutdown():    
        rospy.Subscriber(topic,Float32,send_callback,queue_size=10)
        print(time)
        rospy.sleep(0.5)
        time_list.append(str(time))
        time_list.append(",")
        f = open("./"+file_name+".txt","w")
        f.writelines(time_list)
        f.close()