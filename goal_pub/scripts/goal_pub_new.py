'''
    这部分代码是发送随机坐标的
'''
import random

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

global reach, odom, pos_list, waypoint, count, last_goal, goal, end, odom_len
indoor_odom = "./travel_odom/indoor_odom.txt"
tunnel_odom = "./travel_odom/tunnel_odom.txt"
tunnel_odom_simple = "./travel_odom/tunel_odom_simple.txt"
forest_odom = "./travel_odom/forest_odom.txt"
garage_odom = "./travel_odom/garage_odom.txt"

def reach_callback(Bool):
    global reach
    reach = bool(Bool.data)


class Point3D:
    def __init__(self, x=0, y=0, z=0):
        self._x = x
        self._y = y
        self._z = z

    def x(self):
        return self._x

    def y(self):
        return self._y

    def z(self):
        return self._z

    def x(self, value):
        self._x = value

    def y(self, value):
        self._y = value

    def z(self, value):
        self._z = value

# generate goal with certain point list


def generate_goal_from_list(count):
    goal_list = [[13.94, 40.1], [13.94, 50.1], [
        31.4, 50.1], [31.4, 40.1], [13.94, 40.1], [13.94, 40.1]]
    goal_pos = goal_list[count]
    goal = Point3D(goal_pos[0], goal_pos[1], 0.75)
    return goal
# random generate goal from list table


def generate_goal(odom_list=[]):
    goal = random.choice(odom_list)
    temp = goal.replace("(", "").replace(")", "")
    goal = tuple([float(i) for i in temp.split(",")])
    goal_pos = Point3D(goal[0], goal[1], goal[2])
    return goal_pos
# generate odom list from txt


def generate_goal_with_order(odom_list=[], count=0):
    
    goal = odom_list[count]
    temp = goal.replace("(", "").replace(")", "")
    goal = tuple([float(i) for i in temp.split(",")])
    goal_pos = Point3D(goal[0], goal[1], goal[2])
    return goal_pos


def generate_list_from_odom(path=""):
    odom_list = []
    f = open(path, "r")
    line = f.readlines()
    for item in line:
        odom_list.append(item.strip())
    return odom_list
# send goal to Far planner


def send_goal(goal_pos=Point3D()):
    global reach
    global count
    global last_goal
    global goal
    global end
    global odom
    global odom_len
    pub = rospy.Publisher("/goal_point", PointStamped, queue_size=5)
    pub_done = rospy.Publisher("/send_done", Bool, queue_size=5)
    # way_point_sub = rospy.Subscriber(
    #     "/way_point", PointStamped, way_point_callback, queue_size=5)
    goal_point = PointStamped()
    goal_point.header.frame_id = "map"
    goal_point.header.stamp = rospy.Time.now()
    x = float(goal_pos._x)
    y = float(goal_pos._y)
    z = float(goal_pos._z)
    goal_point.point.x = x
    goal_point.point.y = y
    goal_point.point.z = z
    if(last_goal == None or last_goal != goal):
        rospy.sleep(2)
        pub.publish(goal_point)
        rospy.sleep(1)
        try:
            sub = rospy.Subscriber(
                "/far_reach_goal_status", Bool, reach_callback, queue_size=5)
            # print(reach)
        except:
            reach = False
    if(reach == True):
        print("arrvied")
        last_goal = goal
        if count < odom_len-1:
            # print(last_goal._x, last_goal._y)
            # arg[0] = generate_random_goal(pos_list)
            count += 1
            #goal = generate_goal(odom)
            goal = generate_goal_with_order(odom, count)
            print("goal:", goal._x, goal._y, goal._z)
            reach = False
            pub_done.publish(False)
        else:
            pub_done.publish(True)
            rospy.sleep(1)
            end = True
            print("done!! publish the flag")


if __name__ == '__main__':
    count = 0
    reach = False
    last_goal = None
    end = False
    rospy.init_node("goal_pub")
    odom = generate_list_from_odom("./travel_odom/odom_matterport_17r.txt")
    odom_len = len(odom)
    # goal = generate_goal(odom)
    goal = generate_goal_with_order(odom, count)
    # goal = generate_goal_from_list(count)
    print("goal:", goal._x, goal._y, goal._z)
    while rospy.is_shutdown() != 1 and end != True:
        pub = rospy.Publisher("/goal_pub_work", Bool, queue_size=5)
        pub.publish(True)
        send_goal(goal)
    print("done all")
