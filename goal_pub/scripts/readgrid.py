'''
    这部分代码是用来读取grid形成的文件，并输出grid
'''

import cv2 as cv
import numpy as np
import math
import rospy
from std_msgs.msg import Bool
import invert


def arr_size(arr, n):
    size = int(len(arr) / n)
    s = []
    for i in range(0, int(len(arr)) + 1, size):
        c = arr[i:i + size]
        if c != []:
            s.append(c)
    if len(s) > 2000:
        n = s[:2000]
    # print(len(n))
        return n
    return s


def openfile(path=""):
    grid = []
    file = open(path, "r")
    line = file.readline()
    print(len(line))
    # line = line.replace('|', '')
    # return line
    line = line[:-1]
    # print(len(line))
    if line:
        item = line.split(',')
        print(len(item))
        for i in item:
            if i == '0':
                grid.append(100)
            if i == '1' or i == '3':
                grid.append(255)
            if i == '2':
                grid.append(0)
    print("num of element:", len(grid))
    num = math.sqrt(len(grid))
    out = arr_size(grid, num)
    return out


def savefile(path='', grid=[]):
    f = open(path, 'w')
    f.writelines(grid)
    f.close()


def inverse(image, point=invert.Point2D):
    dst = cv.bitwise_not(image)
    # dst = cv.resize(dst,(400,400))
    # cv.imwrite("./img.png",dst)
    cv.circle(dst, (point._y, point._x), radius=3,
              color=(0, 0, 255), thickness=-1)
    cv.imshow("inverse demo", dst)
    cv.waitKey(30)


if __name__ == '__main__':
    print("Opencv version:", cv.__version__)
    origin = invert.Point2D(-10, -30)
    resolution_inv = invert.Point2D(6.67, 6.67)
    point = invert.Point2D(18, 40)
    grid = openfile("/home/liqunzhao/far_planner/output_grid.txt")
    # grid = np.zeros((1000,1000))
    while 1:
        # rospy.init_node("pub_grid")
        # pub = rospy.Publisher("/output_grid",Bool,queue_size=5)
        # pub.publish(True)
        try:
            sub = invert.pos2sub(point, origin, resolution_inv)
            # for i in range(-2,2):
            #     for j in range(-2,2):
            #         grid[sub._x+i][sub._y+j] = 255
            # grid[sub._x][sub._y] = 255
            # savefile("./output_grid_modify.txt",grid)
            img = np.array(grid, np.uint8)
            inverse(img, sub)
        except(ValueError):
            print("value error")
            continue
        rospy.sleep(2)
