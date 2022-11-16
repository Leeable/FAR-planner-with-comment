'''
    这部分代码是point转换为grid坐标
'''

import math


class Point2D:
    def __init__(self, x=0, y=0):
        self._x = x
        self._y = y

    def x(self):
        return self._x

    def y(self):
        return self._y

    def x(self, value):
        self._x = value

    def y(self, value):
        self._y = value


def pos2sub(point=Point2D(), origin=Point2D, resolution_inv=Point2D):
    sub = Point2D(0, 0)
    sub.x(math.ceil((point._x - origin._x)*resolution_inv._x))
    sub.y(math.ceil((point._y - origin._y)*resolution_inv._y))
    return sub


if __name__ == '__main__':
    origin = Point2D(-10, -30)
    resolution = Point2D(6.67, 6.67)
    point = Point2D(0, 0)
    sub = pos2sub(point, origin, resolution)
    print(sub._x, sub._y)
