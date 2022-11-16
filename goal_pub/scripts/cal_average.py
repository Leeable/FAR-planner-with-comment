import sys
from numpy import *

# def read_data(path=""):
#     time = []
#     time_list = []
#     f = open(path, "r")
#     line = f.readline()
#     if line:
#         time = line.split(",")
#         for item in time:
#             time_list.append(round(float(item), 3))
#     return time_list

def read_data(path=""):
    time = []
    f = open(path, "r")
    for line in f:
        line = line[:5]
        time.append(float(line))
    # print(time)
    return time

file_name = sys.argv[1]
rate = sys.argv[2]
if __name__ == "__main__":
    data = read_data(file_name)
    average = mean(data)
    average*=int(rate)
    print(average)