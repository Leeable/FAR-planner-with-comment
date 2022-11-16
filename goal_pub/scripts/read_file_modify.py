import sys
import matplotlib.pyplot as plt
import numpy as np
from numpy import *


file1 = sys.argv[1]
file2 = sys.argv[2]
save_name = sys.argv[3]

def read_data(path=""):
    time = []
    f = open(path, "r")
    for line in f:
        line = line[:5]
        time.append(float(line))
    # print(time)
    return time
if __name__ == '__main__':
    data1 = read_data(file1)
    mean_far = mean(data1)
    data2 = read_data(file2)
    mean_ours = mean(data2)
    print("the mean:",mean_far,mean_ours)
    # print(data1, data2)
    std_far=std(data1)
    std_ours=std(data2)
    print("the std:",std_far,std_ours)
    count = min(len(data1),len(data2))
    fig, ax = plt.subplots(figsize=(16,6))
    x = range(0, count)
    plt.plot(x, data1, color='b', label='FAR Planner')
    plt.plot(x, data2, color='r', label='OURS')
    y_max = max(max(data1), max(data2))+2
    plt.axis([0, count, 0, y_max])
    plt.xlabel("Number", size = '25')
    plt.ylabel("V-graph update time [ms]", size='25')
    plt.title("V-graph update consumption",size = '25')
    plt.legend(loc='upper right',prop={"size":20})
    plt.xticks(size='25')
    plt.yticks(size='25')
    plt.grid(x)
    
    # if(save_name != None):
    #     plt.savefig("/media/lqz/本地磁盘/lqz 小论文图片/MDPI_template/"+save_name+".png", bbox_inches='tight',dpi=400)
    # plt.show()