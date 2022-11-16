import sys
import matplotlib.pyplot as plt
import numpy as np
from numpy import *


file1 = sys.argv[1]
file2 = sys.argv[2]
save_name = sys.argv[3]
def read_data(path=""):
    time = []
    time_list = []
    f = open(path, "r")
    line = f.readline()
    if line:
        time = line.split(",")
        for item in time:
            time_list.append(round(float(item), 3))
    return time_list

if __name__ == '__main__':
    data1 = read_data(file1)
    data2 = read_data(file2)
    mean_far = mean(data1)
    mean_ours = mean(data2)
    print("the mean:",mean_far,mean_ours)
    std_far=std(data1)
    std_ours=std(data2)
    print("the std:",std_far,std_ours)
    count = min(len(data1),len(data2))
    # f = open("./write.txt", "w")
    # f.write(str(data))
    
    fig, ax = plt.subplots(figsize=(16,6))
    
    # for search time
    # ax.set_ylim(10**-1,10**1)
    # ax.set_yscale('log')
    # for laser
    x = range(0, 700)
    data1=data1[:700]
    data2=data2[:700]
    
    # x = range(0, 500)
    # data1=data1[:500]
    # data2=data2[:500]
    # plt.plot(x, data1, color='b', label='Toal time of image process')
    # plt.plot(x, data2, color='orange', label='Time of image blurring')
    plt.plot(x, data1, color='b', label='FAR Planner')
    plt.plot(x, data2, color='r', label='OURS')
    y_max = max(max(data1), max(data2))+2
    plt.axis([0, 500, 0, y_max])
    plt.xlabel("Number", size = '25')
    plt.ylabel("Path search time [ms]", size='25')
    plt.title("Path search time",size = '25')
    plt.legend(loc='upper right',prop={"size":20})
    plt.xticks(size='25')
    plt.yticks(size='25')
    plt.grid(x)
    
    # if(save_name != None):
    #     plt.savefig("/media/lqz/本地磁盘/lqz 小论文图片/MDPI_template/"+save_name+".png", bbox_inches='tight',dpi=400)
    # plt.show()