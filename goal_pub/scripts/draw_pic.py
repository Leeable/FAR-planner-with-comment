import sys
from numpy import *
import matplotlib.pyplot as plt
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
file_name = sys.argv[1]
rate = sys.argv[2]
is_save = sys.argv[3]
if __name__ == "__main__":
    time = read_data(file_name)
    temp = []
    for i in time:
        a = i*int(rate)
        temp.append(a)
    count = len(temp)  # 获取最小的个数
    y_max = max(temp)+1
    fig, ax = plt.subplots(figsize=(16,6))
    x = range(0, count)
    plt.plot(x, temp, color='r', label='real garage test')
    

    plt.axis([0, count, 0, y_max])
    plt.xlabel("Number", size = '25')
    plt.ylabel("Laser process time[ms]", size='25')
    plt.legend(loc='upper right',prop={"size":20})
    plt.xticks(size='25')
    plt.yticks(size='25')
    plt.grid(x)
    if is_save == "true":
        plt.savefig("/media/lqz/本地磁盘/lqz 小论文图片/MDPI_template/"+file_name+".png", bbox_inches='tight',dpi=400)
    plt.show()