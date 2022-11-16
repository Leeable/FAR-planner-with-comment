import pandas as pd
import numpy as np
import xlwt
def read_data(path=""):
    time = []
    time_list = []
    f = open(path, "r")
    line = f.readline()
    if line:
        time = line.split(",")
        for item in time:
            time_list.append(item)
    return time_list
if __name__ == '__main__':
    path="./test_garage_new1.txt"
    data_re=read_data(path)
    f = xlwt.Workbook("encoding=utf-8")
    sheet1 = f.add_sheet('sheet1',cell_overwrite_ok=True)
    for i in range(len(data_re)):
        sheet1.write(i,0,i+1)
        sheet1.write(i,1,data_re[i])
    f.save('./our_graph_update.xls')
