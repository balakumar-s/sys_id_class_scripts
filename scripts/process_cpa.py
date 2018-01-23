import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import xlwt

DATA_FOLDER='/home/bala/sys_id/cpa_data/'
if __name__=='__main__':
    # read cpa
    dpts=pickle.load( open( "cpa_pts_50.p", "rb" ) )

    xs=[]
    ys=[]
    zs=[]
    for i in range(len(dpts)):
        xs.append(dpts[i][0])
        ys.append(dpts[i][1])
        zs.append(dpts[i][2])

    book = xlwt.Workbook(encoding="utf-8")

    sheet1 = book.add_sheet("Sheet 1")
    for i in range(len(dpts)):
        sheet1.write(i, 0, dpts[i][0])# row,column,data
        sheet1.write(i, 1, dpts[i][1])# row,column,data
        sheet1.write(i, 2, dpts[i][2])# row,column,data
    book.save(DATA_FOLDER+"trial.xls")

    # save to xls
    # plot points:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys,zs=zs,marker='o')
    plt.show()
