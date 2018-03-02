import rospy
import csv
import sys
import numpy as np
from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('ll4ma_kdl')+'/scripts'
sys.path.insert(0,path)
urdf_path=rp.get_path('urlg_robots_description')

from manipulator_model import *

if __name__=='__main__':
    # load robot kdl:
    #urdf_file=urdf_path+'/urdf/lbr4/lbr4_kdl.urdf'
    urdf_file='baxter.urdf'
    robot=ManipulatorSimpleModel(urdf_file,"right_arm_mount","right_wrist")

    print robot.FK(np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0]))[0:3 ,3]
    exit()
    #
    N=500
    csvfile=open('baxter_poses.csv', 'wb')
    spamwriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)

    # get lower and upper limits for baxter:

    j_lower=np.array([-1.70,-2.15,-3.05,-0.05,-3.05,-1.57,-3.05])
    j_upper=np.array([1.70,1.05,3.05,2.62,3.05,2.09,3.05])
    for i in range(N):
        js=np.random.uniform(j_lower,j_upper)
        ee_mat=np.ravel(robot.FK(js))
        dat=np.concatenate((js, ee_mat), axis=0)
        spamwriter.writerow(dat.tolist())
        
    # generate poses


    # store poses 
