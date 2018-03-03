import rospy

from geometry_msgs.msg import PoseStamped
import numpy as np
import baxter_interface
import copy
import pickle
import time
import csv
import tf
# Robot initial positions:
# left: 
left_init={'left_w0': -2.027922601584517, 'left_w1': 2.0612866837210246, 'left_w2': 1.2678351211872945, 'left_e0': 0.47284957786567877, 'left_e1': 0.6661311571392409, 'left_s0': -0.17755827619773665, 'left_s1': -0.8318010822308656}
right_init= {'right_s0': 0.8782040010643993, 'right_s1': 0.14150972768242942, 'right_w0': 2.058985712539197, 'right_w1': -1.5707963267946636, 'right_w2': 0.03604854851530722, 'right_e0': -2.138369218312267, 'right_e1': 0.014189322287940077}

DATA_FOLDER='/home/teach/sys_id/project3/'


class BaxterData:
    def __init__(self):
        print 'initializing'

        rospy.init_node('data_collection')

        # subscribe to checkerboard
        #rospy.Subscriber("/checkerdetector/objectdetection_pose",PoseStamped, self.pose_cb)
        
        #self.pt=np.zeros(3)
        
        self.left=baxter_interface.Limb('left')
        self.right=baxter_interface.Limb('right')
        self.listener = tf.TransformListener()

        #self.GOT_POSE=False
    def get_js(self,arm='left',dict_=False):
        if(arm=='left'):
            js=self.left.joint_angles()
        if(dict_==False):
            js=js.values()
        return js

    def get_ee_pose(self,b_frame='left_arm_mount',e_frame='marker'):
        get_tf=False
        while(not get_tf):
            # listen to tf
            try:
                (trans,rot) = self.listener.lookupTransform(b_frame, e_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            get_tf=True

        # build T mat:
        T=np.eye(4)
        R=tf.transformations.quaternion_matrix(rot)
        T=np.eye(4)
        T=R
        T[0:3,3]=trans
    
        return T

    def store_poses(self,N=25):
        # move robot to home:
        #for i in range(3):
        #    self.left.move_to_joint_positions(left_init)

        # move arm manually:
        n=0
        csvfile=open(DATA_FOLDER+'baxter_poses.csv', 'wb')
        f_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        while(n<N):
            raw_input('Press Enter after moving arm.')
            js=self.get_js()
            print js
            pose=self.get_ee_pose()
            dat=np.concatenate((np.ravel(js),np.ravel(pose)),axis=0)
            f_writer.writerow(dat.tolist())
            n=n+1
    def store_auto_poses(self,N=25):
        # move robot to home:
        for i in range(3):
            self.left.move_to_joint_positions(left_init)

        # read js trajectory from file:
        
        # move arm manually:
        n=0
        csvfile=open(DATA_FOLDER+'baxter_poses.csv', 'wb')
        f_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        while(n<N):
            raw_input('Press Enter after moving arm.')
            js=self.get_js
            pose=self.get_ee_pose()
            dat=np.concatenate((np.ravel(js),np.ravel(pose)),axis=0)
            f_writer.writerow(dat.tolist())

    def record_js_poses(self,N=25):
        n=0
        csvfile=open(DATA_FOLDER+'baxter_poses.csv', 'wb')
        f_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        while(n<N):
            raw_input('Press Enter after moving arm.')
            js=np.ravel(self.get_js)
            f_writer.writerow(js.tolist())
        
if __name__=='__main__':
    # initialize baxter:
    baxt=BaxterData()

    # move manually and store poses:
    baxt.store_poses()

    # move from txt and store poses:
    #baxt.store_auto_poses()
