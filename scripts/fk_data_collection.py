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
left_init={'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}

right_init= {'right_s0': 0.8782040010643993, 'right_s1': 0.14150972768242942, 'right_w0': 2.058985712539197, 'right_w1': -1.5707963267946636, 'right_w2': 0.03604854851530722, 'right_e0': -2.138369218312267, 'right_e1': 0.014189322287940077}

DATA_FOLDER='/home/teach/sys_id/project2/'


class BaxterData:
    def __init__(self):
        print 'initializing'

        rospy.init_node('data_collection_cpa')

        # subscribe to checkerboard
        #rospy.Subscriber("/checkerdetector/objectdetection_pose",PoseStamped, self.pose_cb)
        
        #self.pt=np.zeros(3)
        
        self.left=baxter_interface.Limb('left')
        self.right=baxter_interface.Limb('right')
        self.listener = tf.TransformListener()
        self._joint_names = self.left.joint_names()

        #self.GOT_POSE=False
    def get_js(self,arm='left',dict_=False):
        if(arm=='left'):
            js=self.left.joint_angles()
            kdl_array=np.zeros(7)
            current_angles = self.left.joint_angles()
            for i, name in enumerate(self._joint_names):
                kdl_array[i] = current_angles[name]
        return kdl_array

    def get_ee_pose(self,b_frame='left_arm_mount',e_frame='left_wrist'):
        get_tf=False
        while(not get_tf):
            # listen to tf
            try:
                (trans,rot) = self.listener.lookupTransform(b_frame, e_frame, rospy.Time(0))
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            get_tf=True
        # build T mat:
        print rot
        T=np.eye(4)
        R=tf.transformations.quaternion_matrix(rot)
        T=np.eye(4)
        T=R
        T[0:3,3]=trans
    

        return T

    def store_poses(self,N=5):
        # move robot to home:
        for i in range(3):
            self.left.move_to_joint_positions(left_init)
        pose=self.get_ee_pose()
        print pose
        #print np.linalg.inv(pose)
        # move arm manually:
        n=0
        csvfile=open(DATA_FOLDER+'baxter_fk_poses.csv', 'wb')
        f_writer = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        while(n<N):
            raw_input('Press Enter after moving arm.')
            js=self.get_js()
            pose=self.get_ee_pose()
            dat=np.concatenate((np.ravel(js),np.ravel(pose)),axis=0)
            f_writer.writerow(dat.tolist())
            n+=1

if __name__=='__main__':
    # initialize baxter:
    baxt=BaxterData()

    baxt.store_poses()
