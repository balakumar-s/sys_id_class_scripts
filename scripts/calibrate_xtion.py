import rospy
import tf
import numpy as np
#import PyKDL
#from tf_helper import *
class dualCalibrate:
    def __init__(self):
        rospy.init_node('tf_pub')
        self.r=rospy.Rate(10)
        self.listener=tf.TransformListener()

    def get_pose(self,b_frame='left_arm_mount',e_frame='marker'):
        get_tf=False
        print 'finding pose',b_frame,e_frame
        while(not get_tf):
            # listen to tf
            try:
                (trans,rot) = self.listener.lookupTransform(b_frame, e_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            get_tf=True

        R=tf.transformations.quaternion_matrix(rot)
        T=np.eye(4)
        T=R
        T[0:3,3]=trans
        return T
    def get_tf_mat(self):
        c_T_mx=self.get_pose('camera_link','ar2_object')
        m_T_b=self.get_pose('ar_object','left_arm_mount')

        # find c_T_b
        c_T_b=np.matrix(c_T_mx)*np.matrix(m_T_b)
        print c_T_b
        # save as a pose to a text file:
        c_T_b=np.matrix(np.linalg.inv(c_T_b))
        pose=np.zeros(7)
        pose[0:3]=np.ravel(c_T_b[0:3,3])
        #print c_T_b
        q=tf.transformations.quaternion_from_matrix(c_T_b)
        #print q
        #q=q/np.linalg.norm(q)
        #print q
        pose[3:7]=q
        print "saving calibration data"
        #print pose
        np.save('tf_pose.npy',pose)
    
    def publish_static_tf(self,f_name=''):
        # read numpy file
        pose=np.load('tf_pose.npy')

        print pose
        # publish as tf
        while(not rospy.is_shutdown()):
            br = tf.TransformBroadcaster()
            br.sendTransform((pose[0],pose[1],pose[2]),(pose[3],pose[4],pose[5],pose[6]),rospy.Time.now(), 'camera_link','left_arm_mount')
            self.r.sleep()


if __name__=='__main__':
    dc=dualCalibrate()
    # calibrate?
    #dc.get_tf_mat()

    # publish?
    dc.publish_static_tf()
