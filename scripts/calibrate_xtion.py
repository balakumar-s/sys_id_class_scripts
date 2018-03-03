import rospy
import tf
#import PyKDL
#from tf_helper import *
class dualCalibrate:
    def __init__(self):
        self.listener=tf.TransformListener()

    def get_pose(self,b_frame='left_arm_mount',e_frame='marker'):
        get_tf=False
        while(not get_tf):
            # listen to tf
            try:
                (trans,rot) = listener.lookupTransform(e_frame, b_frame, rospy.Time(0))
                get_tf=True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        R=tf.transformations.quaternion_matrix(rot)
        T=np.eye(4)
        T[0:3,0:3]=R
        T[0:3,3]=trans,
        return T
    def get_tf_mat(self):
        m_T_c=get_pose('marker','camera_link')
        b_T_m=get_pose('left_arm_mount','marker')

        # find c_T_b
        c_T_b=m_T_c.inverse()*b_T_m.inverse()

        # save as a pose to a text file:
        pose=np.zeros(7)
        pose[0:3]=c_T_b[0:3,3]
        q=tf.transformations.quaternion_from_matrix(c_T_b[0:3,0:3])
        pose[3:7]=q
        
        np.save('tf_pose.npy',pose)
    
    def publish_static_tf(self,f_name=''):
        # read numpy file
        pose=np.load('tf_pose.npy')


        # publish as tf
        rospy.init_node('tf_pub')
        r=rospy.Rate(10)
        while(not rospy.is_shutdown()):
            br = tf.TransformBroadcaster()
            br.sendTransform((pose[0],pose[1],pose[2]),
                             (pose[3],pose[4],pose[5],pose[6]),
                             rospy.Time.now(),
                             'left_arm_mount', 'camera_link')
            r.sleep()


if __name__=='__main__':
    dc=dualCalibrate()
    # calibrate?
    dc.get_tf_mat()

    # publish?
    dc.publish_static_tf()
