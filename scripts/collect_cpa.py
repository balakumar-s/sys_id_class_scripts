import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import baxter_interface
import copy
import pickle
import time
# Robot initial positions:
# left: 
left_init={'left_w0': -2.027922601584517, 'left_w1': 2.0612866837210246, 'left_w2': 1.2678351211872945, 'left_e0': 0.47284957786567877, 'left_e1': 0.6661311571392409, 'left_s0': -0.17755827619773665, 'left_s1': -0.8318010822308656}
right_init= {'right_s0': 0.8782040010643993, 'right_s1': 0.14150972768242942, 'right_w0': 2.058985712539197, 'right_w1': -1.5707963267946636, 'right_w2': 0.03604854851530722, 'right_e0': -2.138369218312267, 'right_e1': 0.014189322287940077}
# enable robot

# enable camera

# calibrate grippers

# move arms to initial robot position

# open right gripper to place checkerboard


# close gripper



# subscribe to checkerboard detector


# collect checkerboard pose

class BaxterData:
    def __init__(self):
        print 'initializing'

        rospy.init_node('data_collection_cpa')

        # subscribe to checkerboard
        rospy.Subscriber("/checkerdetector/objectdetection_pose",PoseStamped, self.pose_cb)
        
        self.pt=np.zeros(3)
        
        self.left=baxter_interface.Limb('left')
        self.right=baxter_interface.Limb('right')
        self.GOT_POSE=False
    def pose_cb(self,msg):
        # return point:
        pt=np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        self.pt=pt
        self.GOT_POSE=True
        
    def collect_cpa(self,delta,n_pts,j_idx='right_w1'):
        # we move the right arm
        j0=copy.deepcopy(right_init)
        j_next=copy.deepcopy(j0)
        pt_arr=[]
        for n in range(n_pts):
            # add delta to j_idx
            j_next[j_idx]+=delta
            # move robot to next pose:
            for i in range(3):
                self.right.move_to_joint_positions(j_next)
            # collect data
            print n
            # wait for 0.5 second
            time.sleep(1.0)
            #raw_input('collect pt?')
            self.GOT_POSE=False
            while(not self.GOT_POSE):
                t=1
                #print 'waiting'
            pt=copy.deepcopy(self.pt)
            pt_arr.append(pt)
        return pt_arr
if __name__=='__main__':
    bax=BaxterData()
    '''
    raw_input('Move left arm?')
    for i in range(3):
        bax.left.move_to_joint_positions(left_init)
    '''
    raw_input('Move right arm?')
    for i in range(3):
        bax.right.move_to_joint_positions(right_init)

    raw_input('collect data?')
    
    data=bax.collect_cpa(0.05,35)
    pickle.dump(data,open('cpa_pts_50.p','wb'))
