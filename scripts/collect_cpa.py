import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import baxter_interface
import copy
import pickle
import time
import xlwt
# Robot initial positions:
# left: 
left_init={'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}
right_init= {'right_s0': 0.8782040010643993, 'right_s1': 0.14150972768242942, 'right_w0': 2.058985712539197, 'right_w1': -1.5707963267946636, 'right_w2': 0.03604854851530722, 'right_e0': -2.138369218312267, 'right_e1': 0.014189322287940077}

DATA_FOLDER='/home/teach/sys_id/cpa_data/'

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
        
    def collect_cpa(self,delta,n_pts,time_stream=1.0,j_idx='right_w1'):
        # we move the right arm
        j0=copy.deepcopy(right_init)
        j_next=copy.deepcopy(j0)
        pt_arr=[]
        rate=rospy.Rate(500)
        for n in range(n_pts):
            # add delta to j_idx
            j_next[j_idx]+=delta
            # move robot to next pose:
            for i in range(3):
                self.right.move_to_joint_positions(j_next)

            # wait until robot stops
            #raw_input("robot static?")
            time.sleep(1)
            print 'Collecting data point: ',n
            
            # collect data for time_stream seconds
            self.GOT_POSE=False
            dpts=[]
            while(not self.GOT_POSE):
                # waiting for new checkerboard data:
                t=1
                #print 'waiting'
            now = rospy.get_rostime()
            dtime=rospy.Duration(time_stream) 
            timer=rospy.get_rostime()-now
            cnt=0
            while(timer<dtime or cnt<30):
                if(self.GOT_POSE):
                    pt=copy.deepcopy(self.pt)
                    dpts.append(pt)
                    cnt+=1
                self.GOT_POSE=False
                rate.sleep()
                timer=rospy.get_rostime()-now    
            # save to csv file:
            self.write_xls(dpts,'P'+str(n))
        return pt_arr
    
    def write_xls(self,dpts,f_name):
        book = xlwt.Workbook(encoding="utf-8")

        sheet1 = book.add_sheet("Sheet1")
        for i in range(len(dpts)):
            sheet1.write(i, 0, dpts[i][0])# row,column,data
            sheet1.write(i, 1, dpts[i][1])# row,column,data
            sheet1.write(i, 2, dpts[i][2])# row,column,data
        book.save(DATA_FOLDER+f_name+'.xls')


if __name__=='__main__':
    bax=BaxterData()
    raw_input('Move right arm?')
    for i in range(3):
        #bax.right.move_to_joint_positions(right_init)
        bax.left.move_to_joint_positions(left_init)
    raw_input('collect data?')
    
    data=bax.collect_cpa(0.08,20)
    #pickle.dump(data,open('cpa_pts_50.p','wb'))
