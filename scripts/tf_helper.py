import PyKDL
import numpy as np

def q_T(q,t):
    T=np.eye(4)
    T[0:3,3]=t
    
    R=PyKDL.Rotation.RPY(q[0],q[1],q[2],q[3])
    for i in range(3):
        for j in range(3):
            T[i,j]=R[i,j]
    return T

def R_q(R):
    
