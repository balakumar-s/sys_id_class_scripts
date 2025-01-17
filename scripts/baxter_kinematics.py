#!/usr/bin/python

"""
This is a simplified version of the baxter_kinematics class from the baxter_pykdl package.
"""

import numpy as np
import PyKDL
import rospy
import baxter_interface

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF


class baxter_kinematics(object):

    def __init__(self, rospy_init=False):
        if rospy_init:
            rospy.init_node('baxter_kinematics')
        # KDL chain from URDF
        self._urdf = URDF.from_parameter_server(key='robot_description')
        self._base_link = 'left_arm_mount' #self._urdf.get_root()
        self._tip_link = 'left_wrist'
        self._kdl_tree = kdl_tree_from_urdf_model(self._urdf)
        self._kdl_chain = self._kdl_tree.getChain(self._base_link, self._tip_link)

        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._kdl_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._kdl_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._kdl_chain, self._fk_p_kdl, self._ik_v_kdl)
        
        # Baxter Interface Limb Instances
        self._limb = baxter_interface.Limb('left')
        self._joint_names = self._limb.joint_names()
        self._joint_names = [jn for jn in self._joint_names] # left_e0 is fixed
        self._num_jnts = len(self._joint_names)

    def forward_position_kinematics(self):
        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self._joints_to_kdl(), end_frame)
        #print end_frame
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = PyKDL.ChainIkSolverVel_pinv(self._kdl_chain)
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation is not None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_jnts)
        if seed is not None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self._joints_to_kdl()
        # Make IK Call
        if orientation is not None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_jnts)

        num_tries = 0
        status = self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles)
        if status < 0:
            print "IK solution not found with current joint angles as seed. Trying random seeds..."
        while status < 0 and num_tries < 100:
            seed_array = self._get_random_seed_array()
            status = self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles)
            num_tries += 1
        if status < 0:
            print "No IK solution found."
            return None
        else:
            return np.array(result_angles)

    def print_kdl_chain(self):
        for i in range(self._kdl_chain.getNrOfSegments()):
            segment = self._kdl_chain.getSegment(i)
            segment_name = segment.getName()
            joint = segment.getJoint()
            joint_name = joint.getName()
            joint_type = 'FIXED' if joint.getType() == 8 else 'ROTARY'
            print "* %s" % segment_name
            print " \\"
            print "  --- %s : %s" % (joint_name, joint_type)
            print " /"

    def _joints_to_kdl(self):
        kdl_array = PyKDL.JntArray(self._num_jnts)
        current_angles = self._limb.joint_angles()
        for i, name in enumerate(self._joint_names):
            kdl_array[i] = current_angles[name]
        return kdl_array

    def _get_random_seed_array(self):
        # TODO instead of totally random, maybe it makes more sense to just add noise to a configuration
        # that might work
        seed_array = PyKDL.JntArray(self._num_jnts)
        seed_array[0] = np.random.uniform(-1.7, 1.7)
        seed_array[1] = np.random.uniform(-2.0, 1.0)
        seed_array[2] = np.random.uniform(0.0, 2.5)
        seed_array[3] = np.random.uniform(-3.0, 3.0)
        seed_array[4] = np.random.uniform(-1.57, 1.57)
        seed_array[5] = np.random.uniform(-3.0, 3.0)
        return seed_array
        
