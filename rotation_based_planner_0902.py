#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import dvrk
import math
import PyKDL

pi = np.pi
deg2rad = pi / 180
delta_angle = 1 * deg2rad
delta_x = 0.005
camera2base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def rotz(angle_rad):
    """
    Rotation.RotX, Y, Z returns None object in PyKDL library. Cannot be indexed.
    Use self-defined function instead.
    """
    rotz = np.eye(3)
    rotz = np.array([[math.cos(angle_rad), -math.sin(angle_rad), 0],
                        [math.sin(angle_rad), math.cos(angle_rad), 0],
                        [0, 0, 1]])
    return rotz

def PyKDL_Rotation2np_array(PyKDL_Rotation):
    """ 
    Index PyKDL Rotation element is not convenient.
    Computation is not also well defined in PyKDL, convent to numpy array
    for computation.
    """
    rotation_np = np.eye(3) 
    for i in range(3):
        for j in range(3):
            rotation_np[i, j] = PyKDL_Rotation[i, j]
    
    return rotation_np


def np_array2PyKDL_Rotation(np_array):
    return PyKDL.Rotation(np_array[0,0], np_array[0,1], np_array[0,2],
                            np_array[1,0], np_array[1,1], np_array[1,2],
                            np_array[2,0], np_array[2,1], np_array[2,2])


if __name__ == '__main__':
    psm = dvrk.psm('PSM1')
    # initial_joint_config = np.array([1, 0, 0.1, 0.1, 0, 0.5])
    # initial_joint_config = np.array([ 1.0, -0.11,  0.11, -0.20, -0.13 , -0.20])
    # initial_joint_config = np.array([ 0.823, -0.183,  0.116,  1.306, -0.027, 0.805])
    # initial_joint_config = np.array([ 0.77994196,  0.12682272,  0.11816331,  2.29360951, -0.20266039, 0.50492855])
    initial_joint_config = np.array([0.86180293,  0.22168877,  0.13381731, -1.1603237 , -0.09953447, -1.08623039])
    # end_position array([ 1.0794141 ,  0.24356727,  0.11532444, -1.48058646, -0.12166953, -1.04386594])

    psm.move_joint(initial_joint_config)
    time.sleep(10)

    x_init = psm.get_current_position()
    x_init_position = x_init.p
    x_init_position_np = np.array([[x_init_position.x(), x_init_position.y(), x_init_position.z()]]).T
    # time.sleep(1)

    for i in range(45):
        tip_orientation_PyKDL = psm.get_current_position().M    
        old_orientation = PyKDL_Rotation2np_array(tip_orientation_PyKDL)
        # r = PyKDL.Rotation()
        # rot_z = PyKDL_Rotation2np_array(r.DoRotZ(delta_angle))
        rot_z = rotz(delta_angle)
        rot_z_camera = np.matmul(camera2base, rot_z)
        new_orientation_np = np.matmul(rot_z_camera, old_orientation)
        new_orientation_PyKDL = np_array2PyKDL_Rotation(new_orientation_np)
        # ***** perpendicular method
        """
        angle_t = delta_angle * i
        r_s_perp = np.array([[-math.sin(angle_t), -math.cos(angle_t), 0]]).T
        delta_x_vec_np = delta_x * np.matmul(camera2base, r_s_perp)
        """
        # *****
        # ***** ploygon approximation
        """
        radius = 0.1
        angle_t_next = delta_angle * (i + 1)
        delta_x_np_polygon = radius * (np.array([[math.cos(angle_t_next), -math.sin(angle_t_next), 0]]) -
                                        np.array([[math.cos(angle_t), -math.sin(angle_t), 0]])).T
        delta_x_np_polygon_base = np.matmul(camera2base, delta_x_np_polygon)
        delta_x_polygon_PyKDL = PyKDL.Vector(delta_x_np_polygon_base[0,0], delta_x_np_polygon_base[1,0], delta_x_np_polygon_base[2,0])
        """
        ## ***** 
        ## ***** circular trajectory
        print('Initial positon')
        print(x_init_position_np)
        radius = 0.05
        angle_t = delta_angle * i
        x_disp_np = radius * np.array([[math.cos(angle_t), -math.sin(angle_t), 0]]).T
        x_dsr_base_np = x_init_position_np + np.matmul(camera2base, x_disp_np)
        #x_dsr_base_np = np.matmul(camera2base, x_dsr_np)
        print('New position')
        print(x_dsr_base_np)
        x_dsr_base_PyKDL = PyKDL.Vector(x_dsr_base_np[0,0], x_dsr_base_np[1,0], x_dsr_base_np[2,0])
        ## *****
        # delta_x_vec_PyKDL = PyKDL.Vector(delta_x_vec_np[0,0], delta_x_vec_np[1,0], delta_x_vec_np[2, 0])
        # psm.dmove(delta_x_vec_PyKDL)
        #psm.dmove(delta_x_polygon_PyKDL)
        psm.move(x_dsr_base_PyKDL)
        time.sleep(0.01)
        #psm.move(new_orientation_PyKDL)
        #time.sleep(0.5)
        print(i)

        
