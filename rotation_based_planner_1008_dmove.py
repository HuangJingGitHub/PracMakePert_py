#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import rospy
import dvrk
from dvrk_retraction.srv import *
import math
import PyKDL
import sys

pi = np.pi
target_angle = 30
deg2rad = pi / 180
delta_angle = 1 * deg2rad
delta_x = 0.0005
camera2base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])

def get_visual_info():
    rospy.wait_for_service('visual_info_srv')
    try:
        srv_client = rospy.ServiceProxy('visual_info_srv', visual_info_srv)
        visual_res = srv_client()
        print(visual_res.principal_axis)
        print(visual_res.centroid)
        print(visual_res.rotate_angle)
        return visual_res
    except rospy.ServiceException, e:
        print('Error in get response from /dvrk_retraction/visual_info_srv')



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
    #initial_joint_config = np.array([0.86180293,  0.22168877,  0.13381731, -1.1603237 , -0.09953447, -1.08623039]) 1
    # initial_joint_config = np.array([ 0.89911145,  0.27092891,  0.13593156, -0.88407612, -0.11763831, -0.78022419]) 1
    # initial_joint_config = np.array([ 0.43078949,  0.20155561,  0.14573411, -1.17457623,  0.28593595, -0.69887705]) retractor1
    # initial_joint_config = np.array([ 0.43078949,  0.20155561,  0.14573411, -1.17457623,  0.28593595, -0.69887705])
    # initial_joint_config = np.array([ 1.07082642,  0.36412686,  0.17979802, -1.16640199,  0.17136347, -0.41294346])
    # initial_joint_config = np.array([ 0.94918565,  0.3631229 ,  0.15608109, -1.14900552, -0.01348626, -0.30754539])
    initial_joint_config = np.array([ 0.79647633,  0.26209411,  0.16617919,  1.2238531 , -0.00579491, 0.81676057])
    initial_joint_config = np.array([ 0.6124745 ,  0.14229908,  0.12376718,  1.17652799,  0.33095383, 0.18259337])
    initial_joint_config = np.array([ 0.59146094,  0.16227006,  0.12764179,  1.12115439,  0.40017083, 0.14181436])


    # initial_joint_config = np.array([ 0.90556766,  0.22378935,  0.1722925 , -1.24583889,  0.21130919, -0.25821793])

    # end_position array([ 1.0794141 ,  0.24356727,  0.11532444, -1.48058646, -0.12166953, -1.04386594])
    psm.move_joint(initial_joint_config)
    #time.sleep(5)
    # Give time for phantom placement
    button = input("Press 1 to continue\n")
    if button != 1:
        print("Exit")
        sys.exit()

    x_init = psm.get_current_position()
    x_init_position = x_init.p
    x_init_position_np = np.array([[x_init_position.x(), x_init_position.y(), x_init_position.z()]]).T

    visual_info_initial = get_visual_info()
    rotate_angle = visual_info_initial.rotate_angle
    principal_axis = visual_info_initial.principal_axis
    principal_axis_prep = np.array([[principal_axis[1], -principal_axis[0], 0]])
    centroid_initial = np.array([[visual_info_initial.centroid[0], visual_info_initial.centroid[1]]]).T
    rotate_origin_initial = np.array([[visual_info_initial.rotate_origin[0], visual_info_initial.rotate_origin[1]]]).T 
    rotate_radius_initial = centroid_initial - rotate_origin_initial
    rotate_radius_error = 10
    counter = 1.0
    log_angle = np.array([])
    log_centroid = np.array([])

    while (abs(rotate_angle - target_angle) > 1):
        x_init = psm.get_current_position()
        x_init_position = x_init.p
        x_init_position_np = np.array([[x_init_position.x(), x_init_position.y(), x_init_position.z()]]).T
        x_init_rotation = x_init.M 
        x_init_rotation_np = PyKDL_Rotation2np_array(x_init_rotation)

        visual_info = get_visual_info()
        rotate_angle = visual_info.rotate_angle
        principal_axis = visual_info.principal_axis
        principal_axis_prep = np.array([[principal_axis[1], -principal_axis[0], 0]]).T
        centroid = np.array([[visual_info.centroid[0], visual_info.centroid[1]]]).T
        rotate_origin = np.array([[visual_info_initial.rotate_origin[0], visual_info_initial.rotate_origin[1]]]).T 
        rotate_radius = centroid - rotate_origin
        print(rotate_radius) 
        rotate_radius_error = np.linalg.norm(rotate_radius - rotate_radius_initial)    
        
        x_disp_np = delta_x * principal_axis_prep
        print(x_disp_np)
        x_dsr_base_np = x_init_position_np + np.matmul(camera2base, x_disp_np)
        x_dsr_base_PyKDL = PyKDL.Vector(x_dsr_base_np[0,0], x_dsr_base_np[1,0], x_dsr_base_np[2,0])
        x_disp_base_np = np.matmul(camera2base, x_disp_np)
        x_disp_base_PyKDL = PyKDL.Vector(x_disp_base_np[0,0], x_disp_base_np[1,0], x_disp_base_np[2,0])
        x_rotation_base = np.matmul(camera2base, rotz(-0.05*delta_angle))
        x_rotation_base = np.matmul(x_rotation_base, camera2base.T)
        r = np_array2PyKDL_Rotation(x_rotation_base)

        counter += 1
        if counter < 5:
            rotate_radius_initial += (rotate_radius - rotate_origin_initial) / counter 

        log_angle = np.append(log_angle, rotate_angle)
        log_centroid = np.append(log_centroid, centroid)

        psm.dmove(x_disp_base_PyKDL)
        if rotate_angle > 10:
            psm.dmove(r)
            print("Rotation OK")

    np.save('rotate_angle', log_angle)
    np.save('centroid', log_centroid)
        # time.sleep(0.05)       

