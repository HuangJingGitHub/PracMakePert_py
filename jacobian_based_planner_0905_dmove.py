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
depth = 0.5
alpha = 1
pypr_expd = np.array([[1, 0, 0], [0, 1, 0], [alpha, alpha, 0]])    # partial y partial r expanded
camera_mat_opencv = np.array([[753, 0, 320], [0, 753, 240], [0, 0, 1]])
intrinsic_mat = np.matmul(camera_mat_opencv, np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]))
extrinsic_mat = np.array([[0, 0, 1, -0.05], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
camera_mat = np.matmul(intrinsic_mat, extrinsic_mat.T)
Jd_pre = np.matmul(pypr_expd, camera_mat)
Jd = 1 / depth * np.matmul(Jd_pre, np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]]))
Jd_pinv = np.linalg.pinv(Jd)
K_gian = np.array([[10, 0, 0], [0, 10, 0], [0, 0, 10]])
const_1 = -np.matmul(K_gian, Jd_pinv)

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
    initial_joint_config = np.array([ 0.94918565,  0.3631229 ,  0.15608109, -1.14900552, -0.01348626, -0.30754539])
    initial_joint_config = np.array([ 0.97253147,  0.36555556,  0.15490795, -1.32904852,  0.0589291 , -0.430974  ])
    initial_joint_config = np.array([ 0.94897714,  0.37235157,  0.17860284, -0.93416957, -0.04031219, -0.33554404])


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
    target_region_center = np.array([[visual_info_initial.target_region_center[0], visual_info_initial.target_region_center[1]]]).T
    rotate_radius_error = 10
    counter = 1.0
    log_angle = np.array([])
    log_centroid = np.array([])
    log_rotate_origin = np.array([])
    centroid_error_norm = 100

    while (abs(rotate_angle - 30) > 2 or centroid_error_norm > 20):
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
        rotate_radius_error = np.linalg.norm(rotate_radius - rotate_radius_initial)
        target_region_center = np.array([[visual_info.target_region_center[0], visual_info.target_region_center[1]]]).T    
        angle_error = (rotate_angle - target_angle) / 180 * pi
        centroid_error = centroid - target_region_center
        centroid_error_norm = math.sqrt(centroid_error[0,0]**2 + centroid_error[1,0]**2)
        print(target_region_center)
        print(centroid)
        print(centroid_error)
        print(centroid_error_norm)
        y_error = np.array([[centroid_error[0,0], centroid_error[1,0], angle_error]]).T
        x_dot = np.matmul(const_1, y_error) * 0.005
        x_dot_PyKDL = PyKDL.Vector(x_dot[0,0], x_dot[1,0], x_dot[2,0])
        print('x_dot')
        print(x_dot)

        x_disp_np = delta_x * principal_axis_prep
        # print(x_disp_np)
        x_dsr_base_np = x_init_position_np + np.matmul(camera2base, x_disp_np)
        x_dsr_base_PyKDL = PyKDL.Vector(x_dsr_base_np[0,0], x_dsr_base_np[1,0], x_dsr_base_np[2,0])
        x_disp_base_np = np.matmul(camera2base, x_disp_np)
        x_disp_base_PyKDL = PyKDL.Vector(x_disp_base_np[0,0], x_disp_base_np[1,0], x_disp_base_np[2,0])
        x_rotation_base = np.matmul(camera2base, rotz(-0.02*delta_angle))
        x_rotation_base = np.matmul(x_rotation_base, camera2base.T)
        r = np_array2PyKDL_Rotation(x_rotation_base)

        counter += 1
        if counter < 5:
            rotate_radius_initial += (rotate_radius - rotate_origin_initial) / counter 

        log_angle = np.append(log_angle, rotate_angle)
        log_centroid = np.append(log_centroid, centroid)
        log_rotate_origin = np.append(log_rotate_origin, rotate_origin)

        #psm.dmove(x_disp_base_PyKDL)
        psm.dmove(x_dot_PyKDL)
        if rotate_angle > 15:
            psm.dmove(r)
            print("OK")

    np.save('rotate_angle', log_angle)
    np.save('centroid', log_centroid)
    np.save('rotate_origin', log_rotate_origin)
        # time.sleep(0.05)       

