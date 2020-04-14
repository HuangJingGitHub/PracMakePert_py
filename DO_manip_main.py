#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import rospy
import dvrk
from do_manip.srv import *
import math
import PyKDL
import sys

deg2rad = np.pi / 180
camera2base = np.array([[0, 0,1], [-1,0,0],[0,-1,0]])
psm = dvrk.psm('PSM1')


def get_visual_info():
    rospy.wait_for_service('visual_info_service')
    try:
        srv_client = rospy.ServiceProxy('visual_info_service', visual_info_srv)
        visual_res = srv_client()
        return visual_res
    except rospy.ServiceException, e:
        print('Error in getting response from /do_manip/visual_info_service.')

def rotz(ang_rad):
    rotz = np.array([[math.cos(ang_rad), -math.sin(ang_rad), 0],
                    [math.sin(ang_rad), math.cos(ang_rad), 0],
                    [0, 0, 1]])
    return rotz

def PyKDL_Rotation2np_array(PyKDL_Rotation):
    rotation_np = np.eye(3)
    for i in range(3):
        for j in range(3):
            rotation_np[i, j] = PyKDL_Rotation[i, j]
    return rotation_np


def np_array2PyKDL_Rotation(np_array):
    rot_kdl = PyKDL.Rotation()
    for i in range(3):
        for j in range(3):
            rot_kdl[i, j] = np_array[i, j]
    return rot_kdl

def check_local_contact(visual_info):
    p_C_distance_threshold = 3
    p_C_distance = max(visual_info.contactDistancelr)
    return p_C_distance <= p_C_distance_threshold


def check_safety_constraint(visual_info):
    allowed_angle_El = 60
    allowed_angle_Er = 60
    return (visual_info.deformAngles[0] <= allowed_angle_El and 
            visual_info.deformAngles[1] <= allowed_angle_Er)

def adjust_local_contact(adjustDirection = 0):
    x_t = psm.get_current_position()
    x_t_ori = x_t.M
    x_t_ori_np = PyKDL_Rotation2np_array(x_t_ori)
    step = 0.5 * deg2rad
    delta_ori_np = rotz(step)
    if adjustDirection == 0: # default direction
        x_t_ori_dsr_np = np.matmul(x_t_ori_np, delta_ori_np)
    elif adjustDirection == 1:
        x_t_ori_dsr_np = np.matmul(x_t_ori_np, -delta_ori_np)
    else:
        print('Invalid Argument Given In Function: adjust_local_contact()')
        return
    x_t_ori_dsr_kdl = np_array2PyKDL_Rotation(x_t_ori_dsr_np)
    psm.move(x_t_ori_dsr_kdl)
        

def adjust_safety_constraint(visual_info, adjustDirection = 0):
    # x_t = psm.get_current_position()
    # x_t_pos = x_t.p
    step = 0.005
    diff_lr = visual_info.sl - visual_info.sr
    diff_lr = np.array([diff_lr[0], diff_lr[1]])
    diff_lr = diff_lr / np.linalg.norm(diff_lr)
    ne = np.array([visual_info.ne[0], visual_info.ne[1]])
    gama = 0.1
    motion_direction = diff_lr + gama * ne
    if adjustDirection != 0:
        motion_direction = -motion_direction
    motion_image = np.array([motion_direction[0], motion], [motion_direction[1], [0]])
    motion_base = np.matmul(camera2base, motion_image) * step
    motion_kdl = PyKDL.Vector(motion_base[0][0], motion_base[1][0], motion_base[2][0])
    psm.dmove(motion_kdl)
    


if __name__ == '__main__':
    init_joint_config = np.array([0,0,0,0,0,0])
    psm.move_joint(init_joint_config)

    button = input('Press 1 to start the manipulation\n')
    if button != 1:
        print('Exiting')
        sys.exit()

    while True:
        visual_info = get_visual_info()
        # when deformation control can be performed
        if check_local_contact(visual_info) and check_safety_constraint(visual_info):
            K = 100
            motion_step = 0.005
            Jd_np = np.array([[visual_info.deformJacobian[0], visual_info.deformJacobian[1]]])
            x_dot_image = -np.linalg.pinv(Jd_np) * K * visual_info.featureAngley
            x_dot = np.matmul(camera2base, x_dot_image)
            x_dot_t = x_dot / np.linalg.norm(x_dot) * motion_step
            psm.dmove(x_dot_t)

        while not check_safety_constraint(visual_info):
            if visual_info.deformAngles[0] > visual_info.deformAngles[1]:
                adjust_safety_constraint(visual_info, 0)
            else:
                adjust_safety_constraint(visual_info, 1)
            visual_info = get_visual_info()

        while not check_local_contact(visual_info):
            if visual_info.contactDistancelr[0] > visual_info.contactDistancelr[1]:
                adjust_local_contact(0)
            else:
                adjust_local_contact(1)
            visual_info = get_visual_info()
