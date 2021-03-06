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
from time import sleep

deg2rad = np.pi / 180
camera2base = np.array([[-0.04508708642250811, -0.9483174140553883, -0.3141038631364326],
                        [-0.6338211728705693, 0.2701892860569093, -0.7247540758912874],
                        [0.7721644096038778, 0.166408629281535, -0.613245703321055]])

psm = dvrk.psm('PSM1')

current_angle = 0
current_sw = np.array([])
current_s3 = np.array([])
current_weightVector = np.array([])
distance_linelrp = np.array([])

log_angle = np.array([])
log_contact_distance = np.array([])
log_sw = np.array([])
log_s3 = np.array([])
log_indicatorw = np.array([])
log_distancew = np.array([])
log_weightVector = np.array([])
log_distanceLinelrp = np.array([])

def get_visual_info():
    rospy.wait_for_service('visual_info_service')
    try:
        srv_client = rospy.ServiceProxy('visual_info_service', visual_info_srv)
        visual_res = srv_client()
        return visual_res
    except rospy.ServiceException, e:
        print('Error in getting response from /do_manip/visual_info_service.')

def process_visual_info(visual_info):
    global log_angle, log_contact_distance, log_sw, log_s3, \
            log_indicatorw, log_distancew, log_weightVector, log_distanceLinelrp, \
            current_angle, current_sw, current_s3, current_weightVector, distance_linelrp
    current_angle = visual_info.featureAngley
    current_sw = np.array([[i for i in visual_info.sw]]).T
    current_s3 = np.array([[i for i in visual_info.s3]]).T
    current_weightVector = np.array([[i for i in visual_info.weightVector]]).T
    distance_linelrp = np.array([[i for i in visual_info.distancelrp]]).T

    log_angle = np.append(log_angle, current_angle)
    log_contact_distance = np.append(log_contact_distance, max(visual_info.contactDistancelr))
    log_sw = np.append(log_sw, current_sw)
    log_s3 = np.append(log_s3, current_s3)
    log_indicatorw = np.append(log_indicatorw, visual_info.indicatorw)
    log_distancew = np.append(log_distancew, visual_info.distancew)
    log_weightVector = np.append(log_weightVector, current_weightVector)
    log_distanceLinelrp = np.append(log_distanceLinelrp, distance_linelrp)   
    return True 

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
    p_C_distance_threshold = 20
    p_C_distance = max(visual_info.contactDistancelr)
    # print(p_C_distance)
    return p_C_distance <= p_C_distance_threshold

def check_manipulability(visual_info):
    area_indicator = visual_info.indicatorw
    pt_area_distance = visual_info.distancew
    if area_indicator == 1 or pt_area_distance <= 5:
        return True
    else:
        return False
    
def check_safety_constraint(visual_info):
    allowed_angle_El = 180
    allowed_angle_Er = 180
    print(visual_info.deformAngles)
    return (visual_info.deformAngles[0] <= allowed_angle_El and 
            visual_info.deformAngles[1] <= allowed_angle_Er)

def adjust_local_contact(adjust_direction = 0):
    step = 0.5 * deg2rad
    if adjust_direction == 0: # default direction
        psm.dmove_joint_one(step, 3)
    elif adjust_direction == 1:
        psm.dmove_joint_one(-step, 3)
    else:
        print('Invalid Argument Given In Function: adjust_local_contact()')
        return
    sleep(0.1)

def adjust_manipulatbility(visual_info):
    step = 0.0005
    sl_list = [[i for i in visual_info.sl]]
    sr_list = [[i for i in visual_info.sr]]
    diff_lr = np.array(sl_list).T - np.array(sr_list).T
    diff_lr = diff_lr / np.linalg.norm(diff_lr)
    ne = np.array([[i for i in visual_info.ne]]).T
    gama = 0.15
    area_indicator = visual_info.indicatorw
    pt_area_distance = visual_info.distancew

    if area_indicator == 2:
        motion_direction = diff_lr - gama * ne 
    elif area_indicator == 3:
        motion_direction = -diff_lr - gama * ne
    
    x_dot_image = np.append(motion_direction, [[0]], axis=0)
    x_dot = np.matmul(camera2base, x_dot_image)
    x_dot_scale = x_dot / np.linalg.norm(x_dot) * step
    x_dot_kdl = PyKDL.Vector(x_dot_scale[0][0], x_dot_scale[1][0], x_dot_scale[2][0])
    psm.dmove(x_dot_kdl)
    sleep(0.1)

def adjust_safety_constraint(visual_info, adjust_direction = 0):
    step = 0.0005
    sl_list = [i for i in visual_info.sl]
    sr_list = [i for i in visual_info.sr]
    diff_lr = np.array(sl_list) - np.array(sr_list)
    diff_lr = diff_lr / np.linalg.norm(diff_lr)
    ne = np.array([i for i in visual_info.ne])
    gama = 0.3
    motion_direction = diff_lr - gama * ne
    motion_direction = motion_direction / np.linalg.norm(motion_direction)
    if adjust_direction != 0:
        motion_direction = -motion_direction
    motion_image = np.array([[motion_direction[0]], [motion_direction[1]], [0]])
    motion_base = np.matmul(camera2base, motion_image) * step
    motion_kdl = PyKDL.Vector(motion_base[0][0], motion_base[1][0], motion_base[2][0])
    psm.dmove(motion_kdl)


if __name__ == '__main__':
    psm.home()
    init_joint_config = np.array([-0.02963985,  0.27637344,  0.10573838,  1.51731643, -0.16724308, 0.19804296]) # normal initial

    psm.move_joint(init_joint_config)

    button = input('Press 1 to start the manipulation\n')
    if button != 1:
        print('Exiting')
        sys.exit()
    
    visual_info = get_visual_info()
    process_visual_info(visual_info)

    while (current_angle > 2):
        visual_info = get_visual_info()
        process_visual_info(visual_info)
                
        ### when deformation control can be performed
        if check_local_contact(visual_info):
            print('Local Contact Valid \nSafety Constraint Satisfied')
            K = 100
            motion_step = 0.0005
            Jd_np = np.array([[visual_info.deformJacobian[0], visual_info.deformJacobian[1]]])
            # x_dot_image = K * np.matmul(np.linalg.pinv(Jd_np), np.array([[current_angle]]))
            x_dot_image = -K * np.matmul(Jd_np.T, np.array([[current_angle]]))

            print('current_angle')
            print(current_angle)
            print('Jd_np')
            print(Jd_np)
            print('x_dot_image')
            print(x_dot_image)

            x_dot_image = np.append(x_dot_image, [[0]], axis=0)
            x_dot = np.matmul(camera2base, x_dot_image)
            x_dot_scale = x_dot / np.linalg.norm(x_dot) * motion_step

            print('x_dot_scale:')
            print(x_dot_scale)
            print('')

            x_dot_kdl = PyKDL.Vector(x_dot_scale[0][0], x_dot_scale[1][0], x_dot_scale[2][0])
            psm.dmove(x_dot_kdl)
            sleep(0.1)

        while not check_local_contact(visual_info):
            print('Local Contact Adjustment')
            if visual_info.contactDistancelr[0] > visual_info.contactDistancelr[1]:
                adjust_local_contact(0)
            else:
                adjust_local_contact(1)
                 
            process_visual_info(visual_info)
            visual_info = get_visual_info()
        
        while not check_manipulability(visual_info):
            print('Manipulability Adjustment')
            adjust_manipulatbility(visual_info)

            process_visual_info(visual_info)
            visual_info = get_visual_info()
            

        while not check_local_contact(visual_info):
            print('Local Contact Adjustment')
            if visual_info.contactDistancelr[0] > visual_info.contactDistancelr[1]:
                adjust_local_contact(0)
            else:
                adjust_local_contact(1)

            process_visual_info(visual_info) 
            visual_info = get_visual_info()

    randIdxStr = str(np.random.randint(0,500))
    np.save('./data/0506/log_angle' + randIdxStr, log_angle)
    np.save('./data/0506/log_contact_distance' + randIdxStr, log_contact_distance)
    np.save('./data/0506/log_sw' + randIdxStr, log_sw)
    np.save('./data/0506/log_s3_' + randIdxStr, log_s3)
    np.save('./data/0506/log_indicatorw' + randIdxStr, log_indicatorw)
    np.save('./data/0506/log_distancew' + randIdxStr, log_distancew)
    np.save('./data/0506/log_weightVector' + randIdxStr, log_weightVector)
    np.save('./data/0506/log_distance_linelrp' + randIdxStr, log_distanceLinelrp)
