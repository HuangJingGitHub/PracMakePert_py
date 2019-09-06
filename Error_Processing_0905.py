import numpy as np
import scipy.io


rotate_angle_20deg_1 = np.load('rotate_angle1.npy')
scipy.io.savemat('rotate_angle_20deg_1.mat', {'angle_20deg_1': rotate_angle_20deg_1})
rotate_angle_30deg_1 = np.load('rotate_angle2.npy')
scipy.io.savemat('rotate_angle_30deg_1.mat', {'angle_30deg_1': rotate_angle_30deg_1})
rotate_angle_30deg_2 = np.load('rotate_angle3.npy')
scipy.io.savemat('rotate_angle_30deg_2.mat', {'angle_30deg_2': rotate_angle_30deg_2})

rotate_origin_20deg_1 = np.load('rotate_origin1.npy')
scipy.io.savemat('rotate_origin_20deg_1.mat', {'rotate_origin_20deg_1': rotate_origin_20deg_1})
rotate_origin_30deg_1 = np.load('rotate_origin2.npy')
scipy.io.savemat('rotate_origin_30deg_1.mat', {'rotate_origin_30deg_1.mat': rotate_origin_30deg_1})
rotate_origin_30deg_2 = np.load('rotate_origin3.npy')
scipy.io.savemat('rotate_origin_30deg_2.mat', {'rotate_origin_30deg_2': rotate_origin_30deg_2})

centroid_20deg_1 = np.load('centroid1.npy')
scipy.io.savemat('centroid_20deg_1.mat', {'centroid_20deg_1': centroid_20deg_1})
centroid_30deg_1 = np.load('centroid2.npy')
scipy.io.savemat('centroid_30deg_1.mat', {'centroid_30deg_1.mat': centroid_30deg_1})
centroid_30deg_2 = np.load('centroid3.npy')
scipy.io.savemat('centroid_30deg_2.mat', {'centroid_30deg_2': centroid_30deg_2})

# data of rotation-based motion planner
rotate_angle_30deg_1_mp = np.load('rotate_angle2_mp.npy')
scipy.io.savemat('rotate_angle_30deg_1_mp.mat', {'angle_30deg_1_mp': rotate_angle_30deg_1_mp})
centroid_30deg_1_mp = np.load('centroid2_mp.npy')
scipy.io.savemat('centroid_30deg_1_mp.mat', {'centroid_30deg_1_mp': centroid_30deg_1_mp})

test = np.load('centroid3_mp.npy')
print(test)
