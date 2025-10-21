import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import Delaunay as delaa

pts=np.array([[ 0.,  0.,  0.],
       [ 0.,  0.,  8.],
       [ 0.,  8.,  0.],
       [ 0.,  8.,  8.],
       [ 0., 16.,  0.],
       [ 0., 16.,  8.],
       [ 8.,  0.,  0.],
       [ 8.,  0.,  8.],
       [ 8.,  8.,  0.],
       [ 8.,  8.,  8.],
       [ 8., 16.,  0.],
       [ 8., 16.,  8.],
       [16.,  0.,  0.],
       [16.,  0.,  8.],
       [16.,  8.,  0.],
       [16.,  8.,  8.],
       [16., 16.,  0.],
       [16., 16.,  8.]])
	   
dela=delaa(pts)

def plot_tri_simple(ax, dela):
    for tr in dela.simplices:
        pts = dela.points[tr, :]
        ax.plot3D(pts[[0,1],0], pts[[0,1],1], pts[[0,1],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,2],0], pts[[0,2],1], pts[[0,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[0,3],0], pts[[0,3],1], pts[[0,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,2],0], pts[[1,2],1], pts[[1,2],2], color='g', lw='0.1')
        ax.plot3D(pts[[1,3],0], pts[[1,3],1], pts[[1,3],2], color='g', lw='0.1')
        ax.plot3D(pts[[2,3],0], pts[[2,3],1], pts[[2,3],2], color='g', lw='0.1')

    ax.scatter(dela.points[:,0], dela.points[:,1], dela.points[:,2], color='b')

fig = plt.figure()
ax = plt.axes(projection='3d')
plot_tri_simple(ax, dela)
plt.show()
