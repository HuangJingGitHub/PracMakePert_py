import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import Delaunay

pt_num = 5
pts = np.random.rand(pt_num, 3)
# env_vertices = np.array([
#     [0.0, 0.0, 0.0] + np.random.rand(3) * 0.01,
#     [1.0, 0.0, 0.0] + np.random.rand(3) * 0.01,
#     [1.0, 1.0, 0.0] + np.random.rand(3) * 0.01,
#     [0.0, 1.0, 0.0] + np.random.rand(3) * 0.01,
#     [0.0, 0.0, 1.0] + np.random.rand(3) * 0.01,
#     [1.0, 0.0, 1.0] + np.random.rand(3) * 0.01,
#     [1.0, 1.0, 1.0] + np.random.rand(3) * 0.01,
#     [0.0, 1.0, 1.0]+ np.random.rand(3) * 0.01,
# ])
# pts = np.append(pts, env_vertices, axis=0)
pts[:, 0] *= 100
pts[:, 1] *= 200
pts[:, 2] *= 100
dela = Delaunay(pts)
print(f"delaunay meshes:\n{dela.__dict__}")

def plot_tri_simple(ax, dela):
    for tr in dela.simplices:
        pts = dela.points[tr, :]
        ax.plot3D(pts[[0, 1], 0], pts[[0, 1], 1], pts[[0, 1], 2], color="g", lw="0.5")
        ax.plot3D(pts[[0, 2], 0], pts[[0, 2], 1], pts[[0, 2], 2], color="g", lw="0.5")
        ax.plot3D(pts[[0, 3], 0], pts[[0, 3], 1], pts[[0, 3], 2], color="g", lw="0.5")
        ax.plot3D(pts[[1, 2], 0], pts[[1, 2], 1], pts[[1, 2], 2], color="g", lw="0.5")
        ax.plot3D(pts[[1, 3], 0], pts[[1, 3], 1], pts[[1, 3], 2], color="g", lw="0.5")
        ax.plot3D(pts[[2, 3], 0], pts[[2, 3], 1], pts[[2, 3], 2], color="g", lw="0.5")

    ax.scatter(dela.points[:, 0], dela.points[:, 1], dela.points[:, 2], color="b")
    for i, p in enumerate(dela.points):
        ax.text(p[0], p[1], p[2], f"{i}", color="r")


fig = plt.figure()
ax = plt.axes(projection="3d")
plot_tri_simple(ax, dela)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_aspect("equal")
plt.show()
