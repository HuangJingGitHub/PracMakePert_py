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

class SAMFeedback:
    def __init__(self):
        from sam2.build_sam import build_sam2_camera_predictor

        self.sam2_checkpoint = "/root/sam/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt"
        self.sam2_model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml" # use relative path to sam package
        self.predictor = build_sam2_camera_predictor(self.sam2_model_cfg, self.sam2_checkpoint)
        self.initialized = False

    def init_with_points(self, frame, prompt_points, prompt_labels):
        self.predictor.load_first_frame(frame)
        _, self.out_obj_ids, self.out_mask_logits = self.predictor.add_new_prompt(
            0,
            0,
            points=np.array(prompt_points, dtype=np.float32),
            labels=np.array(prompt_labels, dtype=np.int32),
        )
        self.initialized = True
    
    def init_with_center_pts(self, frame, center_point, bbox_length=30):
        self.predictor.load_first_frame(frame)
        np_pt = np.array(center_point, dtype=np.float32)
        bbox = np.array([np_pt - bbox_length, np_pt + bbox_length], dtype=np.float32)
        _, self.out_obj_ids, self.out_mask_logits = self.predictor.add_new_prompt(
            0,
            0,
            bbox=bbox
        )
        self.initialized = True
    
    def init_with_bbox(self, frame, bbox):
        self.predictor.load_first_frame(frame)
        _, self.out_obj_ids, self.out_mask_logits = self.predictor.add_new_prompt(
            0,
            0,
            bbox=bbox
        )
        self.initialized = True

    def track(self, frame):
        if not self.initialized:
            raise RuntimeError("SAMFeedback not initialized.")
        self.out_obj_ids, self.out_mask_logits = self.predictor.track(frame)
        self.mask = (self.out_mask_logits[0][0] > 0).cpu().numpy().astype("uint8")
        
        # color = np.array([100, 100, 100])
        # h, w = self.mask.shape[-2:]
        # mask_image = self.mask.reshape(h, w, 1) * color.reshape(1, 1, -1).astype("uint8")
        return self.mask
