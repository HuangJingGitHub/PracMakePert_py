import numpy as np
from kd_tree import Node

# -----------------------
# Geometry / collision
# -----------------------
def dist(self, n1: Node, n2: Node):
    return np.linalg.norm(n1.pos - n2.pos)

def get_clearance(self, node, obstacles=None):
    """obstacles: list of (x,y,z,r), shperes"""
    min_dist = float("inf")
    for obs in obstacles:
        obs_center = np.array(obs[:3])
        r = obs[3]
        dist = np.linalg.norm(node.pos - obs_center) - r
        if dist < min_dist:
            min_dist = dist
    return min_dist

def collision(self, node: Node, obstacles=None):
    assert node is not None, "Node is None in collision check"
    for obs in obstacles:
        obs_center = np.array(obs[:3])
        r = obs[3]
        if np.linalg.norm(node.pos - obs_center) <= r:
            return True
    return False

def collision_edge(self, n1: Node, n2: Node, obstacles=None):
    seg = n1.pos - n2.pos
    seg_len = np.linalg.norm(seg)
    if seg_len <= 1e-8:
        return self.collision(n1)
    seg_dir = seg / seg_len

    for obs in obstacles:
        center = obs[:3]
        r = obs[3]
        to_center = center - n1.pos
        t = np.dot(to_center, seg_dir)
        t = np.clip(t, 0.0, seg_len)
        closest_point = n1.pos + t * seg_dir
        dist_to_center = np.linalg.norm(closest_point - center)
        if dist_to_center <= r:
            return True
    return False

def segment_intersects_plane(p1, p2, plane_point, plane_normal, eps=1e-9):
    """
    Check if segment p1->p2 intersects a plane.
    Plane is defined by (plane_point, plane_normal).
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    plane_point = np.asarray(plane_point, dtype=float)
    plane_normal = np.asarray(plane_normal, dtype=float)

    d1 = np.dot(p1 - plane_point, plane_normal)
    d2 = np.dot(p2 - plane_point, plane_normal)

    # Endpoints on opposite sides or touching
    return (d1 * d2 <= eps)

def segment_plane_intersection_point(p1, p2, plane_point, plane_normal, eps=1e-9):
    p1 = np.asarray(p1, float)
    p2 = np.asarray(p2, float)

    d = p2 - p1
    denom = np.dot(d, plane_normal)

    if abs(denom) < eps:
        return None  # Parallel

    t = np.dot(plane_point - p1, plane_normal) / denom
    if 0.0 <= t <= 1.0:
        return p1 + t * d

    return None

def point_in_triangle(p, a, b, c, eps=1e-9):
    v0 = c - a
    v1 = b - a
    v2 = p - a

    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)

    denom = dot00 * dot11 - dot01 * dot01
    if abs(denom) < eps:
        return False

    u = (dot11 * dot02 - dot01 * dot12) / denom
    v = (dot00 * dot12 - dot01 * dot02) / denom

    return (u >= -eps) and (v >= -eps) and (1 -u - v >= -eps)

def segment_triangle_intersection(p1, p2, a, b, c, eps=1e-9):
    # Triangle normal
    ab = b - a
    ac = c - a
    normal = np.cross(ab, ac)
    normal /= np.linalg.norm(normal)

    if not segment_intersects_plane(p1, p2, a, normal, eps):
        return False

    intersection_point = segment_plane_intersection_point(p1, p2, a, normal, eps)
    if intersection_point is None:
        return False

    return point_in_triangle(intersection_point, a, b, c, eps)