from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np
from push_rigid_body_forward_model import select_rigid_body_two_stage_push_plan

DEFAULT_MIN_CAM_DEPTH = 1e-4
DEFAULT_MAX_COORD_ABS = 1e6
DEFAULT_TABLE_LOCAL_SURFACE_RADIUS = 0.035
DEFAULT_TABLE_SURFACE_SAMPLE_MAX_POINTS = 80000
DEFAULT_STAGE_POLICY_YAW_SMALL_THRESHOLD_RAD = math.radians(10.0)
DEFAULT_STAGE_POLICY_CENTER_SMALL_THRESHOLD_M = 0.020
DEFAULT_STAGE_POLICY_TRANSLATE_ALIGN_COS_THRESHOLD = math.cos(math.radians(20.0))
DEFAULT_STAGE_POLICY_TRANSLATE_ALIGN_YAW_THRESHOLD_RAD = math.radians(22.0)
DEFAULT_RECT_FIT_QUANTILE_LOW = 0.02
DEFAULT_RECT_FIT_QUANTILE_HIGH = 0.98
DEFAULT_RECT_FIT_SIZE_SAFETY_SCALE = 1.06
DEFAULT_FIT_CLUSTER_DOMINANCE_RATIO = 0.85


def _valid_cam_xyz_mask_map(
    xyz_map: np.ndarray,
    min_depth: float = DEFAULT_MIN_CAM_DEPTH,
    max_abs: float = DEFAULT_MAX_COORD_ABS,
) -> np.ndarray:
    if xyz_map.ndim != 3 or xyz_map.shape[2] != 3:
        raise ValueError("dense_map_cam must have shape [H, W, 3]")
    valid = np.all(np.isfinite(xyz_map), axis=2)
    valid &= np.max(np.abs(xyz_map), axis=2) <= float(max_abs)
    valid &= xyz_map[..., 2] > float(min_depth)
    return valid


def _transform_points_cam_to_base(points: np.ndarray, rotation: np.ndarray, shift: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3:
        return np.zeros((0, 3), dtype=np.float32)
    valid = np.all(np.isfinite(pts), axis=1)
    valid &= np.max(np.abs(pts), axis=1) <= float(DEFAULT_MAX_COORD_ABS)
    pts = pts[valid]
    if pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)

    rot = np.asarray(rotation, dtype=np.float64)
    sh = np.asarray(shift, dtype=np.float64).reshape(-1)
    if rot.shape != (3, 3) or sh.shape != (3,):
        return np.zeros((0, 3), dtype=np.float32)
    if not np.all(np.isfinite(rot)) or not np.all(np.isfinite(sh)):
        return np.zeros((0, 3), dtype=np.float32)

    out = np.asarray(pts, dtype=np.float64) @ rot.T + sh.reshape((1, 3))
    out = out[np.all(np.isfinite(out), axis=1)]
    if out.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return out.astype(np.float32, copy=False)


def _normalize_rect_yaw(yaw: float) -> float:
    period = math.pi
    value = float(yaw) % period
    if value < 0.0:
        value += period
    return value


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _shortest_rect_yaw_delta(start_yaw: float, goal_yaw: float) -> float:
    direct = _wrap_to_pi(float(goal_yaw) - float(start_yaw))
    flipped = _wrap_to_pi(float(goal_yaw) + math.pi - float(start_yaw))
    return direct if abs(direct) <= abs(flipped) else flipped


def _fit_rectangle_pose_base(points_base: np.ndarray) -> dict[str, Any]:
    pts = np.asarray(points_base, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 4:
        raise RuntimeError("Need at least 4 valid base-frame target points to fit a rectangle.")

    xy = pts[:, :2].astype(np.float64, copy=False)
    z_ref = float(np.median(pts[:, 2].astype(np.float64, copy=False)))

    center_seed = np.median(xy, axis=0)
    centered = xy - center_seed.reshape((1, 2))
    cov = centered.T @ centered / float(max(1, xy.shape[0]))
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    main_axis = eigvecs[:, 0]
    yaw = _normalize_rect_yaw(math.atan2(float(main_axis[1]), float(main_axis[0])))

    def _project_with_yaw(yaw_rad: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
        ex = np.array([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=np.float64)
        ey = np.array([-math.sin(yaw_rad), math.cos(yaw_rad)], dtype=np.float64)
        proj_x = xy @ ex
        proj_y = xy @ ey
        x0, x1 = np.quantile(proj_x, [DEFAULT_RECT_FIT_QUANTILE_LOW, DEFAULT_RECT_FIT_QUANTILE_HIGH])
        y0, y1 = np.quantile(proj_y, [DEFAULT_RECT_FIT_QUANTILE_LOW, DEFAULT_RECT_FIT_QUANTILE_HIGH])
        length = float(x1 - x0) * float(DEFAULT_RECT_FIT_SIZE_SAFETY_SCALE)
        width = float(y1 - y0) * float(DEFAULT_RECT_FIT_SIZE_SAFETY_SCALE)
        center_xy = ex * (0.5 * (float(x1) + float(x0))) + ey * (0.5 * (float(y1) + float(y0)))
        return ex, ey, center_xy.astype(np.float64), length, width

    ex, ey, center_xy, length, width = _project_with_yaw(yaw)
    if width > length:
        yaw = _normalize_rect_yaw(yaw + 0.5 * math.pi)
        ex, ey, center_xy, length, width = _project_with_yaw(yaw)

    aspect_ratio = float(length / max(width, 1e-6))
    yaw_ambiguous = bool(aspect_ratio < 1.05)
    center_base = np.array([float(center_xy[0]), float(center_xy[1]), z_ref], dtype=np.float32)

    return {
        "center_base": center_base,
        "yaw": float(yaw),
        "length": float(length),
        "width": float(width),
        "aspect_ratio": aspect_ratio,
        "yaw_ambiguous": yaw_ambiguous,
        "principal_axis_base_xy": [float(ex[0]), float(ex[1])],
        "secondary_axis_base_xy": [float(ey[0]), float(ey[1])],
    }


def _erode_mask_for_fit(mask: np.ndarray) -> tuple[np.ndarray, int]:
    m = np.asarray(mask, dtype=bool)
    if m.ndim != 2 or not np.any(m):
        return m.copy(), 0
    ys, xs = np.nonzero(m)
    bbox_h = int(np.max(ys) - np.min(ys) + 1)
    bbox_w = int(np.max(xs) - np.min(xs) + 1)
    # Use a gentler trim to avoid over-shrinking contact geometry on small objects.
    erode_px = int(np.clip(round(0.008 * float(min(bbox_h, bbox_w))), 0, 3))
    if erode_px <= 0:
        return m.copy(), 0
    kernel_size = int(2 * erode_px + 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    eroded = cv2.erode(m.astype(np.uint8), kernel, iterations=1) > 0
    if int(np.count_nonzero(eroded)) < 24:
        return m.copy(), 0
    return eroded, erode_px


def _keep_largest_xy_component(points_base: np.ndarray, voxel_size: float) -> tuple[np.ndarray, dict[str, Any]]:
    pts = np.asarray(points_base, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32), {
            "component_count": 0,
            "largest_component_point_count": 0,
            "largest_component_ratio": 0.0,
            "voxel_size_m": float(voxel_size),
            "used": False,
        }

    xy = pts[:, :2].astype(np.float64, copy=False)
    origin = np.min(xy, axis=0)
    voxel = max(1e-4, float(voxel_size))
    coords = np.floor((xy - origin.reshape((1, 2))) / voxel).astype(np.int32)
    unique_coords, inverse = np.unique(coords, axis=0, return_inverse=True)
    voxel_point_counts = np.bincount(inverse, minlength=unique_coords.shape[0]).astype(np.int32)
    coord_to_idx = {(int(c[0]), int(c[1])): int(idx) for idx, c in enumerate(unique_coords)}
    visited = np.zeros((unique_coords.shape[0],), dtype=bool)
    components: list[tuple[list[int], int]] = []
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    for start_idx in range(unique_coords.shape[0]):
        if visited[start_idx]:
            continue
        stack = [int(start_idx)]
        visited[start_idx] = True
        voxel_indices: list[int] = []
        point_count = 0
        while stack:
            cur = stack.pop()
            voxel_indices.append(cur)
            point_count += int(voxel_point_counts[cur])
            cx, cy = unique_coords[cur]
            for dx, dy in neighbors:
                nxt = coord_to_idx.get((int(cx + dx), int(cy + dy)))
                if nxt is None or visited[nxt]:
                    continue
                visited[nxt] = True
                stack.append(int(nxt))
        components.append((voxel_indices, point_count))

    if not components:
        return pts.copy(), {
            "component_count": 0,
            "largest_component_point_count": int(pts.shape[0]),
            "largest_component_ratio": 1.0 if int(pts.shape[0]) > 0 else 0.0,
            "voxel_size_m": float(voxel),
            "used": False,
        }

    best_voxels, best_count = max(components, key=lambda item: item[1])
    keep_voxel_mask = np.zeros((unique_coords.shape[0],), dtype=bool)
    keep_voxel_mask[np.asarray(best_voxels, dtype=np.int32)] = True
    keep_point_mask = keep_voxel_mask[inverse]
    filtered = pts[keep_point_mask]
    best_ratio = float(best_count) / float(max(1, pts.shape[0]))
    return filtered.astype(np.float32, copy=False), {
        "component_count": int(len(components)),
        "largest_component_point_count": int(best_count),
        "largest_component_ratio": float(best_ratio),
        "voxel_size_m": float(voxel),
        "used": True,
    }


def _rot2(yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def _heading_candidates_from_rect_yaw(rect_yaw: float) -> tuple[float, float]:
    heading_a = _wrap_to_pi(float(rect_yaw))
    heading_b = _wrap_to_pi(float(rect_yaw) + math.pi)
    return heading_a, heading_b


def _tip_heading_preference_cost(heading_rad: float) -> float:
    # Penalize tip +x pointing into the base +X hemisphere; prefer base -X.
    return max(0.0, math.cos(float(heading_rad)))


def _tip_heading_negative_x_penalty(heading_rad: float) -> float:
    x_value = math.cos(float(heading_rad))
    if x_value <= 1e-6:
        return 0.0
    return 100.0 + 10.0 * float(x_value)


def _select_tip_heading_sequence(rect_yaws: np.ndarray) -> np.ndarray:
    yaws = np.asarray(rect_yaws, dtype=np.float64).reshape(-1)
    if yaws.size == 0:
        return np.zeros((0,), dtype=np.float32)
    candidate_pairs = [_heading_candidates_from_rect_yaw(float(yaw_i)) for yaw_i in yaws]
    pref_weight = 0.22
    trans_weight = 1.0
    dp = np.full((len(candidate_pairs), 2), np.inf, dtype=np.float64)
    prev = np.full((len(candidate_pairs), 2), -1, dtype=np.int32)
    for state in range(2):
        dp[0, state] = pref_weight * _tip_heading_preference_cost(candidate_pairs[0][state])
    for idx in range(1, len(candidate_pairs)):
        for state in range(2):
            heading_i = candidate_pairs[idx][state]
            base_cost = pref_weight * _tip_heading_preference_cost(heading_i)
            for prev_state in range(2):
                heading_prev = candidate_pairs[idx - 1][prev_state]
                trans_cost = trans_weight * abs(_wrap_to_pi(float(heading_i - heading_prev)))
                total = float(dp[idx - 1, prev_state]) + base_cost + trans_cost
                if total < float(dp[idx, state]):
                    dp[idx, state] = total
                    prev[idx, state] = int(prev_state)
    states = np.zeros((len(candidate_pairs),), dtype=np.int32)
    states[-1] = int(np.argmin(dp[-1]))
    for idx in range(len(candidate_pairs) - 1, 0, -1):
        states[idx - 1] = int(prev[idx, states[idx]])
    out = np.array([candidate_pairs[idx][int(state)] for idx, state in enumerate(states)], dtype=np.float32)
    return out


def _select_tip_heading_equivalent_sequence(raw_headings: np.ndarray) -> np.ndarray:
    headings = np.asarray(raw_headings, dtype=np.float64).reshape(-1)
    if headings.size == 0:
        return np.zeros((0,), dtype=np.float32)
    candidate_pairs = [(_wrap_to_pi(float(h)), _wrap_to_pi(float(h) + math.pi)) for h in headings]
    dp = np.full((len(candidate_pairs), 2), np.inf, dtype=np.float64)
    prev = np.full((len(candidate_pairs), 2), -1, dtype=np.int32)
    large_jump_threshold = math.radians(60.0)
    large_jump_penalty = 250.0
    hemisphere_pref_weight = 0.08
    for state in range(2):
        dp[0, state] = hemisphere_pref_weight * _tip_heading_negative_x_penalty(candidate_pairs[0][state])
    for idx in range(1, len(candidate_pairs)):
        for state in range(2):
            heading_i = candidate_pairs[idx][state]
            base_cost = hemisphere_pref_weight * _tip_heading_negative_x_penalty(float(heading_i))
            for prev_state in range(2):
                heading_prev = candidate_pairs[idx - 1][prev_state]
                jump = abs(_wrap_to_pi(float(heading_i - heading_prev)))
                trans_cost = float(jump) + 2.2 * float(jump * jump)
                if jump > large_jump_threshold:
                    trans_cost += float(large_jump_penalty) * float(jump - large_jump_threshold)
                total = float(dp[idx - 1, prev_state]) + base_cost + trans_cost
                if total < float(dp[idx, state]):
                    dp[idx, state] = total
                    prev[idx, state] = int(prev_state)
    states = np.zeros((len(candidate_pairs),), dtype=np.int32)
    states[-1] = int(np.argmin(dp[-1]))
    for idx in range(len(candidate_pairs) - 1, 0, -1):
        states[idx - 1] = int(prev[idx, states[idx]])
    out = np.array([candidate_pairs[idx][int(state)] for idx, state in enumerate(states)], dtype=np.float32)
    return out.astype(np.float32, copy=False)


def _tip_frame_row_from_center_heading(center_xyz: np.ndarray, heading_rad: float) -> np.ndarray:
    center = np.asarray(center_xyz, dtype=np.float32).reshape(-1)
    if center.shape[0] != 3:
        raise ValueError("center_xyz must be a 3D point.")
    x_axis = np.array([math.cos(float(heading_rad)), math.sin(float(heading_rad)), 0.0], dtype=np.float64)
    z_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    y_axis = np.cross(z_axis, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-8:
        raise RuntimeError("Failed to build a valid tip y-axis.")
    x_axis = x_axis / max(float(np.linalg.norm(x_axis)), 1e-8)
    y_axis = y_axis / y_norm
    row = np.concatenate([center.astype(np.float64), x_axis, y_axis, z_axis], axis=0)
    return row.astype(np.float32, copy=False)


def _tip_frame_row_with_origin(frame_row: np.ndarray, origin_xyz: np.ndarray) -> np.ndarray:
    row = np.asarray(frame_row, dtype=np.float32).reshape(-1)
    origin = np.asarray(origin_xyz, dtype=np.float32).reshape(-1)
    if row.shape[0] < 12 or origin.shape[0] != 3:
        raise ValueError("Invalid frame row or origin.")
    out = row[:12].astype(np.float32, copy=True)
    out[:3] = origin[:3]
    return out


def _rotation_matrix_to_quaternion_xyzw(rot: np.ndarray) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64)
    if r.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3.")
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-8)
    return quat.astype(np.float32, copy=False)


def _tip_frame_rows_to_xquat(frames_base: np.ndarray) -> np.ndarray:
    frames = np.asarray(frames_base, dtype=np.float32)
    if frames.ndim != 2 or frames.shape[1] < 12:
        return np.zeros((0, 7), dtype=np.float32)
    rows: list[np.ndarray] = []
    prev_quat: np.ndarray | None = None
    for row in frames:
        origin = row[:3].astype(np.float32, copy=False)
        x_axis = row[3:6].astype(np.float64, copy=False)
        y_axis = row[6:9].astype(np.float64, copy=False)
        z_axis = row[9:12].astype(np.float64, copy=False)
        rot = np.stack([x_axis, y_axis, z_axis], axis=1)
        quat = _rotation_matrix_to_quaternion_xyzw(rot)
        if prev_quat is not None and float(np.dot(prev_quat.astype(np.float64), quat.astype(np.float64))) < 0.0:
            quat = (-quat).astype(np.float32, copy=False)
        rows.append(np.concatenate([origin, quat], axis=0).astype(np.float32, copy=False))
        prev_quat = quat.astype(np.float32, copy=False)
    if not rows:
        return np.zeros((0, 7), dtype=np.float32)
    return np.vstack(rows).astype(np.float32, copy=False)


def _normalize_xy_vector(vec_xy: np.ndarray, fallback_xy: np.ndarray | None = None) -> np.ndarray:
    vec = np.asarray(vec_xy, dtype=np.float64).reshape(-1)
    if vec.shape[0] >= 2 and np.all(np.isfinite(vec[:2])):
        out = vec[:2].astype(np.float64, copy=False)
        norm = float(np.linalg.norm(out))
        if norm > 1e-8:
            return (out / norm).astype(np.float64, copy=False)
    if fallback_xy is not None:
        fb = np.asarray(fallback_xy, dtype=np.float64).reshape(-1)
        if fb.shape[0] >= 2 and np.all(np.isfinite(fb[:2])):
            out = fb[:2].astype(np.float64, copy=False)
            norm = float(np.linalg.norm(out))
            if norm > 1e-8:
                return (out / norm).astype(np.float64, copy=False)
    return np.array([1.0, 0.0], dtype=np.float64)


def _surface_z_from_plane_at_xy(plane_normal: np.ndarray, plane_d: float, xy: np.ndarray) -> float | None:
    normal = np.asarray(plane_normal, dtype=np.float64).reshape(-1)
    q = np.asarray(xy, dtype=np.float64).reshape(-1)
    if normal.shape[0] != 3 or q.shape[0] != 2 or not np.all(np.isfinite(normal)) or not np.isfinite(float(plane_d)):
        return None
    if abs(float(normal[2])) <= 1e-6:
        return None
    z = -(float(normal[0]) * float(q[0]) + float(normal[1]) * float(q[1]) + float(plane_d)) / float(normal[2])
    if not np.isfinite(z):
        return None
    return float(z)


def _subsample_points(points: np.ndarray, max_points: int, seed: int = 0) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] <= int(max_points):
        return pts.astype(np.float32, copy=False)
    rng = np.random.default_rng(int(seed))
    pick = np.sort(rng.choice(pts.shape[0], size=int(max_points), replace=False))
    return pts[pick].astype(np.float32, copy=False)


def _extract_local_table_support_points_base(
    dense_map_cam: np.ndarray,
    target_mask: np.ndarray,
    rotation_cam_to_base: np.ndarray,
    shift_cam_to_base: np.ndarray,
    plane_normal_base: np.ndarray | None,
    plane_d_base: float | None,
    plane_inlier_threshold_m: float,
) -> np.ndarray:
    dense = np.asarray(dense_map_cam, dtype=np.float32)
    mask = np.asarray(target_mask, dtype=bool)
    if dense.ndim != 3 or dense.shape[2] != 3 or mask.shape != dense.shape[:2]:
        return np.zeros((0, 3), dtype=np.float32)
    if plane_normal_base is None or plane_d_base is None:
        return np.zeros((0, 3), dtype=np.float32)
    valid = _valid_cam_xyz_mask_map(dense)
    support_valid = np.logical_and(valid, ~mask)
    if not np.any(support_valid):
        return np.zeros((0, 3), dtype=np.float32)
    pts_cam = dense[support_valid].astype(np.float32, copy=False)
    pts_base = _transform_points_cam_to_base(pts_cam, rotation_cam_to_base, shift_cam_to_base)
    if pts_base.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    normal = np.asarray(plane_normal_base, dtype=np.float64).reshape(-1)
    if normal.shape[0] == 3 and np.all(np.isfinite(normal)) and np.isfinite(float(plane_d_base)):
        dist = np.abs(pts_base.astype(np.float64) @ normal + float(plane_d_base))
        thr = max(0.006, float(plane_inlier_threshold_m) * 1.5)
        keep = np.isfinite(dist) & (dist <= thr)
        if np.any(keep):
            pts_base = pts_base[keep]
    if pts_base.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return _subsample_points(pts_base, max_points=DEFAULT_TABLE_SURFACE_SAMPLE_MAX_POINTS, seed=0)


def _query_surface_z_sequence_base(
    xy_points: np.ndarray,
    local_table_points_base: np.ndarray,
    plane_normal_base: np.ndarray | None,
    plane_d_base: float | None,
    search_radius_m: float,
    min_surface_z: float | None,
) -> np.ndarray:
    xy = np.asarray(xy_points, dtype=np.float32)
    if xy.ndim != 2 or xy.shape[1] != 2 or xy.shape[0] == 0:
        return np.zeros((0,), dtype=np.float32)
    local_pts = np.asarray(local_table_points_base, dtype=np.float32)
    have_local = local_pts.ndim == 2 and local_pts.shape[1] == 3 and local_pts.shape[0] > 0
    radius2 = float(max(0.005, float(search_radius_m)) ** 2)
    out = np.zeros((xy.shape[0],), dtype=np.float32)
    for idx, q in enumerate(xy.astype(np.float64, copy=False)):
        z_val: float | None = None
        if have_local:
            dxy = local_pts[:, :2].astype(np.float64, copy=False) - q.reshape((1, 2))
            dist2 = np.sum(dxy * dxy, axis=1)
            near = np.isfinite(dist2) & (dist2 <= radius2)
            if np.any(near):
                local_z = local_pts[near, 2].astype(np.float64, copy=False)
                if local_z.size > 0 and np.any(np.isfinite(local_z)):
                    z_val = float(np.quantile(local_z[np.isfinite(local_z)], 0.25))
        if z_val is None:
            z_val = _surface_z_from_plane_at_xy(
                np.asarray(plane_normal_base, dtype=np.float64)
                if plane_normal_base is not None
                else np.zeros((3,), dtype=np.float64),
                float(plane_d_base) if plane_d_base is not None else float("nan"),
                q,
            )
        if z_val is None or not np.isfinite(z_val):
            z_val = float(min_surface_z) if min_surface_z is not None and np.isfinite(min_surface_z) else float("nan")
        if min_surface_z is not None and np.isfinite(min_surface_z):
            z_val = max(float(min_surface_z), float(z_val))
        out[idx] = float(z_val)
    return out.astype(np.float32, copy=False)


def _overwrite_rectangle_z(rectangles_base: list[np.ndarray], z_values: np.ndarray) -> list[np.ndarray]:
    zs = np.asarray(z_values, dtype=np.float32).reshape(-1)
    out: list[np.ndarray] = []
    for idx, rect in enumerate(rectangles_base):
        corners = np.asarray(rect, dtype=np.float32).copy()
        if corners.ndim == 2 and corners.shape[1] == 3 and idx < zs.shape[0]:
            corners[:, 2] = float(zs[idx])
        out.append(corners)
    return out


def _smoothstep01(alpha: np.ndarray) -> np.ndarray:
    a = np.clip(np.asarray(alpha, dtype=np.float64), 0.0, 1.0)
    return a * a * (3.0 - 2.0 * a)


def _choose_contact_face(
    start_center_xy: np.ndarray,
    goal_center_xy: np.ndarray,
    start_yaw: float,
    yaw_delta: float,
    length: float,
    width: float,
) -> dict[str, Any]:
    faces = [
        {
            "name": "+x",
            "normal_local": np.array([1.0, 0.0], dtype=np.float64),
            "tangent_local": np.array([0.0, 1.0], dtype=np.float64),
            "face_center_local": np.array([0.5 * length, 0.0], dtype=np.float64),
            "edge_half": 0.5 * width,
            "torque_alpha_sign": 1.0,
        },
        {
            "name": "-x",
            "normal_local": np.array([-1.0, 0.0], dtype=np.float64),
            "tangent_local": np.array([0.0, 1.0], dtype=np.float64),
            "face_center_local": np.array([-0.5 * length, 0.0], dtype=np.float64),
            "edge_half": 0.5 * width,
            "torque_alpha_sign": -1.0,
        },
        {
            "name": "+y",
            "normal_local": np.array([0.0, 1.0], dtype=np.float64),
            "tangent_local": np.array([1.0, 0.0], dtype=np.float64),
            "face_center_local": np.array([0.0, 0.5 * width], dtype=np.float64),
            "edge_half": 0.5 * length,
            "torque_alpha_sign": -1.0,
        },
        {
            "name": "-y",
            "normal_local": np.array([0.0, -1.0], dtype=np.float64),
            "tangent_local": np.array([1.0, 0.0], dtype=np.float64),
            "face_center_local": np.array([0.0, -0.5 * width], dtype=np.float64),
            "edge_half": 0.5 * length,
            "torque_alpha_sign": 1.0,
        },
    ]

    dxy = np.asarray(goal_center_xy, dtype=np.float64) - np.asarray(start_center_xy, dtype=np.float64)
    trans_norm = float(np.linalg.norm(dxy))
    dxy_unit = dxy / max(trans_norm, 1e-8)
    rot_start = _rot2(start_yaw)
    trans_scale = max(float(length), float(width), 1e-3)
    rot_ratio = abs(float(yaw_delta)) / max(abs(float(yaw_delta)) + trans_norm / trans_scale, 1e-6)

    best: dict[str, Any] | None = None
    for face in faces:
        push_dir_world = rot_start @ (-face["normal_local"])
        trans_score = float(np.dot(dxy_unit, push_dir_world)) if trans_norm > 1e-6 else 0.0
        rot_cap = float(face["edge_half"] / max(0.5 * max(length, width), 1e-6))

        if abs(float(yaw_delta)) < math.radians(4.0):
            alpha = 0.0
        else:
            alpha_mag = float(np.clip(0.10 + 0.75 * rot_ratio, 0.10, 0.85))
            alpha = math.copysign(1.0, float(yaw_delta)) * float(face["torque_alpha_sign"]) * alpha_mag

        score = 0.85 * trans_score + 0.45 * rot_cap
        if trans_norm <= 1e-6 and abs(float(yaw_delta)) > math.radians(4.0):
            score = 0.25 * trans_score + 0.95 * rot_cap
        if best is None or score > float(best["score"]):
            best = {
                **face,
                "alpha": float(alpha),
                "score": float(score),
                "push_dir_world_start": push_dir_world.astype(np.float64),
            }

    if best is None:
        raise RuntimeError("Failed to choose a valid push face.")
    return best


def _rectangle_corners_from_pose(center_xyz: np.ndarray, yaw: float, length: float, width: float) -> np.ndarray:
    cx, cy, cz = [float(v) for v in np.asarray(center_xyz, dtype=np.float32).reshape(-1)[:3]]
    local = np.array(
        [
            [0.5 * length, 0.5 * width],
            [0.5 * length, -0.5 * width],
            [-0.5 * length, -0.5 * width],
            [-0.5 * length, 0.5 * width],
        ],
        dtype=np.float64,
    )
    corners_xy = local @ _rot2(yaw).T + np.array([cx, cy], dtype=np.float64).reshape((1, 2))
    z_col = np.full((4, 1), cz, dtype=np.float64)
    return np.hstack([corners_xy, z_col]).astype(np.float32)


def _tip_rectangle_from_pose(center_xyz: np.ndarray, yaw: float, tip_length: float, tip_width: float) -> np.ndarray:
    return _rectangle_corners_from_pose(center_xyz=center_xyz, yaw=yaw, length=tip_length, width=tip_width)


def _long_edge_face_spec(face: str, object_width: float) -> tuple[np.ndarray, float, float]:
    if face == "+y":
        return np.array([0.0, 1.0], dtype=np.float64), 0.5 * float(object_width), -1.0
    if face == "-y":
        return np.array([0.0, -1.0], dtype=np.float64), -0.5 * float(object_width), 1.0
    raise ValueError(f"Unsupported long-edge face: {face}")


def _contact_face_edge_half(face: str, object_length: float, object_width: float) -> float:
    if face in {"+x", "-x"}:
        return 0.5 * float(object_width)
    if face in {"+y", "-y"}:
        return 0.5 * float(object_length)
    raise ValueError(f"Unsupported contact face: {face}")


def _preferred_translation_face(goal_yaw: float, trans_vec_xy: np.ndarray) -> str:
    trans_vec = np.asarray(trans_vec_xy, dtype=np.float64).reshape(-1)
    if trans_vec.shape[0] != 2:
        return "+y"
    trans_norm = float(np.linalg.norm(trans_vec))
    if trans_norm <= 1e-8:
        return "+y"
    trans_dir = trans_vec / trans_norm
    normal_goal_a = _rot2(float(goal_yaw)) @ np.array([0.0, 1.0], dtype=np.float64)
    normal_goal_b = -normal_goal_a
    return "+y" if float(np.dot(trans_dir, -normal_goal_a)) >= float(np.dot(trans_dir, -normal_goal_b)) else "-y"


def _decide_stage_policy(
    trans_vec_xy: np.ndarray,
    start_yaw: float,
    goal_yaw: float,
    yaw_delta: float,
    current_yaw_ambiguous: bool,
) -> dict[str, Any]:
    trans_vec = np.asarray(trans_vec_xy, dtype=np.float64).reshape(-1)
    center_error_m = float(np.linalg.norm(trans_vec)) if trans_vec.shape[0] == 2 else float("inf")
    yaw_error_rad = abs(float(yaw_delta))
    yaw_small_threshold_rad = float(DEFAULT_STAGE_POLICY_YAW_SMALL_THRESHOLD_RAD)
    center_small_threshold_m = float(DEFAULT_STAGE_POLICY_CENTER_SMALL_THRESHOLD_M)
    translate_align_cos_threshold = float(DEFAULT_STAGE_POLICY_TRANSLATE_ALIGN_COS_THRESHOLD)
    translate_align_yaw_threshold_rad = float(DEFAULT_STAGE_POLICY_TRANSLATE_ALIGN_YAW_THRESHOLD_RAD)
    current_alignment_cos = 0.0
    if trans_vec.shape[0] == 2 and center_error_m > 1e-8:
        trans_dir = trans_vec / center_error_m
        current_long_edge_normal = _rot2(float(start_yaw)) @ np.array([0.0, 1.0], dtype=np.float64)
        current_alignment_cos = abs(float(np.dot(trans_dir, current_long_edge_normal)))

    if center_error_m <= center_small_threshold_m and yaw_error_rad > yaw_small_threshold_rad:
        policy = "rotate_only"
        reason = "center_already_close_rotation_dominant"
    elif yaw_error_rad <= yaw_small_threshold_rad:
        policy = "translate_only"
        reason = "yaw_already_close_translation_sufficient"
    elif (
        not bool(current_yaw_ambiguous)
        and center_error_m > center_small_threshold_m
        and current_alignment_cos >= translate_align_cos_threshold
        and yaw_error_rad <= translate_align_yaw_threshold_rad
    ):
        policy = "translate_only"
        reason = "translation_direction_aligned_with_current_long_edge_normal"
    else:
        yaw_priority_score = yaw_error_rad / max(yaw_small_threshold_rad, 1e-6)
        center_priority_score = center_error_m / max(center_small_threshold_m, 1e-6)
        if yaw_priority_score >= center_priority_score:
            policy = "rotate_only"
            reason = "yaw_error_dominates_single_stage_rotation"
        else:
            policy = "translate_only"
            reason = "center_error_dominates_single_stage_translation"

    return {
        "policy": str(policy),
        "reason": str(reason),
        "center_error_m": float(center_error_m),
        "yaw_error_rad": float(yaw_error_rad),
        "current_alignment_cos": float(current_alignment_cos),
        "yaw_priority_score": float(yaw_error_rad / max(yaw_small_threshold_rad, 1e-6)),
        "center_priority_score": float(center_error_m / max(center_small_threshold_m, 1e-6)),
        "start_yaw_rad": float(start_yaw),
        "goal_yaw_rad": float(goal_yaw),
        "yaw_small_threshold_rad": float(yaw_small_threshold_rad),
        "center_small_threshold_m": float(center_small_threshold_m),
        "translate_alignment_cos_threshold": float(translate_align_cos_threshold),
        "translate_alignment_yaw_threshold_rad": float(translate_align_yaw_threshold_rad),
        "current_yaw_ambiguous": bool(current_yaw_ambiguous),
    }


def _planning_quality_rank(planning_quality: str) -> int:
    quality = str(planning_quality).strip().lower()
    if quality == "success":
        return 2
    if quality == "partial_success":
        return 1
    return 0


def _single_stage_result_rank(
    result: dict[str, Any],
    heuristic_policy: str,
    center_threshold_m: float,
    yaw_threshold_rad: float,
) -> tuple[float, ...]:
    info = result.get("forward_model_info", {}) if isinstance(result.get("forward_model_info", {}), dict) else {}
    final_center_error = float(info.get("selected_final_center_error", float("inf")))
    final_yaw_error = float(info.get("selected_final_yaw_error_rad", float("inf")))
    center_progress = float(info.get("selected_center_progress", float("-inf")))
    yaw_progress = float(info.get("selected_yaw_progress", float("-inf")))
    active_steps = float(info.get("selected_contact_active_steps", 0))
    selected_policy = str(info.get("stage_policy", ""))
    if selected_policy == "rotate_only":
        target_progress = yaw_progress
        target_error = final_yaw_error / max(float(yaw_threshold_rad), 1e-6)
        secondary_progress = center_progress
        secondary_error = final_center_error / max(float(center_threshold_m), 1e-6)
        stage_steps = max(1.0, float(result.get("rotation_steps", 0)))
    else:
        target_progress = center_progress
        target_error = final_center_error / max(float(center_threshold_m), 1e-6)
        secondary_progress = yaw_progress
        secondary_error = final_yaw_error / max(float(yaw_threshold_rad), 1e-6)
        stage_steps = max(1.0, float(result.get("translation_steps", 0)))
    active_ratio = active_steps / stage_steps
    return (
        float(_planning_quality_rank(str(result.get("planning_quality", "")))),
        float(target_progress),
        -float(target_error),
        float(active_ratio),
        float(secondary_progress),
        -float(secondary_error),
        float(active_steps),
        1.0 if selected_policy == str(heuristic_policy) else 0.0,
    )


def _single_stage_candidate_credibility(
    result: dict[str, Any],
    center_threshold_m: float,
    yaw_threshold_rad: float,
) -> dict[str, Any]:
    info = result.get("forward_model_info", {}) if isinstance(result.get("forward_model_info", {}), dict) else {}
    selected_policy = str(info.get("stage_policy", result.get("requested_stage_policy", ""))).strip().lower()
    quality_rank = int(_planning_quality_rank(str(result.get("planning_quality", ""))))
    center_progress = float(info.get("selected_center_progress", float("-inf")))
    yaw_progress = float(info.get("selected_yaw_progress", float("-inf")))
    final_center_error = float(info.get("selected_final_center_error", float("inf")))
    final_yaw_error = float(info.get("selected_final_yaw_error_rad", float("inf")))
    active_steps = int(info.get("selected_contact_active_steps", 0))
    if selected_policy == "rotate_only":
        target_progress = yaw_progress
        target_error = final_yaw_error
        target_threshold = float(yaw_threshold_rad)
        stage_steps = max(1, int(result.get("rotation_steps", 0)))
    else:
        target_progress = center_progress
        target_error = final_center_error
        target_threshold = float(center_threshold_m)
        stage_steps = max(1, int(result.get("translation_steps", 0)))
    active_ratio = float(active_steps) / float(stage_steps)
    credible = bool(
        quality_rank >= 1
        or (
            float(target_progress) > 0.08
            and (int(active_steps) >= 3 or float(active_ratio) >= 0.18)
            and float(target_error) <= 1.75 * max(float(target_threshold), 1e-6)
        )
    )
    return {
        "stage_policy": str(selected_policy),
        "quality_rank": int(quality_rank),
        "target_progress": float(target_progress),
        "target_error": float(target_error),
        "target_threshold": float(target_threshold),
        "active_steps": int(active_steps),
        "active_ratio": float(active_ratio),
        "credible": bool(credible),
    }


def _build_geometry_from_explicit_tip_pose_traj(
    pose_traj: np.ndarray,
    tip_pose_traj: np.ndarray,
    contact_traj: np.ndarray,
    object_length: float,
    object_width: float,
    tip_length: float,
    tip_width: float,
) -> dict[str, Any]:
    poses = np.asarray(pose_traj, dtype=np.float32)
    tip_poses = np.asarray(tip_pose_traj, dtype=np.float32)
    contacts = np.asarray(contact_traj, dtype=np.float32)
    if poses.ndim != 2 or poses.shape[1] != 4 or poses.shape[0] < 2:
        raise ValueError("pose_traj must be [N, 4], N>=2")
    if tip_poses.ndim != 2 or tip_poses.shape != (poses.shape[0], 4):
        raise ValueError("tip_pose_traj must be [N, 4], aligned with pose_traj")
    if contacts.ndim != 2 or contacts.shape != (poses.shape[0], 3):
        raise ValueError("contact_traj must be [N, 3], aligned with pose_traj")

    tip_rows: list[list[float]] = []
    tip_frame_rows: list[np.ndarray] = []
    object_rectangles_base: list[np.ndarray] = []
    tip_rectangles_base: list[np.ndarray] = []
    for pose_row, tip_row in zip(poses, tip_poses, strict=False):
        center_i = np.asarray(pose_row[:3], dtype=np.float32)
        yaw_i = float(pose_row[3])
        tip_center_i = np.asarray(tip_row[:3], dtype=np.float32)
        tip_heading_i = float(tip_row[3])
        tip_rows.append([float(tip_center_i[0]), float(tip_center_i[1]), float(tip_center_i[2])])
        tip_frame_rows.append(
            _tip_frame_row_from_center_heading(center_xyz=tip_center_i, heading_rad=float(tip_heading_i))
        )
        object_rectangles_base.append(
            _rectangle_corners_from_pose(
                center_xyz=center_i,
                yaw=yaw_i,
                length=float(object_length),
                width=float(object_width),
            )
        )
        tip_rectangles_base.append(
            _tip_rectangle_from_pose(
                center_xyz=tip_center_i,
                yaw=tip_heading_i,
                tip_length=float(tip_length),
                tip_width=float(tip_width),
            )
        )

    tip_traj_contact = np.asarray(tip_rows, dtype=np.float32)
    tip_frame_traj = np.asarray(tip_frame_rows, dtype=np.float32)
    start_normal = _normalize_xy_vector(
        tip_traj_contact[0, :2].astype(np.float64, copy=False) - contacts[0, :2].astype(np.float64, copy=False),
        fallback_xy=tip_traj_contact[min(1, tip_traj_contact.shape[0] - 1), :2].astype(np.float64, copy=False)
        - tip_traj_contact[0, :2].astype(np.float64, copy=False),
    )
    end_normal = _normalize_xy_vector(
        tip_traj_contact[-1, :2].astype(np.float64, copy=False) - contacts[-1, :2].astype(np.float64, copy=False),
        fallback_xy=tip_traj_contact[-1, :2].astype(np.float64, copy=False)
        - tip_traj_contact[max(0, tip_traj_contact.shape[0] - 2), :2].astype(np.float64, copy=False),
    )
    return {
        "trajectory_contact_base": contacts.astype(np.float32, copy=False),
        "trajectory_tip_contact_base": tip_traj_contact.astype(np.float32, copy=False),
        "tip_pose_trajectory_base": tip_poses.astype(np.float32, copy=False),
        "tip_frame_trajectory_base": tip_frame_traj.astype(np.float32, copy=False),
        "expected_object_rectangles_base": object_rectangles_base,
        "tip_rectangles_base": tip_rectangles_base,
        "approach_normal_start_xy": start_normal.astype(np.float64, copy=False),
        "approach_normal_end_xy": end_normal.astype(np.float64, copy=False),
    }


def _interpolate_shortest_heading_sequence(start_heading: float, end_heading: float, count: int) -> np.ndarray:
    n = max(2, int(count))
    delta = _wrap_to_pi(float(end_heading) - float(start_heading))
    alpha = np.linspace(0.0, 1.0, num=n, dtype=np.float32)
    values = np.array([_wrap_to_pi(float(start_heading) + float(delta) * float(a)) for a in alpha], dtype=np.float32)
    return values.astype(np.float32, copy=False)


def _insert_face_switch_transfer_segment(
    pose_traj: np.ndarray,
    tip_pose_traj: np.ndarray,
    contact_traj: np.ndarray,
    contact_s_traj: np.ndarray,
    rotation_steps: int,
    translation_steps: int,
    rotation_face: str,
    translation_face: str,
    object_height: float,
    transfer_clearance_m: float | None,
) -> dict[str, Any]:
    poses = np.asarray(pose_traj, dtype=np.float32)
    tip_poses = np.asarray(tip_pose_traj, dtype=np.float32)
    contacts = np.asarray(contact_traj, dtype=np.float32)
    contact_s = np.asarray(contact_s_traj, dtype=np.float32).reshape(-1)
    if (
        poses.ndim != 2
        or poses.shape[1] != 4
        or tip_poses.ndim != 2
        or tip_poses.shape != poses.shape
        or contacts.ndim != 2
        or contacts.shape != (poses.shape[0], 3)
    ):
        return {
            "pose_traj": poses,
            "tip_pose_traj": tip_poses,
            "contact_traj": contacts,
            "contact_s_traj": contact_s,
            "reposition_steps": 0,
            "transfer_clearance_m": 0.0,
            "transfer_active": False,
        }
    if str(rotation_face) == str(translation_face) or int(rotation_steps) <= 0 or int(translation_steps) <= 0:
        return {
            "pose_traj": poses,
            "tip_pose_traj": tip_poses,
            "contact_traj": contacts,
            "contact_s_traj": contact_s,
            "reposition_steps": 0,
            "transfer_clearance_m": 0.0,
            "transfer_active": False,
        }

    split_idx = int(np.clip(int(rotation_steps) - 1, 0, max(0, poses.shape[0] - 1)))
    target_idx = int(np.clip(int(rotation_steps), 0, max(0, poses.shape[0] - 1)))
    if target_idx <= split_idx:
        return {
            "pose_traj": poses,
            "tip_pose_traj": tip_poses,
            "contact_traj": contacts,
            "contact_s_traj": contact_s,
            "reposition_steps": 0,
            "transfer_clearance_m": 0.0,
            "transfer_active": False,
        }

    rot_pose = poses[split_idx].astype(np.float32, copy=True)
    rot_tip = tip_poses[split_idx].astype(np.float32, copy=True)
    trans_tip = tip_poses[target_idx].astype(np.float32, copy=True)
    rot_contact = contacts[split_idx].astype(np.float32, copy=True)
    trans_contact = contacts[target_idx].astype(np.float32, copy=True)

    if transfer_clearance_m is None:
        clearance_m = float(max(0.020, float(object_height)))
    else:
        clearance_m = float(transfer_clearance_m)
    lift_steps = int(np.clip(math.ceil(clearance_m / 0.010), 2, 8))
    xy_move_dist = float(np.linalg.norm(trans_tip[:2].astype(np.float64) - rot_tip[:2].astype(np.float64)))
    move_steps = int(np.clip(math.ceil(max(0.020, xy_move_dist) / 0.015), 3, 14))
    drop_steps = int(np.clip(math.ceil(clearance_m / 0.010), 2, 8))

    lift_target = rot_tip.copy()
    lift_target[2] = float(rot_tip[2] + clearance_m)
    hover_target = trans_tip.copy()
    hover_target[2] = float(trans_tip[2] + clearance_m)

    seg_lift_xyz = np.linspace(rot_tip[:3], lift_target[:3], num=lift_steps, dtype=np.float32)
    seg_hover_xyz = np.linspace(lift_target[:3], hover_target[:3], num=move_steps, dtype=np.float32)
    seg_drop_xyz = np.linspace(hover_target[:3], trans_tip[:3], num=drop_steps, dtype=np.float32)
    transfer_xyz = np.vstack([seg_lift_xyz, seg_hover_xyz[1:], seg_drop_xyz[1:]]).astype(np.float32, copy=False)
    if transfer_xyz.shape[0] <= 2:
        return {
            "pose_traj": poses,
            "tip_pose_traj": tip_poses,
            "contact_traj": contacts,
            "contact_s_traj": contact_s,
            "reposition_steps": 0,
            "transfer_clearance_m": 0.0,
            "transfer_active": False,
        }
    insert_xyz = transfer_xyz[1:-1].astype(np.float32, copy=False)
    reposition_steps = int(insert_xyz.shape[0])
    heading_seq = _interpolate_shortest_heading_sequence(
        float(rot_tip[3]), float(trans_tip[3]), int(transfer_xyz.shape[0])
    )
    insert_heading = heading_seq[1:-1].astype(np.float32, copy=False).reshape((-1, 1))
    insert_tip_pose = np.hstack([insert_xyz, insert_heading]).astype(np.float32, copy=False)

    insert_pose = np.tile(rot_pose.reshape((1, 4)), (reposition_steps, 1)).astype(np.float32, copy=False)
    contact_interp = np.linspace(rot_contact[:3], trans_contact[:3], num=int(transfer_xyz.shape[0]), dtype=np.float32)
    insert_contact = contact_interp[1:-1].astype(np.float32, copy=False)
    if contact_s.shape[0] == poses.shape[0]:
        start_s = float(contact_s[split_idx])
        end_s = float(contact_s[target_idx])
        insert_contact_s = np.linspace(start_s, end_s, num=int(transfer_xyz.shape[0]), dtype=np.float32)[1:-1]
    else:
        insert_contact_s = np.zeros((reposition_steps,), dtype=np.float32)

    out_pose = np.vstack([poses[: split_idx + 1], insert_pose, poses[target_idx:]]).astype(np.float32, copy=False)
    out_tip_pose = np.vstack([tip_poses[: split_idx + 1], insert_tip_pose, tip_poses[target_idx:]]).astype(
        np.float32, copy=False
    )
    out_contact = np.vstack([contacts[: split_idx + 1], insert_contact, contacts[target_idx:]]).astype(
        np.float32, copy=False
    )
    if contact_s.shape[0] == poses.shape[0]:
        out_contact_s = np.concatenate([contact_s[: split_idx + 1], insert_contact_s, contact_s[target_idx:]]).astype(
            np.float32, copy=False
        )
    else:
        out_contact_s = contact_s.astype(np.float32, copy=False)
    return {
        "pose_traj": out_pose,
        "tip_pose_traj": out_tip_pose,
        "contact_traj": out_contact,
        "contact_s_traj": out_contact_s,
        "reposition_steps": int(reposition_steps),
        "transfer_clearance_m": float(clearance_m),
        "transfer_active": True,
    }


def estimate_target_yaw_base(
    dense_map_cam: np.ndarray,
    target_mask: np.ndarray,
    rotation_cam_to_base: np.ndarray,
    shift_cam_to_base: np.ndarray,
) -> dict[str, Any]:
    """
    Estimate target object's base-frame yaw from dense stereo map and segmentation mask.
    The definition is identical to push trajectory planner:
    - fit a base-frame rectangle from masked points
    - yaw is rectangle long-edge orientation in [0, pi)
    """
    dense = np.asarray(dense_map_cam, dtype=np.float32)
    mask = np.asarray(target_mask, dtype=bool)
    if dense.ndim != 3 or dense.shape[2] != 3:
        raise ValueError("dense_map_cam must be [H, W, 3]")
    if mask.shape != dense.shape[:2]:
        raise ValueError("target_mask shape must match dense_map_cam[:2]")

    valid = _valid_cam_xyz_mask_map(dense)
    target_valid = np.logical_and(mask, valid)
    if not np.any(target_valid):
        raise RuntimeError("No valid target point cloud from segmentation mask.")

    target_points_cam = dense[target_valid].astype(np.float32, copy=False)
    target_points_base = _transform_points_cam_to_base(target_points_cam, rotation_cam_to_base, shift_cam_to_base)
    if target_points_base.shape[0] == 0:
        raise RuntimeError("Failed to transform target object points to base frame.")

    fit_mask, fit_mask_erode_px = _erode_mask_for_fit(mask)
    fit_valid = np.logical_and(fit_mask, valid)
    if int(np.count_nonzero(fit_valid)) < 24:
        fit_valid = target_valid.copy()
        fit_mask_erode_px = 0

    fit_points_cam = dense[fit_valid].astype(np.float32, copy=False)
    fit_points_base_pre = _transform_points_cam_to_base(fit_points_cam, rotation_cam_to_base, shift_cam_to_base)
    if fit_points_base_pre.shape[0] < 24:
        fit_points_base_pre = target_points_base.copy()
        fit_mask_erode_px = 0

    q_lo_xy, q_hi_xy = np.quantile(fit_points_base_pre[:, :2].astype(np.float64, copy=False), [0.10, 0.90], axis=0)
    robust_xy_spread = np.maximum(q_hi_xy - q_lo_xy, 1e-4)
    fit_component_voxel_size = float(np.clip(0.07 * float(np.min(robust_xy_spread)), 0.0025, 0.0060))
    fit_points_base_filtered, fit_component_debug = _keep_largest_xy_component(
        fit_points_base_pre,
        voxel_size=fit_component_voxel_size,
    )
    min_fit_keep_count = max(24, int(round(0.35 * float(fit_points_base_pre.shape[0]))))
    fit_cluster_largest_component_ratio = float(fit_component_debug.get("largest_component_ratio", 0.0))
    fit_cluster_used = bool(
        fit_points_base_filtered.shape[0] >= min_fit_keep_count
        and fit_cluster_largest_component_ratio > float(DEFAULT_FIT_CLUSTER_DOMINANCE_RATIO)
    )
    fit_target_points_base = (
        fit_points_base_filtered.astype(np.float32, copy=False)
        if fit_cluster_used
        else fit_points_base_pre.astype(np.float32, copy=False)
    )

    fit_filter_debug = {
        "raw_target_point_count": int(target_points_base.shape[0]),
        "fit_mask_point_count": int(np.count_nonzero(fit_valid)),
        "fit_mask_erode_px": int(fit_mask_erode_px),
        "fit_mask_used_eroded": bool(fit_mask_erode_px > 0),
        "fit_pre_cluster_point_count": int(fit_points_base_pre.shape[0]),
        "fit_post_cluster_point_count": int(fit_points_base_filtered.shape[0]),
        "fit_cluster_used": bool(fit_cluster_used),
        "fit_cluster_component_count": int(fit_component_debug.get("component_count", 0)),
        "fit_cluster_largest_component_point_count": int(fit_component_debug.get("largest_component_point_count", 0)),
        "fit_cluster_largest_component_ratio": float(fit_cluster_largest_component_ratio),
        "fit_cluster_dominance_threshold": float(DEFAULT_FIT_CLUSTER_DOMINANCE_RATIO),
        "fit_cluster_voxel_size_m": float(fit_component_debug.get("voxel_size_m", fit_component_voxel_size)),
    }

    pose = _fit_rectangle_pose_base(fit_target_points_base)
    center = np.asarray(pose["center_base"], dtype=np.float32).reshape(-1)
    if center.shape[0] != 3 or not np.all(np.isfinite(center)):
        raise RuntimeError("Failed to estimate target center in base frame.")

    yaw = float(pose["yaw"])
    if not np.isfinite(yaw):
        raise RuntimeError("Failed to estimate finite yaw.")

    return {
        "yaw": float(yaw),
        "center_base": center.astype(np.float32, copy=False),
        "length": float(pose["length"]),
        "width": float(pose["width"]),
        "aspect_ratio": float(pose["aspect_ratio"]),
        "yaw_ambiguous": bool(pose["yaw_ambiguous"]),
        "principal_axis_base_xy": [float(v) for v in pose.get("principal_axis_base_xy", [1.0, 0.0])],
        "secondary_axis_base_xy": [float(v) for v in pose.get("secondary_axis_base_xy", [0.0, 1.0])],
        "fit_filter_debug": fit_filter_debug,
    }


def plan_push_trajectory_base(
    dense_map_cam: np.ndarray,
    target_mask: np.ndarray,
    goal_pose_base: dict[str, Any],
    rotation_cam_to_base: np.ndarray,
    shift_cam_to_base: np.ndarray,
    num_waypoints: int = 20,
    tip_length_m: float | None = None,
    tip_width_m: float | None = None,
    face_switch_transfer_clearance_m: float | None = None,
    search_trajectory_count: int = 192,
    table_surface_info: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """
    Plan a push trajectory using the rigid-body forward model only.
    """
    dense = np.asarray(dense_map_cam, dtype=np.float32)
    mask = np.asarray(target_mask, dtype=bool)
    if dense.ndim != 3 or dense.shape[2] != 3:
        raise ValueError("dense_map_cam must be [H, W, 3]")
    if mask.shape != dense.shape[:2]:
        raise ValueError("target_mask shape must match dense_map_cam[:2]")

    valid = _valid_cam_xyz_mask_map(dense)
    target_valid = np.logical_and(mask, valid)
    if not np.any(target_valid):
        raise RuntimeError("No valid target point cloud from segmentation mask.")

    target_points_cam = dense[target_valid].astype(np.float32, copy=False)
    target_points_base = _transform_points_cam_to_base(target_points_cam, rotation_cam_to_base, shift_cam_to_base)
    if target_points_base.shape[0] == 0:
        raise RuntimeError("Failed to transform target object points to base frame.")

    fit_mask, fit_mask_erode_px = _erode_mask_for_fit(mask)
    fit_valid = np.logical_and(fit_mask, valid)
    if int(np.count_nonzero(fit_valid)) < 24:
        fit_valid = target_valid.copy()
        fit_mask_erode_px = 0
    fit_points_cam = dense[fit_valid].astype(np.float32, copy=False)
    fit_points_base_pre = _transform_points_cam_to_base(fit_points_cam, rotation_cam_to_base, shift_cam_to_base)
    if fit_points_base_pre.shape[0] < 24:
        fit_points_base_pre = target_points_base.copy()
        fit_mask_erode_px = 0

    q_lo_xy, q_hi_xy = np.quantile(fit_points_base_pre[:, :2].astype(np.float64, copy=False), [0.10, 0.90], axis=0)
    robust_xy_spread = np.maximum(q_hi_xy - q_lo_xy, 1e-4)
    fit_component_voxel_size = float(np.clip(0.07 * float(np.min(robust_xy_spread)), 0.0025, 0.0060))
    fit_points_base_filtered, fit_component_debug = _keep_largest_xy_component(
        fit_points_base_pre,
        voxel_size=fit_component_voxel_size,
    )
    min_fit_keep_count = max(24, int(round(0.35 * float(fit_points_base_pre.shape[0]))))
    fit_cluster_largest_component_ratio = float(fit_component_debug.get("largest_component_ratio", 0.0))
    fit_cluster_used = bool(
        fit_points_base_filtered.shape[0] >= min_fit_keep_count
        and fit_cluster_largest_component_ratio > float(DEFAULT_FIT_CLUSTER_DOMINANCE_RATIO)
    )
    if fit_cluster_used:
        fit_target_points_base = fit_points_base_filtered.astype(np.float32, copy=False)
    else:
        fit_target_points_base = fit_points_base_pre.astype(np.float32, copy=False)

    fit_filter_debug = {
        "raw_target_point_count": int(target_points_base.shape[0]),
        "fit_mask_point_count": int(np.count_nonzero(fit_valid)),
        "fit_mask_erode_px": int(fit_mask_erode_px),
        "fit_mask_used_eroded": bool(fit_mask_erode_px > 0),
        "fit_pre_cluster_point_count": int(fit_points_base_pre.shape[0]),
        "fit_post_cluster_point_count": int(fit_points_base_filtered.shape[0]),
        "fit_cluster_used": bool(fit_cluster_used),
        "fit_cluster_component_count": int(fit_component_debug.get("component_count", 0)),
        "fit_cluster_largest_component_point_count": int(fit_component_debug.get("largest_component_point_count", 0)),
        "fit_cluster_largest_component_ratio": float(fit_cluster_largest_component_ratio),
        "fit_cluster_dominance_threshold": float(DEFAULT_FIT_CLUSTER_DOMINANCE_RATIO),
        "fit_cluster_voxel_size_m": float(fit_component_debug.get("voxel_size_m", fit_component_voxel_size)),
    }

    current_pose = _fit_rectangle_pose_base(fit_target_points_base)
    start_center = np.asarray(current_pose["center_base"], dtype=np.float32).reshape(-1)
    if start_center.shape[0] != 3 or not np.all(np.isfinite(start_center)):
        raise RuntimeError("Failed to fit a valid current rectangle center.")

    n = max(40, int(num_waypoints) * 2)

    if not isinstance(goal_pose_base, dict):
        raise ValueError("goal_pose_base must be a dict with center and optional yaw.")
    goal_center = np.asarray(goal_pose_base.get("center"), dtype=np.float32).reshape(-1)
    if goal_center.shape[0] != 3:
        raise ValueError("goal_pose_base.center must be a valid 3D point.")
    if not np.isfinite(goal_center[0]) or not np.isfinite(goal_center[1]):
        raise ValueError("goal_pose_base.center x/y must be finite.")
    if not np.isfinite(goal_center[2]):
        goal_center[2] = float(start_center[2])

    raw_goal_yaw = goal_pose_base.get("yaw")
    goal_yaw = None if raw_goal_yaw is None else _normalize_rect_yaw(float(raw_goal_yaw))
    current_yaw = float(current_pose["yaw"])
    current_yaw_ambiguous = bool(current_pose.get("yaw_ambiguous", False))

    if current_yaw_ambiguous and goal_yaw is not None:
        start_yaw = float(goal_yaw)
        yaw_mode = "goal_locked_due_to_ambiguous_current_rect"
    else:
        start_yaw = current_yaw
        yaw_mode = "rotate_and_translate"

    if goal_yaw is None:
        goal_yaw = start_yaw
        yaw_mode = "translate_only_legacy_goal_point"

    yaw_delta = _shortest_rect_yaw_delta(start_yaw, goal_yaw)

    table_z = float(np.quantile(target_points_base[:, 2], 0.05))
    top_z = float(np.quantile(target_points_base[:, 2], 0.95))
    object_height = max(0.01, top_z - table_z)
    push_tip_z = float(table_z + 0.50 * object_height)
    if tip_length_m is None:
        tip_length = max(0.028, min(0.070, 0.65 * float(current_pose["length"])))
    else:
        tip_length = float(tip_length_m)
    if tip_width_m is None:
        tip_width = max(0.010, min(0.028, 0.38 * float(current_pose["width"])))
    else:
        tip_width = float(tip_width_m)
    if not np.isfinite(tip_length) or tip_length <= 1e-4:
        raise ValueError("tip_length_m must be a finite positive value.")
    if not np.isfinite(tip_width) or tip_width <= 1e-4:
        raise ValueError("tip_width_m must be a finite positive value.")
    if tip_width > tip_length:
        tip_length, tip_width = tip_width, tip_length
    if face_switch_transfer_clearance_m is not None:
        if not np.isfinite(face_switch_transfer_clearance_m) or float(face_switch_transfer_clearance_m) <= 1e-4:
            raise ValueError("face_switch_transfer_clearance_m must be a finite positive value.")
    if not np.isfinite(search_trajectory_count) or int(search_trajectory_count) <= 0:
        raise ValueError("search_trajectory_count must be a finite positive integer.")
    approach_offset = max(0.020, 1.2 * tip_width)
    planner_mode_normalized = "rigid_body_sim"
    trans_vec_xy = goal_center[:2].astype(np.float64) - start_center[:2].astype(np.float64)
    stage_policy_info = _decide_stage_policy(
        trans_vec_xy=trans_vec_xy,
        start_yaw=float(start_yaw),
        goal_yaw=float(goal_yaw),
        yaw_delta=float(yaw_delta),
        current_yaw_ambiguous=bool(current_yaw_ambiguous),
    )
    heuristic_stage_policy = str(stage_policy_info["policy"])
    candidate_stage_results: list[dict[str, Any]] = []
    for candidate_stage_policy in ("rotate_only", "translate_only"):
        candidate_res = select_rigid_body_two_stage_push_plan(
            start_center=start_center,
            start_yaw=float(start_yaw),
            goal_center=goal_center,
            goal_yaw=float(goal_yaw),
            object_length=float(current_pose["length"]),
            object_width=float(current_pose["width"]),
            tip_length=float(tip_length),
            tip_width=float(tip_width),
            num_contact_points=int(n),
            stage_policy=str(candidate_stage_policy),
            search_trajectory_count=int(search_trajectory_count),
        )
        candidate_res["requested_stage_policy"] = str(candidate_stage_policy)
        candidate_stage_results.append(candidate_res)
    candidate_res_by_policy = {
        str(candidate.get("requested_stage_policy", "")): candidate for candidate in candidate_stage_results
    }
    heuristic_candidate = candidate_res_by_policy.get(str(heuristic_stage_policy))
    alternate_stage_policy = "translate_only" if str(heuristic_stage_policy) == "rotate_only" else "rotate_only"
    alternate_candidate = candidate_res_by_policy.get(str(alternate_stage_policy))
    candidate_summaries = {
        policy: _single_stage_candidate_credibility(
            candidate,
            center_threshold_m=float(stage_policy_info["center_small_threshold_m"]),
            yaw_threshold_rad=float(stage_policy_info["yaw_small_threshold_rad"]),
        )
        for policy, candidate in candidate_res_by_policy.items()
    }
    if heuristic_candidate is None:
        rigid_res = max(
            candidate_stage_results,
            key=lambda res: _single_stage_result_rank(
                res,
                heuristic_policy=str(heuristic_stage_policy),
                center_threshold_m=float(stage_policy_info["center_small_threshold_m"]),
                yaw_threshold_rad=float(stage_policy_info["yaw_small_threshold_rad"]),
            ),
        )
        stage_policy_selection_reason = "heuristic_candidate_missing_fallback_ranked_selection"
    else:
        selected_candidate = heuristic_candidate
        stage_policy_selection_reason = str(stage_policy_info["reason"])
        heuristic_summary = candidate_summaries.get(str(heuristic_stage_policy), {})
        alternate_summary = candidate_summaries.get(str(alternate_stage_policy), {})
        if alternate_candidate is not None:
            heuristic_rank = _single_stage_result_rank(
                heuristic_candidate,
                heuristic_policy=str(heuristic_stage_policy),
                center_threshold_m=float(stage_policy_info["center_small_threshold_m"]),
                yaw_threshold_rad=float(stage_policy_info["yaw_small_threshold_rad"]),
            )
            alternate_rank = _single_stage_result_rank(
                alternate_candidate,
                heuristic_policy=str(heuristic_stage_policy),
                center_threshold_m=float(stage_policy_info["center_small_threshold_m"]),
                yaw_threshold_rad=float(stage_policy_info["yaw_small_threshold_rad"]),
            )
            heuristic_credible = bool(heuristic_summary.get("credible", False))
            alternate_credible = bool(alternate_summary.get("credible", False))
            if alternate_credible and not heuristic_credible:
                selected_candidate = alternate_candidate
                stage_policy_selection_reason = (
                    f"forward_model_override_from_{heuristic_stage_policy}_to_{alternate_stage_policy}"
                )
            elif alternate_credible and heuristic_credible:
                if (
                    alternate_rank > heuristic_rank
                    and float(alternate_summary.get("target_progress", 0.0))
                    > float(heuristic_summary.get("target_progress", 0.0)) + 0.05
                ):
                    selected_candidate = alternate_candidate
                    stage_policy_selection_reason = (
                        f"forward_model_override_from_{heuristic_stage_policy}_to_{alternate_stage_policy}"
                    )
                else:
                    stage_policy_selection_reason = str(stage_policy_info["reason"])
            elif not alternate_credible:
                stage_policy_selection_reason = f"{stage_policy_info['reason']}_alternate_not_credible"
            else:
                stage_policy_selection_reason = (
                    f"{stage_policy_info['reason']}_heuristic_not_credible_no_better_alternate"
                )
        rigid_res = selected_candidate
    stage_policy = str(rigid_res.get("requested_stage_policy", heuristic_stage_policy))
    forward_model_info: dict[str, Any] = {"used": False, "model_name": "", "candidate_count": 0}
    contact_s_traj = np.zeros((0,), dtype=np.float32)
    planning_quality = "success"
    partial_success = False
    pose_traj = np.asarray(rigid_res["pose_traj"], dtype=np.float32)
    tip_pose_traj = np.asarray(rigid_res["tip_pose_traj"], dtype=np.float32)
    contact_traj = np.asarray(rigid_res["contact_traj"], dtype=np.float32)
    contact_s_traj = np.asarray(rigid_res.get("contact_s_traj", np.zeros((0,), dtype=np.float32)), dtype=np.float32)
    rot_steps = int(rigid_res["rotation_steps"])
    trans_steps = int(rigid_res["translation_steps"])
    yaw_mode = str(rigid_res.get("yaw_plan_mode", yaw_mode))
    rotation_face = str(rigid_res.get("rotation_face", ""))
    translation_face = str(rigid_res.get("translation_face", ""))
    rotation_alpha_signed = float(rigid_res.get("rotation_alpha_signed", 0.0))
    planning_quality = str(rigid_res.get("planning_quality", "success"))
    partial_success = bool(rigid_res.get("partial_success", False))
    forward_model_info = dict(rigid_res.get("forward_model_info", {}))
    forward_model_info["used"] = True
    forward_model_info["contact_s_trajectory"] = [float(v) for v in contact_s_traj.reshape(-1)]
    forward_model_info["heuristic_stage_policy"] = str(heuristic_stage_policy)
    forward_model_info["evaluated_stage_policies"] = [
        {
            "stage_policy": str(candidate.get("requested_stage_policy", "")),
            "planning_quality": str(candidate.get("planning_quality", "")),
            "credibility": candidate_summaries.get(str(candidate.get("requested_stage_policy", "")), {}),
            "final_center_error": float(
                (
                    candidate.get("forward_model_info", {})
                    if isinstance(candidate.get("forward_model_info", {}), dict)
                    else {}
                ).get("selected_final_center_error", float("inf"))
            ),
            "final_yaw_error_rad": float(
                (
                    candidate.get("forward_model_info", {})
                    if isinstance(candidate.get("forward_model_info", {}), dict)
                    else {}
                ).get("selected_final_yaw_error_rad", float("inf"))
            ),
        }
        for candidate in candidate_stage_results
    ]
    geom_data = _build_geometry_from_explicit_tip_pose_traj(
        pose_traj=pose_traj,
        tip_pose_traj=tip_pose_traj,
        contact_traj=contact_traj,
        object_length=float(current_pose["length"]),
        object_width=float(current_pose["width"]),
        tip_length=float(tip_length),
        tip_width=float(tip_width),
    )
    contact_traj = np.asarray(geom_data["trajectory_contact_base"], dtype=np.float32)
    tip_traj_contact = np.asarray(geom_data["trajectory_tip_contact_base"], dtype=np.float32)
    tip_pose_traj = np.asarray(geom_data["tip_pose_trajectory_base"], dtype=np.float32)
    tip_frame_traj = np.asarray(
        geom_data.get("tip_frame_trajectory_base", np.zeros((0, 12), dtype=np.float32)), dtype=np.float32
    )
    object_rectangles_base = list(geom_data["expected_object_rectangles_base"])
    tip_rectangles_base = list(geom_data["tip_rectangles_base"])
    reposition_steps = 0
    transfer_clearance_m = 0.0

    if pose_traj.shape[0] < 2 or tip_traj_contact.shape[0] < 2:
        raise RuntimeError("Failed to build a valid two-stage push trajectory.")

    table_surface = table_surface_info if isinstance(table_surface_info, dict) else {}
    plane_normal_base: np.ndarray | None = None
    plane_d_base: float | None = None
    plane_inlier_threshold_m = 0.012
    if table_surface:
        raw_normal = table_surface.get("plane_normal_base")
        if raw_normal is not None:
            normal_arr = np.asarray(raw_normal, dtype=np.float32).reshape(-1)
            if normal_arr.shape[0] == 3 and np.all(np.isfinite(normal_arr)):
                plane_normal_base = normal_arr.astype(np.float32, copy=False)
        raw_d = table_surface.get("plane_d_base")
        if raw_d is not None and np.isfinite(raw_d):
            plane_d_base = float(raw_d)
        raw_thr = table_surface.get("inlier_threshold_m")
        if raw_thr is not None and np.isfinite(raw_thr):
            plane_inlier_threshold_m = float(max(0.004, min(0.05, float(raw_thr))))

    local_table_points_base = _extract_local_table_support_points_base(
        dense_map_cam=dense,
        target_mask=mask,
        rotation_cam_to_base=rotation_cam_to_base,
        shift_cam_to_base=shift_cam_to_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        plane_inlier_threshold_m=float(plane_inlier_threshold_m),
    )

    object_xy = pose_traj[:, :2].astype(np.float32, copy=False)
    # _query_surface_z_sequence_base() returns z if xy has local, otherwise returns other
    surface_floor_seed = _query_surface_z_sequence_base(
        xy_points=object_xy[:1],  # query surface z at the start of the trajectory
        local_table_points_base=local_table_points_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        search_radius_m=DEFAULT_TABLE_LOCAL_SURFACE_RADIUS,
        min_surface_z=None,
    )
    surface_floor_z = (
        float(surface_floor_seed[0])
        if surface_floor_seed.shape[0] > 0 and np.isfinite(surface_floor_seed[0])
        else float(table_z)
    )
    object_surface_z = _query_surface_z_sequence_base(
        xy_points=object_xy,
        local_table_points_base=local_table_points_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        search_radius_m=DEFAULT_TABLE_LOCAL_SURFACE_RADIUS,
        min_surface_z=float(surface_floor_z),
    )
    object_center_offset_z = max(0.0, float(start_center[2]) - float(surface_floor_z))
    pose_traj[:, 2] = np.maximum(
        float(start_center[2]),
        object_surface_z.astype(np.float32, copy=False) + float(object_center_offset_z),
    ).astype(np.float32, copy=False)

    tip_xy_contact = tip_traj_contact[:, :2].astype(np.float32, copy=False)
    tip_surface_seed = _query_surface_z_sequence_base(
        xy_points=tip_xy_contact[:1],
        local_table_points_base=local_table_points_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        search_radius_m=DEFAULT_TABLE_LOCAL_SURFACE_RADIUS,
        min_surface_z=None,
    )
    tip_surface_floor_z = (
        float(tip_surface_seed[0])
        if tip_surface_seed.shape[0] > 0 and np.isfinite(tip_surface_seed[0])
        else float(surface_floor_z)
    )
    tip_surface_z_contact = _query_surface_z_sequence_base(
        xy_points=tip_xy_contact,
        local_table_points_base=local_table_points_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        search_radius_m=DEFAULT_TABLE_LOCAL_SURFACE_RADIUS,
        min_surface_z=float(tip_surface_floor_z),
    )
    tip_center_offset_z = max(0.0, float(push_tip_z) - float(tip_surface_floor_z))
    tip_z_contact = np.maximum(
        float(push_tip_z),
        tip_surface_z_contact.astype(np.float32, copy=False) + float(tip_center_offset_z),
    ).astype(np.float32, copy=False)
    contact_traj[:, 2] = tip_z_contact
    tip_traj_contact[:, 2] = tip_z_contact
    tip_pose_traj[:, 2] = tip_z_contact

    transfer_res = _insert_face_switch_transfer_segment(
        pose_traj=pose_traj,
        tip_pose_traj=tip_pose_traj,
        contact_traj=contact_traj,
        contact_s_traj=contact_s_traj,
        rotation_steps=int(rot_steps),
        translation_steps=int(trans_steps),
        rotation_face=str(rotation_face),
        translation_face=str(translation_face),
        object_height=float(object_height),
        transfer_clearance_m=(
            None if face_switch_transfer_clearance_m is None else float(face_switch_transfer_clearance_m)
        ),
    )
    pose_traj = np.asarray(transfer_res["pose_traj"], dtype=np.float32)
    tip_pose_traj = np.asarray(transfer_res["tip_pose_traj"], dtype=np.float32)
    contact_traj = np.asarray(transfer_res["contact_traj"], dtype=np.float32)
    reposition_steps = int(transfer_res.get("reposition_steps", 0))
    transfer_clearance_m = float(transfer_res.get("transfer_clearance_m", 0.0))
    contact_s_traj = np.asarray(transfer_res.get("contact_s_traj", contact_s_traj), dtype=np.float32).reshape(-1)
    if tip_pose_traj.ndim == 2 and tip_pose_traj.shape[1] >= 4 and tip_pose_traj.shape[0] > 0:
        tip_pose_traj = tip_pose_traj.astype(np.float32, copy=True)
        tip_pose_traj[:, 3] = _select_tip_heading_equivalent_sequence(
            tip_pose_traj[:, 3].astype(np.float64, copy=False)
        )

    object_rectangles_base = [
        _rectangle_corners_from_pose(
            center_xyz=pose_row[:3].astype(np.float32, copy=False),
            yaw=float(pose_row[3]),
            length=float(current_pose["length"]),
            width=float(current_pose["width"]),
        )
        for pose_row in pose_traj
    ]
    tip_traj_contact = tip_pose_traj[:, :3].astype(np.float32, copy=True)
    tip_frame_rows: list[np.ndarray] = []
    tip_rectangle_rows: list[np.ndarray] = []
    for tip_row in tip_pose_traj:
        center_xyz = tip_row[:3].astype(np.float32, copy=False)
        heading = float(tip_row[3])
        tip_frame_rows.append(_tip_frame_row_from_center_heading(center_xyz=center_xyz, heading_rad=heading))
        tip_rectangle_rows.append(
            _tip_rectangle_from_pose(
                center_xyz=center_xyz,
                yaw=heading,
                tip_length=float(tip_length),
                tip_width=float(tip_width),
            )
        )
    tip_frame_traj = (
        np.vstack(tip_frame_rows).astype(np.float32, copy=False)
        if tip_frame_rows
        else np.zeros((0, 12), dtype=np.float32)
    )
    tip_rectangles_base = list(tip_rectangle_rows)

    start_normal = np.asarray(geom_data["approach_normal_start_xy"], dtype=np.float64).reshape(-1)
    end_normal = np.asarray(geom_data["approach_normal_end_xy"], dtype=np.float64).reshape(-1)
    approach_point = tip_traj_contact[0].copy()
    approach_point[:2] += (start_normal * float(approach_offset)).astype(np.float32)
    retract_point = tip_traj_contact[-1].copy()
    retract_point[:2] += (end_normal * float(approach_offset)).astype(np.float32)
    approach_retract_xy = np.vstack([approach_point[:2].reshape((1, 2)), retract_point[:2].reshape((1, 2))]).astype(
        np.float32, copy=False
    )
    approach_retract_surface_z = _query_surface_z_sequence_base(
        xy_points=approach_retract_xy,
        local_table_points_base=local_table_points_base,
        plane_normal_base=plane_normal_base,
        plane_d_base=plane_d_base,
        search_radius_m=DEFAULT_TABLE_LOCAL_SURFACE_RADIUS,
        min_surface_z=float(tip_surface_floor_z),
    )
    approach_retract_z = np.maximum(
        float(push_tip_z),
        approach_retract_surface_z.astype(np.float32, copy=False) + float(tip_center_offset_z),
    ).astype(np.float32, copy=False)
    approach_point[2] = float(approach_retract_z[0])
    retract_point[2] = float(approach_retract_z[-1])
    tip_traj_full = np.vstack([approach_point.reshape((1, 3)), tip_traj_contact, retract_point.reshape((1, 3))]).astype(
        np.float32
    )
    if tip_frame_traj.shape[0] > 0:
        tip_frame_full = np.vstack(
            [
                _tip_frame_row_with_origin(tip_frame_traj[0], approach_point).reshape((1, 12)),
                tip_frame_traj[:, :12].astype(np.float32, copy=False),
                _tip_frame_row_with_origin(tip_frame_traj[-1], retract_point).reshape((1, 12)),
            ]
        ).astype(np.float32, copy=False)
    else:
        tip_frame_full = np.zeros((0, 12), dtype=np.float32)
    xquat_traj_full = _tip_frame_rows_to_xquat(tip_frame_full)
    final_pose = pose_traj[-1].astype(np.float64, copy=False)
    final_center_xy_error_vec = goal_center[:2].astype(np.float64, copy=False) - final_pose[:2]
    final_center_xy_error = float(np.linalg.norm(final_center_xy_error_vec))
    final_yaw_error = float(_shortest_rect_yaw_delta(float(final_pose[3]), float(goal_yaw)))
    stage_policy_reason = str(stage_policy_selection_reason)

    return {
        "trajectory_base": tip_traj_full,
        "xquat_trajectory_base": xquat_traj_full,
        "trajectory_contact_base": contact_traj,
        "trajectory_pose_base": pose_traj,
        "tip_pose_trajectory_base": tip_pose_traj,
        "tip_frame_trajectory_base": tip_frame_traj,
        "tip_frame_full_trajectory_base": tip_frame_full,
        "contact_s_trajectory": contact_s_traj.astype(np.float32, copy=False),
        "expected_object_center_base": pose_traj[:, :3].astype(np.float32, copy=False),
        "expected_object_rectangles_base": object_rectangles_base,
        "tip_rectangles_base": tip_rectangles_base,
        "current_pose_base": {
            "center": start_center.astype(np.float32),
            "yaw": float(current_yaw),
            "length": float(current_pose["length"]),
            "width": float(current_pose["width"]),
            "aspect_ratio": float(current_pose["aspect_ratio"]),
            "yaw_ambiguous": bool(current_pose["yaw_ambiguous"]),
            "table_z": float(surface_floor_z),
            "object_height": float(object_height),
        },
        "goal_pose_base": {
            "center": np.array([goal_center[0], goal_center[1], pose_traj[-1, 2]], dtype=np.float32),
            "yaw": float(goal_yaw),
        },
        "contact_face": {
            "rotation_face": str(rotation_face),
            "rotation_alpha": float(rotation_alpha_signed),
            "translation_face": str(translation_face),
            "edge_half": _contact_face_edge_half(
                str(translation_face),
                float(current_pose["length"]),
                float(current_pose["width"]),
            ),
            "rotation_edge_half": _contact_face_edge_half(
                str(rotation_face),
                float(current_pose["length"]),
                float(current_pose["width"]),
            ),
            "translation_edge_half": _contact_face_edge_half(
                str(translation_face),
                float(current_pose["length"]),
                float(current_pose["width"]),
            ),
            "tip_length": float(tip_length),
            "tip_width": float(tip_width),
            "approach_offset": float(approach_offset),
            "face_switch_transfer_lifted": bool(reposition_steps > 0),
            "face_switch_transfer_clearance_m": float(transfer_clearance_m),
            "face_switch_transfer_clearance_request_m": (
                None if face_switch_transfer_clearance_m is None else float(face_switch_transfer_clearance_m)
            ),
            "force_model": "normal_to_object_face",
        },
        "tip_frame_definition": {
            "x_axis": "tip_length_edge",
            "y_axis": "tip_width_edge",
            "z_axis": "normal_down_to_base_negative_z",
            "right_handed": True,
            "x_axis_preference": "pi-equivalent heading selection prefers base_negative_x_hemisphere_with_continuity",
        },
        "push_tip_z": float(tip_traj_contact[0, 2] if tip_traj_contact.shape[0] > 0 else push_tip_z),
        "yaw_plan_mode": yaw_mode,
        "yaw_delta_rad": float(yaw_delta),
        "goal_pose_error": {
            "center_x_error_m": float(final_center_xy_error_vec[0]),
            "center_y_error_m": float(final_center_xy_error_vec[1]),
            "center_xy_error_m": float(final_center_xy_error),
            "yaw_error_rad": float(final_yaw_error),
            "yaw_error_abs_rad": float(abs(final_yaw_error)),
            "goal_reached": bool(
                float(final_center_xy_error) <= float(stage_policy_info["center_small_threshold_m"])
                and abs(float(final_yaw_error)) <= float(stage_policy_info["yaw_small_threshold_rad"])
            ),
            "center_threshold_m": float(stage_policy_info["center_small_threshold_m"]),
            "yaw_threshold_rad": float(stage_policy_info["yaw_small_threshold_rad"]),
        },
        "stage_policy": str(stage_policy),
        "stage_policy_reason": str(stage_policy_reason),
        "stage_policy_info": {
            **stage_policy_info,
            "heuristic_policy": str(heuristic_stage_policy),
            "selected_policy": str(stage_policy),
        },
        "planner_mode": str(planner_mode_normalized),
        "planning_quality": str(planning_quality),
        "partial_success": bool(partial_success),
        "planner_status": (
            (
                "open_loop_translate_only_push_rigid_body_sim"
                if stage_policy == "translate_only"
                else "open_loop_rotate_only_push_rigid_body_sim"
            )
            + (
                ""
                if str(planning_quality) == "success"
                else ("_partial_success" if bool(partial_success) else "_best_effort")
            )
        ),
        "target_point_count": int(target_points_base.shape[0]),
        "target_points_base": target_points_base.astype(np.float32),
        "fit_target_points_base": fit_target_points_base.astype(np.float32),
        "fit_filter_debug": fit_filter_debug,
        "rotation_steps": int(rot_steps),
        "reposition_steps": int(reposition_steps),
        "translation_steps": int(trans_steps),
        "forward_model_info": {
            **forward_model_info,
            "stage_policy": str(stage_policy),
            "stage_policy_reason": str(stage_policy_reason),
        },
        "table_surface_info": {
            "status": str(table_surface.get("status", "missing")),
            "plane_normal_base": (
                None if plane_normal_base is None else [float(v) for v in plane_normal_base.reshape(-1)[:3]]
            ),
            "plane_d_base": None if plane_d_base is None else float(plane_d_base),
            "inlier_threshold_m": float(plane_inlier_threshold_m),
            "support_point_count_no_target": int(local_table_points_base.shape[0]),
            "surface_floor_z": float(surface_floor_z),
            "tip_surface_floor_z": float(tip_surface_floor_z),
        },
    }
