"""
3D Collision-Free Path Planner (RRT-Connect + Shortcut Smoothing) for 6-DOF Robot Arm.
Checks robot link segments against imported Workcell obstacle bounding boxes.
"""
import math
import random
import time
import numpy as np
from . import dh_parol6


def _rot_matrix(axis: str, angle_deg: float) -> np.ndarray:
    rad = math.radians(angle_deg)
    c, s = math.cos(rad), math.sin(rad)
    ax = (axis or "z").lower()
    if ax == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)
    elif ax == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    else:
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def compute_skeleton_points(
    joints_deg: list[float],
    dh_table: list[dict],
    tcp_offset: list[float] | None = None
) -> list[np.ndarray]:
    """
    Computes 3D world positions [P0, P1, P2, P3, P4, P5, P6, P_tcp] in Robotics Z-up (mm),
    matching robot.js applyDHGeometry() and _applyJointNodeRotation().
    """
    if tcp_offset is None:
        tcp_offset = [0.0, 0.0, 65.0, 0.0, 0.0, 0.0]

    def g(idx, key, default=0.0):
        if idx < len(dh_table):
            val = dh_table[idx].get(key, default)
            return float(val) if val is not None else default
        return default

    d1, a1 = g(0, "d", 118.0), g(0, "a", 0.0)
    a2, d2 = g(1, "a", 145.0), g(1, "d", 0.0)
    a3, d3 = g(2, "a", 0.0), g(2, "d", 0.0)
    d4, a4 = g(3, "d", 175.0), g(3, "a", 0.0)
    d5, a5 = g(4, "d", 0.0), g(4, "a", 0.0)
    d6, a6 = g(5, "d", 70.0), g(5, "a", 0.0)

    offsets = [
        np.array([g(0, "jx"), g(0, "jy"), d1 + g(0, "jz")], dtype=np.float64),
        np.array([a1 + g(1, "jx"), d2 + g(1, "jy"), g(1, "jz")], dtype=np.float64),
        np.array([a3 + g(2, "jx"), d3 + g(2, "jy"), a2 + g(2, "jz")], dtype=np.float64),
        np.array([a4 + g(3, "jx"), g(3, "jy"), g(3, "jz")], dtype=np.float64),
        np.array([a5 + g(4, "jx"), d5 + g(4, "jy"), d4 + g(4, "jz")], dtype=np.float64),
        np.array([a6 + g(5, "jx"), g(5, "jy"), d6 + g(5, "jz")], dtype=np.float64),
    ]
    default_axes = ["z", "y", "y", "z", "y", "z"]

    pos = np.zeros(3, dtype=np.float64)
    rot = np.eye(3, dtype=np.float64)
    pts = [pos.copy()]

    for i in range(6):
        pos = pos + rot @ offsets[i]
        axis = g_str(dh_table, i, "rot_axis", default_axes[i])
        rot = rot @ _rot_matrix(axis, float(joints_deg[i]))
        pts.append(pos.copy())

    tcp_vec = np.array(tcp_offset[:3], dtype=np.float64)
    pts.append(pos + rot @ tcp_vec)
    return pts


def g_str(dh_table: list[dict], idx: int, key: str, default: str) -> str:
    if idx < len(dh_table):
        return str(dh_table[idx].get(key) or default)
    return default


def _prepare_obstacles(obstacles: list[dict]) -> list[dict]:
    """Pre-packs obstacle outer AABBs and sub-box arrays into NumPy arrays for fast vectorized checks."""
    prepared = []
    for obs in obstacles:
        if "_bmin_arr" in obs:
            prepared.append(obs)
            continue
        b_min = obs.get("min", [0, 0, 0])
        b_max = obs.get("max", [0, 0, 0])
        sub_boxes = obs.get("sub_boxes")
        boxes = sub_boxes if (sub_boxes and len(sub_boxes) > 0) else [{"min": b_min, "max": b_max}]
        bmin_arr = np.array([b["min"] for b in boxes], dtype=np.float64)
        bmax_arr = np.array([b["max"] for b in boxes], dtype=np.float64)
        prepared.append({
            "min": b_min,
            "max": b_max,
            "_bmin_arr": bmin_arr,
            "_bmax_arr": bmax_arr,
        })
    return prepared


def is_config_collision_free(
    joints_deg: list[float],
    dh_table: list[dict],
    obstacles: list[dict],
    tcp_offset: list[float] | None = None,
    link_radius_mm: float = 35.0,
    safety_margin_mm: float = 15.0,
    link_radii: list[float] | None = None
) -> bool:
    """Checks if all robot link segments + TCP clear ALL obstacle AABBs / sub-boxes and floor."""
    pts = compute_skeleton_points(joints_deg, dh_table, tcp_offset)

    # Check floor clearance from J2 onwards
    for p in pts[2:]:
        if p[2] < 5.0:
            return False

    if not obstacles:
        return True

    # Sample points along each link segment from J1 to TCP with per-link radius
    sample_pts = []
    sample_pads = []
    for k in range(1, len(pts) - 1):
        p_a, p_b = pts[k], pts[k + 1]
        seg_len = float(np.linalg.norm(p_b - p_a))
        r_k = float(link_radii[k]) if (link_radii and k < len(link_radii)) else link_radius_mm
        pad = r_k + safety_margin_mm
        n_samples = max(3, int(math.ceil(seg_len / 20.0)) + 1)
        for s in range(n_samples):
            t = s / (n_samples - 1)
            sample_pts.append(p_a + t * (p_b - p_a))
            sample_pads.append(pad)

    pts_arr = np.array(sample_pts, dtype=np.float64)
    pads_arr = np.array(sample_pads, dtype=np.float64)
    max_pad = float(np.max(pads_arr)) + 40.0

    for obs in _prepare_obstacles(obstacles):
        b_min = obs["min"]
        b_max = obs["max"]
        ox0, oy0, oz0 = b_min[0] - max_pad, b_min[1] - max_pad, b_min[2] - max_pad
        ox1, oy1, oz1 = b_max[0] + max_pad, b_max[1] + max_pad, b_max[2] + max_pad

        mask = (
            (pts_arr[:, 0] >= ox0) & (pts_arr[:, 0] <= ox1) &
            (pts_arr[:, 1] >= oy0) & (pts_arr[:, 1] <= oy1) &
            (pts_arr[:, 2] >= oz0) & (pts_arr[:, 2] <= oz1)
        )
        if not np.any(mask):
            continue

        cand_pts = pts_arr[mask]          # (M, 3)
        cand_pads = pads_arr[mask][:, None]  # (M, 1)
        bmin_arr = obs["_bmin_arr"]       # (N, 3)
        bmax_arr = obs["_bmax_arr"]       # (N, 3)

        # Vectorized check: (M, 1, 3) vs (1, N, 3)
        hit = np.all(
            (cand_pts[:, None, :] >= (bmin_arr[None, :, :] - cand_pads[:, :, None])) &
            (cand_pts[:, None, :] <= (bmax_arr[None, :, :] + cand_pads[:, :, None])),
            axis=2
        )
        if np.any(hit):
            return False

    return True


def is_segment_collision_free(
    q_a: list[float],
    q_b: list[float],
    dh_table: list[dict],
    obstacles: list[dict],
    tcp_offset: list[float] | None = None,
    step_deg: float = 5.0,
    safety_margin_mm: float = 15.0,
    link_radii: list[float] | None = None
) -> bool:
    qa = np.array(q_a, dtype=np.float64)
    qb = np.array(q_b, dtype=np.float64)
    max_diff = float(np.max(np.abs(qb - qa)))
    steps = max(2, int(math.ceil(max_diff / step_deg)) + 1)
    for i in range(steps):
        t = i / (steps - 1)
        q = (qa + t * (qb - qa)).tolist()
        if not is_config_collision_free(q, dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
            return False
    return True


def plan_collision_free_joint_path(
    start_q: list[float],
    goal_q: list[float],
    dh_table: list[dict],
    obstacles: list[dict],
    tcp_offset: list[float] | None = None,
    safety_margin_mm: float = 15.0,
    max_iters: int = 2000,
    link_radii: list[float] | None = None
) -> dict:
    """
    Plans a collision-free joint-space path from start_q to goal_q using RRT-Connect
    followed by greedy shortcut smoothing.
    """
    if not obstacles:
        return {"success": True, "direct_clear": True, "path": [start_q, goal_q], "message": "Không có vật cản."}

    obstacles = _prepare_obstacles(obstacles)
    t_start = time.monotonic()
    time_budget_sec = 4.0

    if not is_config_collision_free(start_q, dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
        return {"success": False, "direct_clear": False, "path": [], "message": "Điểm xuất phát đang va chạm với vật cản!"}

    if not is_config_collision_free(goal_q, dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
        return {"success": False, "direct_clear": False, "path": [], "message": "Điểm đích đang va chạm với vật cản!"}

    # Fast check: is direct MoveJ already collision-free?
    if is_segment_collision_free(start_q, goal_q, dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
        return {"success": True, "direct_clear": True, "path": [start_q, goal_q], "message": "Đường đi thẳng không chạm vật cản."}

    limits = [
        [float(dh_table[i].get("min_deg", -180.0)), float(dh_table[i].get("max_deg", 180.0))]
        for i in range(6)
    ]

    qa = np.array(start_q, dtype=np.float64)
    qb = np.array(goal_q, dtype=np.float64)

    tree_a = [{"q": qa, "parent": -1}]
    tree_b = [{"q": qb, "parent": -1}]
    step_size = 22.0

    def nearest_idx(tree, q_target):
        dists = [float(np.linalg.norm(n["q"] - q_target)) for n in tree]
        return int(np.argmin(dists))

    def steer(q_from, q_to):
        diff = q_to - q_from
        d = float(np.linalg.norm(diff))
        if d <= step_size:
            return q_to
        return q_from + (diff / d) * step_size

    def extract_path(tree, idx):
        pts = []
        cur = idx
        while cur != -1:
            pts.append(tree[cur]["q"])
            cur = tree[cur]["parent"]
        pts.reverse()
        return pts

    found_path = None
    for it in range(max_iters):
        if it % 20 == 0 and (time.monotonic() - t_start) > time_budget_sec:
            break
        if it % 5 == 0:
            q_rand = qb if (it % 2 == 0) else qa
        else:
            q_rand = np.array([random.uniform(limits[j][0], limits[j][1]) for j in range(6)], dtype=np.float64)

        idx_a = nearest_idx(tree_a, q_rand)
        q_new_a = steer(tree_a[idx_a]["q"], q_rand)

        if is_segment_collision_free(tree_a[idx_a]["q"].tolist(), q_new_a.tolist(), dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
            new_idx_a = len(tree_a)
            tree_a.append({"q": q_new_a, "parent": idx_a})

            # Try connecting tree_b to q_new_a
            idx_b = nearest_idx(tree_b, q_new_a)
            q_cur_b = tree_b[idx_b]["q"]
            connected = False

            for _ in range(12):
                q_next_b = steer(q_cur_b, q_new_a)
                if not is_segment_collision_free(q_cur_b.tolist(), q_next_b.tolist(), dh_table, obstacles, tcp_offset, safety_margin_mm=safety_margin_mm, link_radii=link_radii):
                    break
                cur_parent_b = idx_b if q_cur_b is tree_b[idx_b]["q"] else (len(tree_b) - 1)
                tree_b.append({"q": q_next_b, "parent": cur_parent_b})
                q_cur_b = q_next_b
                if float(np.linalg.norm(q_cur_b - q_new_a)) < 1e-3:
                    connected = True
                    break

            if connected:
                path_a = extract_path(tree_a, new_idx_a)
                path_b = extract_path(tree_b, len(tree_b) - 1)
                path_b.reverse()
                raw_path = path_a + path_b[1:]
                # If tree_a started from goal, reverse
                if np.allclose(raw_path[0], qb) and not np.allclose(qa, qb):
                    raw_path.reverse()
                found_path = raw_path
                break

        # Swap trees for bidirectional balance
        tree_a, tree_b = tree_b, tree_a

    if found_path is None:
        return {
            "success": False,
            "direct_clear": False,
            "path": [],
            "message": "Không tìm thấy đường đi né vật cản khả thi (thử giảm lề an toàn hoặc dời vật cản)."
        }

    # Greedy shortcut smoothing to minimize number of generated MoveJ waypoints
    smoothed = [found_path[0]]
    cur_i = 0
    while cur_i < len(found_path) - 1:
        next_i = len(found_path) - 1
        while next_i > cur_i + 1:
            if is_segment_collision_free(
                found_path[cur_i].tolist(),
                found_path[next_i].tolist(),
                dh_table,
                obstacles,
                tcp_offset,
                safety_margin_mm=safety_margin_mm,
                link_radii=link_radii
            ):
                break
            next_i -= 1
        smoothed.append(found_path[next_i])
        cur_i = next_i

    rounded_path = [[round(float(v), 2) for v in q] for q in smoothed]
    return {
        "success": True,
        "direct_clear": False,
        "path": rounded_path,
        "message": f"Đã sinh {len(rounded_path)} điểm MoveJ né vật cản an toàn."
    }
