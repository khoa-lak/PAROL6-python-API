"""
Trajectory Planner for MoveJ, MoveL, MoveC, and Gripper/IO actions.
"""
import numpy as np
from .forward import forward_kinematics, rpy_deg_to_rotation_matrix, rotation_matrix_to_rpy_deg
from .inverse import solve_numerical_ik

def slerp_quaternion(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0) + (s1 * q1)

def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        S = 2.0 * np.sqrt(tr + 1.0)
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return np.array([qw, qx, qy, qz], dtype=np.float64)

def quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)

def quintic_polynomial_profile(t: float) -> float:
    t = np.clip(t, 0.0, 1.0)
    return 10.0 * (t ** 3) - 15.0 * (t ** 4) + 6.0 * (t ** 5)

def plan_move_j(
    q_start: list[float],
    q_end: list[float],
    steps: int = 40,
    tcp_offset: list[float] = None
) -> list[dict]:
    q_s = np.array(q_start, dtype=np.float64)
    q_e = np.array(q_end, dtype=np.float64)
    points = []
    for step in range(steps + 1):
        s = quintic_polynomial_profile(step / steps)
        q_curr = q_s + s * (q_e - q_s)
        q_list = [round(float(a), 2) for a in q_curr]
        fk = forward_kinematics(q_list, tcp_offset)
        points.append({
            "step": step,
            "t": round(step / steps, 3),
            "joint_angles": q_list,
            "cartesian": fk["position"],
            "orientation": fk["orientation"]
        })
    return points

def plan_move_l(
    p_start: dict, rpy_start: dict,
    p_end: dict, rpy_end: dict,
    seed_q: list[float],
    steps: int = 40,
    tcp_offset: list[float] = None
) -> list[dict]:
    p0 = np.array([p_start["x"], p_start["y"], p_start["z"]], dtype=np.float64)
    p1 = np.array([p_end["x"], p_end["y"], p_end["z"]], dtype=np.float64)

    R0 = rpy_deg_to_rotation_matrix(rpy_start["roll"], rpy_start["pitch"], rpy_start["yaw"])
    R1 = rpy_deg_to_rotation_matrix(rpy_end["roll"], rpy_end["pitch"], rpy_end["yaw"])

    q0 = rotation_to_quaternion(R0)
    q1 = rotation_to_quaternion(R1)

    points = []
    current_seed = seed_q.copy()

    for step in range(steps + 1):
        s = quintic_polynomial_profile(step / steps)
        p_interp = p0 + s * (p1 - p0)
        q_interp = slerp_quaternion(q0, q1, s)
        R_interp = quaternion_to_rotation(q_interp)
        rpy_interp = rotation_matrix_to_rpy_deg(R_interp)

        success, q_sol = solve_numerical_ik(p_interp, R_interp, current_seed, tcp_offset, max_iters=40)
        current_seed = q_sol

        points.append({
            "step": step,
            "t": round(step / steps, 3),
            "joint_angles": [round(a, 2) for a in q_sol],
            "cartesian": {"x": round(float(p_interp[0]), 2), "y": round(float(p_interp[1]), 2), "z": round(float(p_interp[2]), 2)},
            "orientation": {"roll": round(rpy_interp[0], 2), "pitch": round(rpy_interp[1], 2), "yaw": round(rpy_interp[2], 2)},
            "ik_valid": success
        })
    return points
