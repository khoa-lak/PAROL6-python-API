"""
Inverse Kinematics (IK) for PAROL6 6-DOF Robot Arm.
Dynamically reads updated DH parameters and Joint Limits from dh_parol6.
"""
import numpy as np
from . import dh_parol6
from .forward import forward_kinematics, rpy_deg_to_rotation_matrix

def check_joint_limits(joint_angles_deg: list[float]) -> tuple[bool, list[str]]:
    violations = []
    is_valid = True
    for i, (angle, limits) in enumerate(zip(joint_angles_deg, dh_parol6.JOINT_LIMITS_DEG)):
        if angle < limits[0] - 0.01 or angle > limits[1] + 0.01:
            is_valid = False
            violations.append(f"J{i+1}: {angle:.1f}° ngoài phạm vi [{limits[0]}°, {limits[1]}°]")
    return is_valid, violations

def detect_singularities(joint_angles_deg: list[float], wrist_center: np.ndarray) -> dict:
    j5 = joint_angles_deg[4]
    j3 = joint_angles_deg[2]
    r_xy = np.hypot(wrist_center[0], wrist_center[1])

    is_wrist = abs(j5) < 3.0
    is_shoulder = r_xy < 15.0
    is_elbow = abs(j3) < 2.0 or abs(abs(j3) - 180.0) < 2.0

    return {
        "wrist_singularity": bool(is_wrist),
        "shoulder_singularity": bool(is_shoulder),
        "elbow_singularity": bool(is_elbow),
        "has_singularity": bool(is_wrist or is_shoulder or is_elbow)
    }

def solve_numerical_ik(
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    current_q_deg: list[float],
    tcp_offset: list[float] = None,
    max_iters: int = 50,
    tol_pos: float = 0.5,
    tol_rot: float = 0.02
) -> tuple[bool, list[float]]:
    q = np.array([np.radians(a) for a in current_q_deg], dtype=np.float64)
    damping = 0.05

    for _ in range(max_iters):
        fk = forward_kinematics([float(np.degrees(a)) for a in q], tcp_offset)
        cur_pos = np.array([fk["position"]["x"], fk["position"]["y"], fk["position"]["z"]])
        cur_rot = rpy_deg_to_rotation_matrix(fk["orientation"]["roll"], fk["orientation"]["pitch"], fk["orientation"]["yaw"])

        e_pos = target_pos - cur_pos
        R_err = target_rot @ cur_rot.T
        e_rot = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1]
        ])

        error = np.hstack([e_pos, e_rot * 100.0])
        if np.linalg.norm(e_pos) < tol_pos and np.linalg.norm(e_rot) < tol_rot:
            return True, [float(np.degrees(a)) for a in q]

        J = np.zeros((6, 6))
        delta = 1e-4
        for i in range(6):
            q_perturbed = q.copy()
            q_perturbed[i] += delta
            fk_p = forward_kinematics([float(np.degrees(a)) for a in q_perturbed], tcp_offset)
            pos_p = np.array([fk_p["position"]["x"], fk_p["position"]["y"], fk_p["position"]["z"]])
            rot_p = rpy_deg_to_rotation_matrix(fk_p["orientation"]["roll"], fk_p["orientation"]["pitch"], fk_p["orientation"]["yaw"])
            
            d_pos = (pos_p - cur_pos) / delta
            R_p_err = rot_p @ cur_rot.T
            d_rot = 0.5 * np.array([
                R_p_err[2, 1] - R_p_err[1, 2],
                R_p_err[0, 2] - R_p_err[2, 0],
                R_p_err[1, 0] - R_p_err[0, 1]
            ]) / delta
            J[:, i] = np.hstack([d_pos, d_rot * 100.0])

        JJT = J @ J.T
        delta_q = J.T @ np.linalg.inv(JJT + (damping ** 2) * np.eye(6)) @ error
        
        max_step = np.radians(10.0)
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step:
            delta_q = (delta_q / step_norm) * max_step

        q += delta_q

    q_deg = [(float(np.degrees(a)) + 180.0) % 360.0 - 180.0 for a in q]
    return False, q_deg

def inverse_kinematics(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
    current_q_deg: list[float] = None,
    tcp_offset: list[float] = None
) -> dict:
    if current_q_deg is None:
        current_q_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if tcp_offset is None:
        tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    R_target = rpy_deg_to_rotation_matrix(roll, pitch, yaw)
    P_target = np.array([x, y, z], dtype=np.float64)

    R_tcp_offset = rpy_deg_to_rotation_matrix(tcp_offset[3], tcp_offset[4], tcp_offset[5])
    P_tcp_offset = np.array(tcp_offset[:3], dtype=np.float64)

    R_flange = R_target @ R_tcp_offset.T
    P_flange = P_target - R_flange @ P_tcp_offset

    z_approach = R_flange[:, 2]
    P_w = P_flange - dh_parol6.D6 * z_approach

    # Calculate max reach from active DH parameters
    max_arm_reach = sum(abs(row["a"]) + abs(row["d"]) for row in dh_parol6.DH_PARAMS[1:5]) + 30.0
    dist_to_shoulder = np.sqrt(P_w[0]**2 + P_w[1]**2 + (P_w[2] - dh_parol6.D1)**2)
    if dist_to_shoulder > max_arm_reach:
        return {
            "success": False,
            "error": "Out of reach: Vị trí mục tiêu vượt quá tầm với tối đa của robot",
            "joint_angles": current_q_deg,
            "singularities": {"has_singularity": False, "wrist_singularity": False, "shoulder_singularity": False, "elbow_singularity": False},
            "is_within_limits": False,
            "violations": ["Vượt tầm với (Out of reach)"]
        }

    success, q_sol = solve_numerical_ik(P_target, R_target, current_q_deg, tcp_offset)
    singularities = detect_singularities(q_sol, P_w)
    is_valid, violations = check_joint_limits(q_sol)

    return {
        "success": success and is_valid,
        "joint_angles": [round(a, 2) for a in q_sol],
        "wrist_center": {"x": round(float(P_w[0]), 2), "y": round(float(P_w[1]), 2), "z": round(float(P_w[2]), 2)},
        "singularities": singularities,
        "is_within_limits": is_valid,
        "violations": violations
    }
