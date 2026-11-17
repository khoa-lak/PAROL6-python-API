"""
Forward Kinematics (FK) for PAROL6 6-DOF Robot Arm.
"""
import numpy as np
from .dh_parol6 import DH_PARAMS

def dh_transform_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ], dtype=np.float64)

def rotation_matrix_to_rpy_deg(R: np.ndarray) -> tuple[float, float, float]:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))

def rpy_deg_to_rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def forward_kinematics(joint_angles_deg: list[float], tcp_offset: list[float] = None) -> dict:
    if tcp_offset is None:
        tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    joint_angles_rad = [np.radians(q) for q in joint_angles_deg]
    T_current = np.eye(4, dtype=np.float64)
    link_matrices = [T_current.copy().tolist()]

    for i in range(6):
        dh = DH_PARAMS[i]
        theta = joint_angles_rad[i] + dh["offset"]
        T_i = dh_transform_matrix(dh["a"], dh["alpha"], dh["d"], theta)
        T_current = T_current @ T_i
        link_matrices.append(T_current.copy().tolist())

    T_flange = T_current.copy()
    flange_pos = T_flange[:3, 3]
    flange_rpy = rotation_matrix_to_rpy_deg(T_flange[:3, :3])

    R_tcp_offset = rpy_deg_to_rotation_matrix(tcp_offset[3], tcp_offset[4], tcp_offset[5])
    T_tcp_offset = np.eye(4)
    T_tcp_offset[:3, :3] = R_tcp_offset
    T_tcp_offset[:3, 3] = tcp_offset[:3]

    T_tcp = T_flange @ T_tcp_offset
    tcp_pos = T_tcp[:3, 3]
    tcp_rpy = rotation_matrix_to_rpy_deg(T_tcp[:3, :3])

    return {
        "position": {"x": round(float(tcp_pos[0]), 3), "y": round(float(tcp_pos[1]), 3), "z": round(float(tcp_pos[2]), 3)},
        "orientation": {"roll": round(tcp_rpy[0], 2), "pitch": round(tcp_rpy[1], 2), "yaw": round(tcp_rpy[2], 2)},
        "flange_pose": {
            "x": round(float(flange_pos[0]), 3), "y": round(float(flange_pos[1]), 3), "z": round(float(flange_pos[2]), 3),
            "roll": round(flange_rpy[0], 2), "pitch": round(flange_rpy[1], 2), "yaw": round(flange_rpy[2], 2)
        },
        "link_matrices": link_matrices
    }
