"""
Denavit-Hartenberg (DH) parameters and kinematic constraints for PAROL6 6-DOF.
Supports dynamic runtime updates and multi-model robot persistence.
Distances in mm, angles in radians (internal) and degrees (UI/API).
"""
import copy
import numpy as np

DEFAULT_DH_TABLE_DEG = [
    {"joint": 1, "name": "J1 (Shoulder Base)", "a": 0.0,   "alpha_deg": 90.0,  "d": 118.0, "offset_deg": 0.0,  "min_deg": -170.0, "max_deg": 170.0, "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "z"},
    {"joint": 2, "name": "J2 (Upper Arm)",     "a": 145.0, "alpha_deg": 0.0,   "d": 0.0,   "offset_deg": 90.0, "min_deg": -100.0, "max_deg": 90.0,  "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "y"},
    {"joint": 3, "name": "J3 (Elbow)",         "a": 0.0,   "alpha_deg": 90.0,  "d": 0.0,   "offset_deg": 0.0,  "min_deg": -150.0, "max_deg": 130.0, "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "y"},
    {"joint": 4, "name": "J4 (Forearm)",       "a": 0.0,   "alpha_deg": -90.0, "d": 175.0, "offset_deg": 0.0,  "min_deg": -180.0, "max_deg": 180.0, "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "z"},
    {"joint": 5, "name": "J5 (Wrist)",         "a": 0.0,   "alpha_deg": 90.0,  "d": 0.0,   "offset_deg": 0.0,  "min_deg": -105.0, "max_deg": 105.0, "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "y"},
    {"joint": 6, "name": "J6 (Flange)",        "a": 0.0,   "alpha_deg": 0.0,   "d": 70.0,  "offset_deg": 0.0,  "min_deg": -360.0, "max_deg": 360.0, "jx": 0.0, "jy": 0.0, "jz": 0.0, "rot_axis": "z"},
]

DEFAULT_LINK_OFFSETS = [
    {"link": 0, "name": "Part 0: Đế Robot (Base)",        "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "z", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 1, "name": "Part 1: Cụm Vai (Khâu J1)",      "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "z", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 2, "name": "Part 2: Cánh Tay Trên (Khâu J2)","dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "y", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 3, "name": "Part 3: Khuỷu Tay (Khâu J3)",    "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "y", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 4, "name": "Part 4: Cẳng Tay (Khâu J4)",     "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "z", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 5, "name": "Part 5: Cổ Tay (Khâu J5)",       "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "y", "auto_center": False, "deleted": False, "stl_file": None},
    {"link": 6, "name": "Part 6: Mặt Bích (Khâu J6)",     "dx": 0.0, "dy": 0.0, "dz": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0, "pivot_dx": 0.0, "pivot_dy": 0.0, "pivot_dz": 0.0, "pivot_rx": 0.0, "pivot_ry": 0.0, "pivot_rz": 0.0, "scale": 1.0, "coord_mode": "dh_joint", "align_mode": "cad", "stl_up_axis": "z", "rot_axis": "z", "auto_center": False, "deleted": False, "stl_file": None},
]

CURRENT_DH_TABLE_DEG = copy.deepcopy(DEFAULT_DH_TABLE_DEG)
CURRENT_LINK_OFFSETS = copy.deepcopy(DEFAULT_LINK_OFFSETS)

D1 = 118.0
A2 = 145.0
A3 = 0.0
D4 = 175.0
D6 = 70.0

DH_PARAMS = []
JOINT_LIMITS_DEG = []


def _rebuild_runtime_dh():
    global D1, A2, A3, D4, D6, DH_PARAMS, JOINT_LIMITS_DEG
    DH_PARAMS.clear()
    JOINT_LIMITS_DEG.clear()

    for row in CURRENT_DH_TABLE_DEG:
        DH_PARAMS.append({
            "a": float(row.get("a", 0.0)),
            "alpha": float(np.radians(row.get("alpha_deg", 0.0))),
            "d": float(row.get("d", 0.0)),
            "offset": float(np.radians(row.get("offset_deg", 0.0)))
        })
        JOINT_LIMITS_DEG.append([
            float(row.get("min_deg", -180.0)),
            float(row.get("max_deg", 180.0))
        ])

    if len(CURRENT_DH_TABLE_DEG) >= 6:
        D1 = float(CURRENT_DH_TABLE_DEG[0].get("d", 118.0))
        A2 = float(CURRENT_DH_TABLE_DEG[1].get("a", 145.0))
        A3 = float(CURRENT_DH_TABLE_DEG[2].get("a", 0.0))
        D4 = float(CURRENT_DH_TABLE_DEG[3].get("d", 175.0))
        D6 = float(CURRENT_DH_TABLE_DEG[5].get("d", 70.0))


def get_dh_table_deg() -> list[dict]:
    return copy.deepcopy(CURRENT_DH_TABLE_DEG)


def set_dh_table_deg(new_table: list[dict]) -> list[dict]:
    global CURRENT_DH_TABLE_DEG
    if not isinstance(new_table, list) or len(new_table) != 6:
        raise ValueError("DH table must contain 6 rows (J1..J6)")

    updated = []
    for i, row in enumerate(new_table):
        default_row = DEFAULT_DH_TABLE_DEG[i]
        updated.append({
            "joint": i + 1,
            "name": str(row.get("name", default_row["name"])),
            "a": float(row.get("a", default_row["a"])),
            "alpha_deg": float(row.get("alpha_deg", default_row["alpha_deg"])),
            "d": float(row.get("d", default_row["d"])),
            "offset_deg": float(row.get("offset_deg", default_row["offset_deg"])),
            "min_deg": float(row.get("min_deg", default_row["min_deg"])),
            "max_deg": float(row.get("max_deg", default_row["max_deg"])),
            "jx": float(row.get("jx", 0.0)),
            "jy": float(row.get("jy", 0.0)),
            "jz": float(row.get("jz", 0.0)),
            "rot_axis": str(row.get("rot_axis", default_row["rot_axis"])),
        })
    CURRENT_DH_TABLE_DEG = updated
    _rebuild_runtime_dh()
    return get_dh_table_deg()


def reset_dh_table_deg() -> list[dict]:
    global CURRENT_DH_TABLE_DEG
    CURRENT_DH_TABLE_DEG = copy.deepcopy(DEFAULT_DH_TABLE_DEG)
    _rebuild_runtime_dh()
    return get_dh_table_deg()


def get_link_offsets() -> list[dict]:
    return copy.deepcopy(CURRENT_LINK_OFFSETS)


def set_link_offsets(new_offsets: list[dict]) -> list[dict]:
    global CURRENT_LINK_OFFSETS
    if not isinstance(new_offsets, list) or len(new_offsets) != 7:
        return get_link_offsets()
    updated = []
    for i, row in enumerate(new_offsets):
        def_row = DEFAULT_LINK_OFFSETS[i]
        updated.append({
            "link": i,
            "name": str(row.get("name", def_row["name"])),
            "dx": float(row.get("dx", 0.0)),
            "dy": float(row.get("dy", 0.0)),
            "dz": float(row.get("dz", 0.0)),
            "rx": float(row.get("rx", 0.0)),
            "ry": float(row.get("ry", 0.0)),
            "rz": float(row.get("rz", 0.0)),
            "pivot_dx": float(row.get("pivot_dx", 0.0)),
            "pivot_dy": float(row.get("pivot_dy", 0.0)),
            "pivot_dz": float(row.get("pivot_dz", 0.0)),
            "pivot_rx": float(row.get("pivot_rx", 0.0)),
            "pivot_ry": float(row.get("pivot_ry", 0.0)),
            "pivot_rz": float(row.get("pivot_rz", 0.0)),
            "scale": float(row.get("scale", 1.0)),
            "coord_mode": str(row.get("coord_mode", CURRENT_LINK_OFFSETS[i].get("coord_mode", "dh_joint"))),
            "align_mode": str(row.get("align_mode", CURRENT_LINK_OFFSETS[i].get("align_mode", "cad"))),
            "stl_up_axis": str(row.get("stl_up_axis", CURRENT_LINK_OFFSETS[i].get("stl_up_axis", "z"))),
            "rot_axis": str(row.get("rot_axis", CURRENT_LINK_OFFSETS[i].get("rot_axis", def_row["rot_axis"]))),
            "auto_center": bool(row.get("auto_center", CURRENT_LINK_OFFSETS[i].get("auto_center", False))),
            "deleted": bool(row.get("deleted", CURRENT_LINK_OFFSETS[i].get("deleted", False))),
            "stl_file": row.get("stl_file", CURRENT_LINK_OFFSETS[i].get("stl_file"))
        })
    CURRENT_LINK_OFFSETS = updated
    return get_link_offsets()


def reset_link_offsets() -> list[dict]:
    global CURRENT_LINK_OFFSETS
    CURRENT_LINK_OFFSETS = copy.deepcopy(DEFAULT_LINK_OFFSETS)
    return get_link_offsets()


_rebuild_runtime_dh()
