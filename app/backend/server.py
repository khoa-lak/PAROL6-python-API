"""
FastAPI Server & WebSocket Hub for PAROL6 3D OLP Studio.
Supports Multi-Model Robot Arm Library (saving/loading complete robot mechanisms:
7 link STL files + DH table + per-link origin/rotation offsets + TCP offset).
Located inside PAROL6-python-API/app/backend/
"""
import os
import re
import json
import time
import shutil
import base64
import asyncio
from pathlib import Path
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Response
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .kinematics import dh_parol6
from .kinematics.dh_parol6 import (
    DEFAULT_DH_TABLE_DEG,
    DEFAULT_LINK_OFFSETS,
    get_dh_table_deg,
    set_dh_table_deg,
    reset_dh_table_deg,
    get_link_offsets,
    set_link_offsets,
    reset_link_offsets,
)
from .kinematics.forward import forward_kinematics
from .kinematics.inverse import inverse_kinematics
from .kinematics.trajectory import plan_move_j, plan_move_l
from .kinematics.collision_planner import plan_collision_free_joint_path
from .communication.serial_manager import serial_manager
from .postprocessor.gcode_exporter import export_to_gcode, export_to_stm32_stream

app = FastAPI(title="PAROL6 3D OLP Studio", version="2.2.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def disable_static_cache(request: Request, call_next):
    response = await call_next(request)
    if request.url.path == "/" or request.url.path.endswith((".html", ".js", ".css")):
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
    return response

# ----------------- Persistent Multi-Model Robot Storage ----------------- #
APP_DIR = Path(__file__).resolve().parent.parent
USER_DATA_DIR = APP_DIR / "user_data"
MODELS_DIR = USER_DATA_DIR / "robot_models"
STATE_FILE = USER_DATA_DIR / "active_state.json"

# Legacy paths for automatic migration
LEGACY_STL_DIR = USER_DATA_DIR / "robot_stls"
LEGACY_PROFILE_FILE = USER_DATA_DIR / "robot_profile.json"

USER_DATA_DIR.mkdir(parents=True, exist_ok=True)
MODELS_DIR.mkdir(parents=True, exist_ok=True)


def _slugify(name: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9_\-]+", "_", name.strip()).strip("_").lower()
    return s or f"robot_{int(time.time())}"


def _ensure_default_models():
    """Ensure default model exists and migrate legacy profile if present."""
    default_dir = MODELS_DIR / "parol6_default"
    if not default_dir.exists():
        default_dir.mkdir(parents=True, exist_ok=True)
        (default_dir / "stls").mkdir(exist_ok=True)
        meta = {
            "id": "parol6_default",
            "name": "PAROL6 Mặc Định (Chuẩn)",
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "dh_table": DEFAULT_DH_TABLE_DEG,
            "link_offsets": DEFAULT_LINK_OFFSETS,
            "tcp_offset": {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0}
        }
        if LEGACY_PROFILE_FILE.exists():
            try:
                leg = json.loads(LEGACY_PROFILE_FILE.read_text(encoding="utf-8"))
                if "dh_table" in leg:
                    meta["dh_table"] = leg["dh_table"]
                if "link_offsets" in leg:
                    meta["link_offsets"] = leg["link_offsets"]
                if "tcp_offset" in leg:
                    meta["tcp_offset"] = leg["tcp_offset"]
            except Exception:
                pass
        if LEGACY_STL_DIR.exists():
            for i in range(7):
                old_stl = LEGACY_STL_DIR / f"link_{i}.stl"
                if old_stl.exists() and old_stl.stat().st_size > 0:
                    shutil.copy2(old_stl, default_dir / "stls" / f"link_{i}.stl")
        (default_dir / "model.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )


_ensure_default_models()


def _get_active_model_id() -> str:
    if STATE_FILE.exists():
        try:
            st = json.loads(STATE_FILE.read_text(encoding="utf-8"))
            mid = st.get("active_model_id", "parol6_default")
            if (MODELS_DIR / mid / "model.json").exists():
                return mid
        except Exception:
            pass
    return "parol6_default"


def _set_active_model_id(model_id: str):
    STATE_FILE.write_text(
        json.dumps({"active_model_id": model_id}, indent=2), encoding="utf-8"
    )


def _load_model_into_runtime(model_id: str) -> dict:
    model_dir = MODELS_DIR / model_id
    meta_file = model_dir / "model.json"
    if not meta_file.exists():
        model_id = "parol6_default"
        model_dir = MODELS_DIR / model_id
        meta_file = model_dir / "model.json"

    try:
        data = json.loads(meta_file.read_text(encoding="utf-8"))
    except Exception:
        data = {
            "id": model_id,
            "name": model_id,
            "dh_table": DEFAULT_DH_TABLE_DEG,
            "link_offsets": DEFAULT_LINK_OFFSETS,
            "tcp_offset": {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0}
        }

    if "dh_table" in data and isinstance(data["dh_table"], list) and len(data["dh_table"]) == 6:
        set_dh_table_deg(data["dh_table"])
    else:
        reset_dh_table_deg()

    if "link_offsets" in data and isinstance(data["link_offsets"], list) and len(data["link_offsets"]) == 7:
        set_link_offsets(data["link_offsets"])
    else:
        reset_link_offsets()

    saved_stls = {}
    stl_dir = model_dir / "stls"
    link_offsets = get_link_offsets()
    for i in range(7):
        stl_path = stl_dir / f"link_{i}.stl"
        if stl_path.exists() and stl_path.stat().st_size > 0:
            saved_stls[str(i)] = {
                "url": f"/api/models/{model_id}/stl/{i}?t={int(stl_path.stat().st_mtime)}",
                "filename": link_offsets[i].get("stl_file") or f"link_{i}.stl",
                "size": stl_path.stat().st_size
            }
        else:
            link_offsets[i]["stl_file"] = None
    set_link_offsets(link_offsets)

    _set_active_model_id(model_id)
    return {
        "model_id": model_id,
        "model_name": data.get("name", model_id),
        "dh_table": get_dh_table_deg(),
        "link_offsets": get_link_offsets(),
        "joint_limits": dh_parol6.JOINT_LIMITS_DEG,
        "saved_stls": saved_stls,
        "tcp_offset": data.get("tcp_offset", {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0}),
        "workcell_objects": data.get("workcell_objects", [])
    }


def _save_active_model_to_disk(extra_data: dict = None, model_id: str = None, model_name: str = None) -> str:
    if not model_id:
        model_id = _get_active_model_id()
    model_dir = MODELS_DIR / model_id
    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "stls").mkdir(exist_ok=True)
    meta_file = model_dir / "model.json"

    existing = {}
    if meta_file.exists():
        try:
            existing = json.loads(meta_file.read_text(encoding="utf-8"))
        except Exception:
            pass

    meta = {
        "id": model_id,
        "name": model_name or existing.get("name") or model_id,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "dh_table": get_dh_table_deg(),
        "link_offsets": get_link_offsets(),
        "tcp_offset": existing.get("tcp_offset", {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0}),
        "workcell_objects": existing.get("workcell_objects", [])
    }
    if extra_data:
        meta.update(extra_data)

    meta_file.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    _set_active_model_id(model_id)
    return model_id


# Initialize active model on server startup
_load_model_into_runtime(_get_active_model_id())

active_websockets: list[WebSocket] = []


def on_serial_rx(line: str):
    if server_loop and server_loop.is_running():
        asyncio.run_coroutine_threadsafe(
            broadcast_ws({"type": "serial_rx", "data": line}),
            server_loop
        )


async def broadcast_ws(message: dict):
    for ws in list(active_websockets):
        try:
            await ws.send_json(message)
        except Exception:
            if ws in active_websockets:
                active_websockets.remove(ws)


serial_manager.rx_callbacks.append(on_serial_rx)
server_loop = None


@app.on_event("startup")
async def on_startup():
    global server_loop
    server_loop = asyncio.get_event_loop()


# ----------------- USB & G-code REST Endpoints ----------------- #

@app.get("/api/ports")
def get_ports():
    return {"ports": serial_manager.list_ports()}


class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 115200


@app.post("/api/connect")
def connect_port(req: ConnectRequest):
    return serial_manager.connect(req.port, req.baudrate)


@app.post("/api/disconnect")
def disconnect_port():
    return serial_manager.disconnect()


class SendRequest(BaseModel):
    command: str


@app.post("/api/send")
def send_command(req: SendRequest):
    return {"success": serial_manager.send_line(req.command)}


@app.post("/api/emergency_stop")
def emergency_stop():
    serial_manager.emergency_stop()
    return {"success": True, "message": "Emergency Stop Triggered"}


class StreamRequest(BaseModel):
    waypoints: list[dict]
    mode: str = "gcode"


@app.post("/api/stream")
def stream_waypoints(req: StreamRequest):
    if req.mode == "stm32_ascii":
        cmds = export_to_stm32_stream(req.waypoints)
    else:
        gcode_text = export_to_gcode(req.waypoints)
        cmds = [line for line in gcode_text.splitlines() if line.strip() and not line.strip().startswith(";")]
    success = serial_manager.stream_program(cmds)
    return {"success": success, "lines_queued": len(cmds)}


class ExportRequest(BaseModel):
    waypoints: list[dict]
    speed: float = 1000.0
    program_name: str = "PAROL6_JOB"


@app.post("/api/export_gcode")
def export_gcode_endpoint(req: ExportRequest):
    gcode = export_to_gcode(req.waypoints, req.speed, req.program_name)
    return {"gcode": gcode}


class AvoidanceRequest(BaseModel):
    start_joints: list[float]
    goal_joints: list[float]
    obstacles: list[dict]
    dh_table: list[dict] | None = None
    tcp_offset: list[float] | None = None
    safety_margin_mm: float = 15.0
    link_radii: list[float] | None = None


@app.post("/api/plan_avoidance")
def plan_avoidance_endpoint(req: AvoidanceRequest):
    dh = req.dh_table if req.dh_table else get_dh_table_deg()
    res = plan_collision_free_joint_path(
        req.start_joints,
        req.goal_joints,
        dh,
        req.obstacles,
        req.tcp_offset,
        req.safety_margin_mm,
        link_radii=req.link_radii
    )
    return res


@app.get("/api/robot_info")
def get_robot_info():
    return {
        "model": _get_active_model_id(),
        "dof": 6,
        "joint_limits": dh_parol6.JOINT_LIMITS_DEG,
        "dh_table": get_dh_table_deg(),
        "link_offsets": get_link_offsets(),
        "default_home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }


# ----------------- Multi-Model Robot Library Endpoints ----------------- #

@app.get("/api/models")
def list_robot_models():
    models = []
    active_id = _get_active_model_id()
    for d in sorted(MODELS_DIR.iterdir()):
        if d.is_dir() and (d / "model.json").exists():
            try:
                m = json.loads((d / "model.json").read_text(encoding="utf-8"))
                stl_dir = d / "stls"
                stl_count = sum(1 for i in range(7) if (stl_dir / f"link_{i}.stl").exists())
                models.append({
                    "id": d.name,
                    "name": m.get("name", d.name),
                    "stl_count": stl_count,
                    "updated_at": m.get("updated_at", "")
                })
            except Exception:
                pass
    return {"active_model_id": active_id, "models": models}


@app.get("/api/models/{model_id}")
def get_and_activate_model(model_id: str):
    profile = _load_model_into_runtime(model_id)
    return {"success": True, **profile}


class SaveModelRequest(BaseModel):
    model_id: str | None = None
    model_name: str
    dh_table: list[dict]
    link_offsets: list[dict]
    tcp_offset: dict | None = None
    copy_stls_from_active: bool = True
    workcell_objects: list[dict] | None = None


@app.post("/api/models/save")
def save_robot_model(req: SaveModelRequest):
    prev_active_id = _get_active_model_id()
    target_id = req.model_id.strip() if req.model_id and req.model_id.strip() else _slugify(req.model_name)

    target_dir = MODELS_DIR / target_id
    target_stl_dir = target_dir / "stls"
    target_stl_dir.mkdir(parents=True, exist_ok=True)

    # Copy existing STL files from previous active model if saving as a new model
    if req.copy_stls_from_active and target_id != prev_active_id:
        src_stl_dir = MODELS_DIR / prev_active_id / "stls"
        if src_stl_dir.exists():
            for i in range(7):
                s = src_stl_dir / f"link_{i}.stl"
                d = target_stl_dir / f"link_{i}.stl"
                if s.exists() and not d.exists():
                    shutil.copy2(s, d)

    set_dh_table_deg(req.dh_table)
    set_link_offsets(req.link_offsets)
    extra = {}
    if req.tcp_offset is not None:
        extra["tcp_offset"] = req.tcp_offset
    if req.workcell_objects is not None:
        extra["workcell_objects"] = req.workcell_objects
    _save_active_model_to_disk(extra_data=extra or None, model_id=target_id, model_name=req.model_name)

    profile = _load_model_into_runtime(target_id)
    return {"success": True, **profile}


@app.post("/api/models/{model_id}/stl/{link_index}")
async def upload_model_stl(model_id: str, link_index: int, request: Request, filename: str = ""):
    if link_index < 0 or link_index > 6:
        return {"success": False, "error": "Invalid link index (0..6)"}
    body = await request.body()
    if not body:
        return {"success": False, "error": "Empty STL body"}

    model_dir = MODELS_DIR / model_id
    stl_dir = model_dir / "stls"
    stl_dir.mkdir(parents=True, exist_ok=True)

    stl_path = stl_dir / f"link_{link_index}.stl"
    stl_path.write_bytes(body)

    offsets = get_link_offsets()
    offsets[link_index]["stl_file"] = filename or f"link_{link_index}.stl"
    offsets[link_index]["deleted"] = False
    set_link_offsets(offsets)
    _save_active_model_to_disk(model_id=model_id)

    return {
        "success": True,
        "model_id": model_id,
        "link_index": link_index,
        "filename": offsets[link_index]["stl_file"],
        "size": len(body),
        "url": f"/api/models/{model_id}/stl/{link_index}?t={int(stl_path.stat().st_mtime)}"
    }


@app.get("/api/models/{model_id}/stl/{link_index}")
def get_model_stl(model_id: str, link_index: int):
    stl_path = MODELS_DIR / model_id / "stls" / f"link_{link_index}.stl"
    if not stl_path.exists():
        return Response(status_code=404, content="STL not found")
    return FileResponse(str(stl_path), media_type="application/sla", filename=f"link_{link_index}.stl")


@app.delete("/api/models/{model_id}/stl/{link_index}")
def delete_single_model_part(model_id: str, link_index: int):
    """Deletes a single Part (both its saved STL file on disk and hides/removes its 3D block)."""
    if link_index < 0 or link_index > 6:
        return {"success": False, "error": "Invalid link index (0..6)"}
    stl_path = MODELS_DIR / model_id / "stls" / f"link_{link_index}.stl"
    if stl_path.exists():
        try:
            stl_path.unlink()
        except Exception:
            pass
    # Also remove legacy STL if present
    leg_stl = LEGACY_STL_DIR / f"link_{link_index}.stl"
    if leg_stl.exists():
        try:
            leg_stl.unlink()
        except Exception:
            pass

    offsets = get_link_offsets()
    offsets[link_index]["stl_file"] = None
    offsets[link_index]["deleted"] = True
    offsets[link_index]["dx"] = 0.0
    offsets[link_index]["dy"] = 0.0
    offsets[link_index]["dz"] = 0.0
    offsets[link_index]["rx"] = 0.0
    offsets[link_index]["ry"] = 0.0
    offsets[link_index]["rz"] = 0.0
    offsets[link_index]["scale"] = 1.0
    set_link_offsets(offsets)
    _save_active_model_to_disk(model_id=model_id)
    return {"success": True, **_load_model_into_runtime(model_id)}


@app.post("/api/models/{model_id}/clear_all")
def clear_all_model_parts(model_id: str):
    """Deletes all pre-existing 3D blocks and imported STL files in the model, leaving a blank 3D workspace."""
    stl_dir = MODELS_DIR / model_id / "stls"
    if stl_dir.exists():
        shutil.rmtree(stl_dir, ignore_errors=True)
        stl_dir.mkdir(parents=True, exist_ok=True)
    if LEGACY_STL_DIR.exists():
        shutil.rmtree(LEGACY_STL_DIR, ignore_errors=True)

    offsets = reset_link_offsets()
    for i in range(7):
        offsets[i]["deleted"] = True
        offsets[i]["stl_file"] = None
    set_link_offsets(offsets)
    _save_active_model_to_disk(model_id=model_id)
    return {"success": True, **_load_model_into_runtime(model_id)}


@app.post("/api/models/{model_id}/restore_defaults")
def restore_default_model_blocks(model_id: str):
    """Restores the 7 sample PAROL6 3D blocks."""
    offsets = get_link_offsets()
    for i in range(7):
        offsets[i]["deleted"] = False
    set_link_offsets(offsets)
    _save_active_model_to_disk(model_id=model_id)
    return {"success": True, **_load_model_into_runtime(model_id)}


@app.delete("/api/models/{model_id}")
def delete_robot_model(model_id: str):
    other_models = [
        d.name for d in sorted(MODELS_DIR.iterdir())
        if d.is_dir() and (d / "model.json").exists() and d.name != model_id
    ]
    if not other_models:
        # If deleting the only remaining model (e.g. parol6_default), clear all its parts & STLs so 3D view is empty!
        return clear_all_model_parts(model_id)

    target_dir = MODELS_DIR / model_id
    if target_dir.exists():
        shutil.rmtree(target_dir, ignore_errors=True)

    next_model_id = other_models[0]
    profile = _load_model_into_runtime(next_model_id)
    return {"success": True, **profile}


@app.get("/api/models/{model_id}/export_bundle")
def export_model_bundle(model_id: str):
    """Exports the complete robot model (including all 7 binary STL files encoded in base64 + DH + offsets) into a single .robot.json file."""
    model_dir = MODELS_DIR / model_id
    meta_file = model_dir / "model.json"
    if not meta_file.exists():
        return Response(status_code=404, content="Model not found")

    meta = json.loads(meta_file.read_text(encoding="utf-8"))
    stls_b64 = {}
    stl_dir = model_dir / "stls"
    for i in range(7):
        p = stl_dir / f"link_{i}.stl"
        if p.exists() and p.stat().st_size > 0:
            stls_b64[str(i)] = {
                "filename": meta.get("link_offsets", DEFAULT_LINK_OFFSETS)[i].get("stl_file") or f"link_{i}.stl",
                "data_b64": base64.b64encode(p.read_bytes()).decode("ascii")
            }

    bundle = {
        "format": "PAROL6_ROBOT_MODEL_BUNDLE_V1",
        "model_id": meta.get("id", model_id),
        "model_name": meta.get("name", model_id),
        "dh_table": meta.get("dh_table", get_dh_table_deg()),
        "link_offsets": meta.get("link_offsets", get_link_offsets()),
        "tcp_offset": meta.get("tcp_offset", {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0}),
        "stls_base64": stls_b64,
        "workcell_objects": meta.get("workcell_objects", [])
    }
    return JSONResponse(content=bundle)


@app.post("/api/models/import_bundle")
async def import_model_bundle(request: Request):
    """Imports a .robot.json bundle file containing embedded STL files, DH table, and link offsets."""
    body = await request.json()
    model_name = body.get("model_name") or body.get("robot") or f"Robot Imported {time.strftime('%H:%M')}"
    model_id = _slugify(body.get("model_id") or model_name)
    # Avoid overwriting default if importing custom
    if model_id == "parol6_default" and body.get("model_name"):
        model_id = _slugify(model_name)

    model_dir = MODELS_DIR / model_id
    stl_dir = model_dir / "stls"
    stl_dir.mkdir(parents=True, exist_ok=True)

    dh_table = body.get("dh_table", DEFAULT_DH_TABLE_DEG)
    link_offsets = body.get("link_offsets", DEFAULT_LINK_OFFSETS)
    tcp_offset = body.get("tcp_offset", {"x": 0, "y": 0, "z": 65, "roll": 0, "pitch": 0, "yaw": 0})

    stls_b64 = body.get("stls_base64", {})
    for idx_str, stl_obj in stls_b64.items():
        i = int(idx_str)
        if 0 <= i <= 6 and "data_b64" in stl_obj:
            raw_bytes = base64.b64decode(stl_obj["data_b64"])
            (stl_dir / f"link_{i}.stl").write_bytes(raw_bytes)
            if i < len(link_offsets):
                link_offsets[i]["stl_file"] = stl_obj.get("filename", f"link_{i}.stl")

    set_dh_table_deg(dh_table)
    set_link_offsets(link_offsets)
    _save_active_model_to_disk(
        extra_data={
            "tcp_offset": tcp_offset,
            "workcell_objects": body.get("workcell_objects", [])
        },
        model_id=model_id,
        model_name=model_name
    )

    profile = _load_model_into_runtime(model_id)
    return {"success": True, **profile}


# ----------------- Legacy / Active Profile Endpoints ----------------- #

@app.get("/api/robot_profile")
def get_robot_profile():
    return _load_model_into_runtime(_get_active_model_id())


class DHUpdateRequest(BaseModel):
    dh_table: list[dict]
    link_offsets: list[dict] | None = None
    tcp_offset: dict | None = None
    save_to_disk: bool = True
    model_id: str | None = None
    model_name: str | None = None
    workcell_objects: list[dict] | None = None


@app.post("/api/dh_config")
def update_dh_config(req: DHUpdateRequest):
    updated_dh = set_dh_table_deg(req.dh_table)
    if req.link_offsets is not None:
        set_link_offsets(req.link_offsets)
    if req.save_to_disk:
        extra = {}
        if req.tcp_offset is not None:
            extra["tcp_offset"] = req.tcp_offset
        if req.workcell_objects is not None:
            extra["workcell_objects"] = req.workcell_objects
        _save_active_model_to_disk(extra_data=extra or None, model_id=req.model_id, model_name=req.model_name)
    return {
        "success": True,
        "model_id": _get_active_model_id(),
        "dh_table": updated_dh,
        "link_offsets": get_link_offsets(),
        "joint_limits": dh_parol6.JOINT_LIMITS_DEG
    }


@app.post("/api/dh_reset")
def reset_dh_config():
    updated_dh = reset_dh_table_deg()
    _save_active_model_to_disk()
    return {
        "success": True,
        "dh_table": updated_dh,
        "link_offsets": get_link_offsets(),
        "joint_limits": dh_parol6.JOINT_LIMITS_DEG
    }


@app.post("/api/robot_stl/{link_index}")
async def upload_robot_stl(link_index: int, request: Request, filename: str = "", model_id: str = ""):
    target_model = model_id.strip() if model_id.strip() else _get_active_model_id()
    return await upload_model_stl(target_model, link_index, request, filename)


@app.get("/api/robot_stl/{link_index}")
def get_saved_robot_stl(link_index: int):
    return get_model_stl(_get_active_model_id(), link_index)


@app.delete("/api/robot_stl/all")
def delete_all_saved_stls():
    active_id = _get_active_model_id()
    stl_dir = MODELS_DIR / active_id / "stls"
    if stl_dir.exists():
        for i in range(7):
            p = stl_dir / f"link_{i}.stl"
            if p.exists():
                try:
                    p.unlink()
                except Exception:
                    pass
    offsets = reset_link_offsets()
    _save_active_model_to_disk(model_id=active_id)
    return {"success": True, "link_offsets": offsets}


# ----------------- WebSocket Real-Time Kinematics ----------------- #

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_websockets.append(websocket)
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")

            if msg_type == "fk":
                joints = data.get("joints", [0, 0, 0, 0, 0, 0])
                tcp_offset = data.get("tcp_offset", [0, 0, 0, 0, 0, 0])
                res = forward_kinematics(joints, tcp_offset)
                await websocket.send_json({"type": "fk_result", "data": res})

            elif msg_type == "ik":
                pose = data.get("pose", {"x": 0, "y": 250, "z": 200, "roll": 0, "pitch": 90, "yaw": 0})
                cur_q = data.get("current_joints", [0, 0, 0, 0, 0, 0])
                tcp_offset = data.get("tcp_offset", [0, 0, 0, 0, 0, 0])
                res = inverse_kinematics(
                    pose["x"], pose["y"], pose["z"],
                    pose["roll"], pose["pitch"], pose["yaw"],
                    cur_q, tcp_offset
                )
                await websocket.send_json({"type": "ik_result", "data": res})

            elif msg_type == "plan_trajectory":
                wps = data.get("waypoints", [])
                tcp_offset = data.get("tcp_offset", [0, 0, 0, 0, 0, 0])
                sim_points = []
                for i in range(len(wps) - 1):
                    wp_a = wps[i]
                    wp_b = wps[i + 1]
                    mtype = wp_b.get("type", "MoveJ").upper()
                    if mtype == "MOVEL":
                        pts = plan_move_l(
                            wp_a.get("cartesian", {"x": 0, "y": 200, "z": 200}),
                            wp_a.get("orientation", {"roll": 0, "pitch": 0, "yaw": 0}),
                            wp_b.get("cartesian", {"x": 0, "y": 200, "z": 200}),
                            wp_b.get("orientation", {"roll": 0, "pitch": 0, "yaw": 0}),
                            wp_a.get("joints", [0, 0, 0, 0, 0, 0]),
                            steps=30,
                            tcp_offset=tcp_offset
                        )
                    else:
                        pts = plan_move_j(
                            wp_a.get("joints", [0, 0, 0, 0, 0, 0]),
                            wp_b.get("joints", [0, 0, 0, 0, 0, 0]),
                            steps=30,
                            tcp_offset=tcp_offset
                        )
                    sim_points.extend(pts)

                await websocket.send_json({"type": "trajectory_result", "points": sim_points})

    except WebSocketDisconnect:
        if websocket in active_websockets:
            active_websockets.remove(websocket)

# ----------------- Mount Static Frontend ----------------- #
frontend_path = Path(__file__).resolve().parent.parent / "frontend"
if frontend_path.exists():
    app.mount("/", StaticFiles(directory=str(frontend_path), html=True), name="frontend")
