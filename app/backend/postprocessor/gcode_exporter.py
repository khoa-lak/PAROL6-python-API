"""
Post-Processor and G-code Exporter for PAROL6 Robot System.
"""
import time

def export_to_gcode(waypoints: list[dict], speed: float = 1000.0, program_name: str = "PAROL6_JOB") -> str:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        f"; ==========================================",
        f"; Program: {program_name}.gcode",
        f"; Generated for PAROL6 6-DOF Robot",
        f"; Date: {timestamp}",
        f"; Target: STM32F407VET6 (USB / SD Card)",
        f"; ==========================================",
        "G21          ; Set units to millimeters",
        "G90          ; Absolute positioning mode",
        f"F{speed:.0f}       ; Default feedrate",
        ""
    ]

    for idx, wp in enumerate(waypoints):
        cmd_type = wp.get("type", "MoveJ").upper()
        comment = wp.get("comment", "")
        f_rate = wp.get("speed", speed)

        if cmd_type == "MOVEJ":
            q = wp.get("joints", [0, 0, 0, 0, 0, 0])
            lines.append(f"; Step {idx+1}: MoveJ {comment}")
            lines.append(f"G0 J1={q[0]:.2f} J2={q[1]:.2f} J3={q[2]:.2f} J4={q[3]:.2f} J5={q[4]:.2f} J6={q[5]:.2f} F{f_rate:.0f}")

        elif cmd_type == "MOVEL":
            p = wp.get("cartesian", {"x": 0, "y": 0, "z": 0})
            o = wp.get("orientation", {"roll": 0, "pitch": 0, "yaw": 0})
            lines.append(f"; Step {idx+1}: MoveL {comment}")
            lines.append(f"G1 X{p.get('x',0):.2f} Y{p.get('y',0):.2f} Z{p.get('z',0):.2f} A{o.get('roll',0):.2f} B{o.get('pitch',0):.2f} C{o.get('yaw',0):.2f} F{f_rate:.0f}")

        elif cmd_type == "MOVEC":
            p = wp.get("cartesian", {"x": 0, "y": 0, "z": 0})
            via = wp.get("via", {"x": 0, "y": 0, "z": 0})
            lines.append(f"; Step {idx+1}: MoveC Arc {comment}")
            lines.append(f"G2 X{p.get('x',0):.2f} Y{p.get('y',0):.2f} Z{p.get('z',0):.2f} I{via.get('x',0):.2f} J{via.get('y',0):.2f} F{f_rate:.0f}")

        elif cmd_type in ("GRIPPER", "SETIO"):
            state = wp.get("state", "OPEN")
            pin = wp.get("pin", 1)
            lines.append(f"; Step {idx+1}: Gripper / IO {state}")
            if state == "CLOSE":
                lines.append(f"M3 S1000 ; Gripper Clamped (IO Pin {pin}=1)")
            else:
                lines.append(f"M5 ; Gripper Released (IO Pin {pin}=0)")

        elif cmd_type == "WAIT":
            delay_ms = wp.get("delay_ms", 500)
            lines.append(f"G4 P{delay_ms} ; Dwell for {delay_ms}ms")

    lines.extend([
        "",
        "; End of Program",
        "M2           ; Program Stop",
        "; =========================================="
    ])

    return "\n".join(lines)

def export_to_stm32_stream(waypoints: list[dict]) -> list[str]:
    commands = []
    for wp in waypoints:
        cmd_type = wp.get("type", "MoveJ").upper()
        speed = wp.get("speed", 1000)

        if cmd_type == "MOVEJ":
            q = wp.get("joints", [0, 0, 0, 0, 0, 0])
            commands.append(f"$MJ,{q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f},{q[4]:.2f},{q[5]:.2f},{speed:.0f}")
        elif cmd_type == "MOVEL":
            p = wp.get("cartesian", {"x": 0, "y": 0, "z": 0})
            o = wp.get("orientation", {"roll": 0, "pitch": 0, "yaw": 0})
            commands.append(f"$ML,{p['x']:.2f},{p['y']:.2f},{p['z']:.2f},{o['roll']:.2f},{o['pitch']:.2f},{o['yaw']:.2f},{speed:.0f}")
        elif cmd_type in ("GRIPPER", "SETIO"):
            pin = wp.get("pin", 1)
            val = 1 if wp.get("state") == "CLOSE" or wp.get("value") == 1 else 0
            commands.append(f"$IO,{pin},{val}")
        elif cmd_type == "WAIT":
            ms = wp.get("delay_ms", 500)
            commands.append(f"$DWELL,{ms}")
    return commands
