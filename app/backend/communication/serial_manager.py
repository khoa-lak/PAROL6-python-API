"""
USB Serial Communication Manager for STM32F407VET6 Robot Controller.
"""
import time
import threading
import serial
import serial.tools.list_ports

class SerialManager:
    def __init__(self):
        self.ser = None
        self.port = None
        self.baudrate = 115200
        self.is_connected = False
        self.is_streaming = False
        self.stream_thread = None
        self.rx_callbacks = []
        self.rx_thread = None
        self._stop_event = threading.Event()
        self.lock = threading.Lock()
        self.log_history = []

    def log(self, direction: str, message: str):
        entry = {"time": time.strftime("%H:%M:%S"), "dir": direction, "msg": message}
        self.log_history.append(entry)
        if len(self.log_history) > 200:
            self.log_history.pop(0)

    def list_ports(self) -> list[dict]:
        ports = serial.tools.list_ports.comports()
        result = []
        for p in ports:
            result.append({
                "port": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "is_stm32": "STM" in p.description.upper() or "VIRTUAL" in p.description.upper()
            })
        return result

    def connect(self, port: str, baudrate: int = 115200) -> dict:
        with self.lock:
            if self.is_connected:
                self.disconnect()

            try:
                self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0, write_timeout=1.0)
                self.port = port
                self.baudrate = baudrate
                self.is_connected = True
                self._stop_event.clear()

                self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.rx_thread.start()
                self.log("SYS", f"Connected to {port} @ {baudrate} bps")
                return {"success": True, "message": f"Connected to {port}"}
            except Exception as e:
                self.is_connected = False
                self.log("ERR", f"Connection failed: {str(e)}")
                return {"success": False, "error": str(e)}

    def disconnect(self) -> dict:
        with self.lock:
            self.is_streaming = False
            self._stop_event.set()
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.is_connected = False
            self.log("SYS", "Disconnected from serial port")
            return {"success": True, "message": "Disconnected"}

    def send_line(self, line: str) -> bool:
        if not self.is_connected or not self.ser:
            return False
        try:
            line_clean = line.strip() + "\n"
            self.ser.write(line_clean.encode("utf-8"))
            self.log("TX", line.strip())
            return True
        except Exception as e:
            self.log("ERR", f"Send error: {str(e)}")
            return False

    def emergency_stop(self):
        if self.is_connected and self.ser:
            try:
                self.is_streaming = False
                self.ser.write(b"\x18\nM112\n")
                self.log("TX", "[EMERGENCY STOP TRIGGERED: M112]")
            except Exception:
                pass

    def _rx_loop(self):
        while not self._stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        self.log("RX", line)
                        for cb in self.rx_callbacks:
                            try:
                                cb(line)
                            except Exception:
                                pass
                else:
                    time.sleep(0.01)
            except Exception:
                break
        self.is_connected = False

    def stream_program(self, command_lines: list[str], progress_callback=None) -> bool:
        if not self.is_connected:
            return False

        def _streamer():
            self.is_streaming = True
            total = len(command_lines)
            for idx, cmd in enumerate(command_lines):
                if not self.is_streaming or not self.is_connected:
                    break

                cmd_str = cmd.strip()
                if not cmd_str or cmd_str.startswith(";"):
                    continue

                ack_received = threading.Event()
                def on_ack(response_line):
                    if "ok" in response_line.lower() or "done" in response_line.lower():
                        ack_received.set()

                self.rx_callbacks.append(on_ack)
                self.send_line(cmd_str)

                got_ack = ack_received.wait(timeout=5.0)
                if on_ack in self.rx_callbacks:
                    self.rx_callbacks.remove(on_ack)

                if progress_callback:
                    progress_callback(idx + 1, total, cmd_str)

                if not got_ack:
                    self.log("WRN", f"Timeout waiting for ACK on command: {cmd_str}")

            self.is_streaming = False
            self.log("SYS", "Program streaming completed.")

        self.stream_thread = threading.Thread(target=_streamer, daemon=True)
        self.stream_thread.start()
        return True

serial_manager = SerialManager()
