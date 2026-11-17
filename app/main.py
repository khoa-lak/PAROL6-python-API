"""
Main Desktop Entrypoint for PAROL6 3D OLP Studio.
Runs from PAROL6-python-API/app/main.py
"""
import os
import sys
import time
import threading

# Configure safe UTF-8 streams for Windows console (cp1252) and windowless mode (pythonw.exe)
if sys.stdout is None:
    sys.stdout = open(os.devnull, "w", encoding="utf-8", errors="replace")
elif hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

if sys.stderr is None:
    sys.stderr = open(os.devnull, "w", encoding="utf-8", errors="replace")
elif hasattr(sys.stderr, "reconfigure"):
    try:
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import uvicorn
import webbrowser
from backend.server import app

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8000
APP_URL = f"http://{SERVER_HOST}:{SERVER_PORT}"


def safe_print(msg: str):
    try:
        print(msg, flush=True)
    except Exception:
        pass


def run_server():
    try:
        config = uvicorn.Config(
            app=app,
            host=SERVER_HOST,
            port=SERVER_PORT,
            log_level="info",
            access_log=False
        )
        server = uvicorn.Server(config)
        server.run()
    except Exception as e:
        safe_print(f"[Backend Error] {e}")


def main():
    safe_print("=" * 65)
    safe_print("[PAROL6] 3D OLP Studio (PAROL6-python-API/app)")
    safe_print(f"[Server] Backend URL: {APP_URL}")
    safe_print("=" * 65)

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    time.sleep(1.0)

    try:
        import webview
        safe_print("[GUI] Starting Native Desktop Window (PyWebView)...")
        window = webview.create_window(
            title="LAK ROBOTICS - Mo Phong & Lap Trinh Ngoai Tuyen 6-DOF",
            url=APP_URL,
            width=1440,
            height=900,
            min_size=(1024, 720),
            confirm_close=False,
            text_select=True
        )
        webview.start(debug=False)
        safe_print("[GUI] Application closed.")
        sys.exit(0)

    except Exception as e:
        safe_print(f"[Fallback] Opening in default browser ({e})...")
        webbrowser.open(APP_URL)

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            safe_print("\n[Server] Stopped.")
            sys.exit(0)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        pass
    except Exception:
        import traceback
        err_msg = traceback.format_exc()
        try:
            import ctypes
            ctypes.windll.user32.MessageBoxW(
                0,
                f"Đã xảy ra lỗi khi khởi chạy PAROL6 Studio:\n\n{err_msg}",
                "PAROL6 Studio Lỗi",
                0x10
            )
        except Exception:
            pass
        sys.exit(1)
