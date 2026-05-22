"""Moore6mController.py

Single root process for all Caltech 6m telescope control activities.

Architecture
------------
- Moore6mSerial      : owned directly in this process (serial port)
- ZMQ REP socket     : queued command/response for Moore6mClient (port 5566)
- ZMQ PULL socket    : immediate e-stop bypass (port 5567)
- SRT daemon         : multiprocessing.Process — killed instantly on stop
- Dash dashboard     : multiprocessing.Process — killed instantly on stop
- Tkinter GUI        : runs on main thread

Using Process (not Thread) for daemon and dashboard means:
  - stop() can call .kill() for an immediate, guaranteed OS-level kill
  - ports are released by the OS the moment the process dies
  - no ZMQ/waitress/Dash internal threads can block our shutdown

The controller itself exits via os._exit(0) which bypasses Python's
shutdown sequence entirely (ZMQ C-level threads cannot block it).

Usage (shortcut target):
    powershell.exe -ExecutionPolicy Bypass -File "C:\path\to\run_controller.ps1"

CLI args (all optional, override config where applicable):
    --port        Serial port (default COM1)
    --baud        Baudrate   (default 115200)
    --config_dir  Path to SRT config directory (default ./config)
    --config_file Config YAML filename (default config.yaml)
    --cmd-port    ZMQ command REP port (default 5566)
    --estop-port  ZMQ e-stop PULL port (default 5567)
    --safe-mode   Start with motion/control commands blocked
    --nogui       Headless mode (no Tkinter window; run until Ctrl-C)
"""

import argparse
import json
import logging
import multiprocessing
import os
import threading
import time
from pathlib import Path
from typing import Optional

import zmq

try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
except Exception:
    tk = None

from srt.daemon.rotor_control.moore6m_serial import Moore6mSerial
from srt import config_loader


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_CMD_PORT    = 5566
DEFAULT_ESTOP_PORT  = 5567
DEFAULT_CONFIG_DIR  = "config"
DEFAULT_CONFIG_FILE = "config.yaml"

GUI_POLL_MS  = 500
LOG_MAXLINES = 500


# ---------------------------------------------------------------------------
# Process target functions (must be module-level for multiprocessing spawn)
# ---------------------------------------------------------------------------

def _run_daemon(config_dir, config_dict):
    from srt.daemon import daemon as srt_d
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(threadName)s]: %(message)s",
    )
    d = srt_d.SmallRadioTelescopeDaemon(config_dir, config_dict)
    d.srt_daemon_main()


def _run_dashboard(config_dir, config_dict):
    from waitress import serve
    from srt.dashboard import app as srt_app
    logging.getLogger("waitress.queue").disabled = True
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s %(levelname)s: %(message)s",
    )
    app_server, _ = srt_app.generate_app(config_dir, config_dict)
    host = config_dict.get("DASHBOARD_HOST", "127.0.0.1")
    port = config_dict.get("DASHBOARD_PORT", 8080)
    serve(app_server, host=host, port=port)


# ---------------------------------------------------------------------------
# Config loader
# ---------------------------------------------------------------------------

def _load_config(config_dir: str, config_file: str) -> Optional[dict]:
    config_dir_path = Path(config_dir)
    config_path     = config_dir_path / config_file
    schema_path     = config_dir_path / "schema.yaml"
    sky_coords_path = config_dir_path / "sky_coords.csv"

    missing = [
        str(p) for p in (config_dir_path, sky_coords_path, schema_path, config_path)
        if not p.exists()
    ]
    if missing:
        logging.error("Missing required config paths: %s", missing)
        return None
    if not config_loader.validate_yaml_schema(config_path, schema_path):
        logging.error("Config YAML failed schema validation: %s", config_path)
        return None
    return config_loader.load_yaml(config_path)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class Moore6mController:

    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        config_dir: str,
        config_file: str,
        cmd_port: int   = DEFAULT_CMD_PORT,
        estop_port: int = DEFAULT_ESTOP_PORT,
        safe_mode: bool = False,
    ):
        self.serial_port = serial_port
        self.baudrate    = baudrate
        self.config_dir  = config_dir
        self.config_file = config_file
        self.cmd_port    = cmd_port
        self.estop_port  = estop_port

        self._running = threading.Event()
        self._running.set()

        self.config_dict = _load_config(config_dir, config_file)
        if self.config_dict is None:
            raise RuntimeError(
                f"Failed to load config from {config_dir}/{config_file}"
            )

        self._log_lock  = threading.Lock()
        self._log_lines : list = []
        self._log_dirty = False

        logging.warning(
            "Initialising Moore6mSerial on %s @ %d baud", serial_port, baudrate
        )
        self.moore6m = Moore6mSerial(
            port=serial_port,
            baudrate=baudrate,
            safe_mode=bool(safe_mode),
        )
        logging.warning("Moore6mSerial ready")

        self._zmq_ctx      = zmq.Context()
        self._cmd_socket   = self._zmq_ctx.socket(zmq.REP)
        self._cmd_socket.bind(f"tcp://*:{self.cmd_port}")
        self._estop_socket = self._zmq_ctx.socket(zmq.PULL)
        self._estop_socket.bind(f"tcp://*:{self.estop_port}")

        self._daemon_proc    : Optional[multiprocessing.Process] = None
        self._dashboard_proc : Optional[multiprocessing.Process] = None

        self._dash_host = self.config_dict.get("DASHBOARD_HOST", "127.0.0.1")
        self._dash_port = self.config_dict.get("DASHBOARD_PORT", 8080)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, msg: str):
        ts   = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        logging.info(msg)
        with self._log_lock:
            self._log_lines.append(line)
            if len(self._log_lines) > LOG_MAXLINES:
                self._log_lines = self._log_lines[-LOG_MAXLINES:]
            self._log_dirty = True

    def get_log_lines(self) -> list:
        with self._log_lock:
            self._log_dirty = False
            return list(self._log_lines)

    def log_is_dirty(self) -> bool:
        with self._log_lock:
            return self._log_dirty

    # ------------------------------------------------------------------
    # ZMQ threads
    # ------------------------------------------------------------------

    def start(self):
        for target, name in (
            (self._cmd_loop,   "zmq-cmd-loop"),
            (self._estop_loop, "zmq-estop-loop"),
        ):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()

    def _cmd_loop(self):
        while self._running.is_set():
            try:
                msg = self._cmd_socket.recv_string(flags=0)
            except zmq.error.ZMQError:
                break
            resp = self._handle_command(msg)
            try:
                self._cmd_socket.send_string(json.dumps(resp))
            except Exception:
                logging.exception("Failed to send ZMQ response")

    def _estop_loop(self):
        poller = zmq.Poller()
        poller.register(self._estop_socket, zmq.POLLIN)
        while self._running.is_set():
            try:
                socks = dict(poller.poll(200))
            except zmq.error.ZMQError:
                break
            if self._estop_socket in socks:
                try:
                    self._estop_socket.recv_string(flags=0)
                except Exception:
                    continue
                self._log("E-stop received via ZMQ")
                self._send_spa_immediate()

    # ------------------------------------------------------------------
    # Daemon process
    # ------------------------------------------------------------------

    def start_daemon(self) -> bool:
        if self.daemon_is_running():
            self._log("Daemon already running")
            return True
        try:
            self._daemon_proc = multiprocessing.Process(
                target=_run_daemon,
                args=(self.config_dir, self.config_dict),
                name="srt-daemon",
                daemon=True,
            )
            self._daemon_proc.start()
            self._log(f"Daemon started (pid {self._daemon_proc.pid})")
            return True
        except Exception:
            logging.exception("Failed to start daemon")
            self._log("ERROR: daemon failed to start")
            return False

    def stop_daemon(self):
        p = self._daemon_proc
        if p is None:
            return
        self._daemon_proc = None
        p.quit() 
        time.sleep(0.5)  # Give the daemon a moment to exit gracefully
        if not p.is_alive():
            return
        self._log(f"Stopping daemon (pid {p.pid})")
        p.terminate()
        p.join(timeout=2.0)
        if p.is_alive():
            self._log("Daemon did not terminate cleanly, killing")
            p.kill()
            p.join(timeout=1.0)
        self._log("Daemon stopped")

    def daemon_is_running(self) -> bool:
        return bool(self._daemon_proc and self._daemon_proc.is_alive())

    # ------------------------------------------------------------------
    # Dashboard process
    # ------------------------------------------------------------------

    def start_dashboard(self) -> bool:
        if self.dashboard_is_running():
            self._log("Dashboard already running")
            return True
        try:
            self._dashboard_proc = multiprocessing.Process(
                target=_run_dashboard,
                args=(self.config_dir, self.config_dict),
                name="srt-dashboard",
                daemon=True,
            )
            self._dashboard_proc.start()
            self._log(
                f"Dashboard started (pid {self._dashboard_proc.pid}) "
                f"on http://{self._dash_host}:{self._dash_port}"
            )
            return True
        except Exception:
            logging.exception("Failed to start dashboard")
            self._log("ERROR: dashboard failed to start")
            return False

    def stop_dashboard(self):
        p = self._dashboard_proc
        if p is None:
            return
        self._dashboard_proc = None
        if not p.is_alive():
            return
        self._log(f"Stopping dashboard (pid {p.pid})")
        p.terminate()
        p.join(timeout=2.0)
        if p.is_alive():
            self._log("Dashboard did not terminate cleanly, killing")
            p.kill()
            p.join(timeout=1.0)
        self._log("Dashboard stopped")

    def dashboard_is_running(self) -> bool:
        return bool(self._dashboard_proc and self._dashboard_proc.is_alive())

    # ------------------------------------------------------------------
    # E-stop
    # ------------------------------------------------------------------

    def _send_spa_immediate(self) -> bool:
        try:
            m = self.moore6m
            if hasattr(m, "_serial_lock") and hasattr(m, "serial") and m.serial:
                with m._serial_lock:
                    m._record_serial_comm("sent", "SPA (immediate)")
                    m.serial.write(b"SPA\r")
                    time.sleep(0.01)
                if m.state not in (Moore6mSerial.State.FAULT,
                                   Moore6mSerial.State.SHUTDOWN):
                    m._transition_state(
                        Moore6mSerial.State.READY, "stop-all immediate"
                    )
                self._log("SPA sent (immediate)")
                return True
        except Exception:
            logging.exception("Immediate SPA failed")
            self._log("ERROR: immediate SPA failed")
        return False

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def stop(self):
        if not self._running.is_set():
            return
        self._running.clear()
        self._log("Controller shutting down")

        self.stop_daemon()
        self.stop_dashboard()

        try:
            if hasattr(self.moore6m, "_command_worker_stop"):
                self.moore6m._command_worker_stop.set()
            m = self.moore6m
            if hasattr(m, "serial") and m.serial and m.serial.is_open:
                try:
                    m.serial.write(b"SPA\r")
                except Exception:
                    pass
                m.serial.close()
            m._transition_state(Moore6mSerial.State.SHUTDOWN, "cleanup invoked")
        except Exception:
            logging.exception("Error during serial shutdown")

        for sock in (self._cmd_socket, self._estop_socket):
            try:
                sock.setsockopt(zmq.LINGER, 0)
                sock.close()
            except Exception:
                pass
        try:
            self._zmq_ctx.destroy(linger=0)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # ZMQ command dispatch
    # ------------------------------------------------------------------

    def _handle_command(self, raw: str) -> dict:
        parts = raw.strip().split()
        cmd   = parts[0].upper() if parts else ""
        try:
            if cmd in ("STATUS", "GET_STATUS"):
                az, el = self.moore6m.status()
                fsm    = self.moore6m.get_fsm_status()
                _fields = [
                    "mode", "CalSts", "AzBrkOn", "ElBrkOn", "EmStopOn",
                    "azerr", "elerr", "amp_currents", "safe_mode",
                    "ElUpPreLim", "ElDnPreLim", "ElUpFinLim", "ElDnFinLim",
                    "AzCwPreLim", "AzCcwPreLim", "AzCwFinLim", "AzCcwFinLim",
                    "AzLT180", "SimMode",
                ]
                diagnostics = {k: getattr(self.moore6m, k, None) for k in _fields}
                return {"ok": True, "fsm": fsm, "az": az, "el": el,
                        "diagnostics": diagnostics}

            if cmd == "POINT" and len(parts) >= 3:
                res = self.moore6m.point(float(parts[1]), float(parts[2]))
                return {"ok": True, "response": res}

            if cmd == "POINT_RADEC" and len(parts) >= 3:
                res = self.moore6m.point_radec(float(parts[1]), float(parts[2]))
                return {"ok": True, "response": res}

            if cmd == "CALIBRATE":
                threading.Thread(
                    target=self.moore6m.calibrate, daemon=True
                ).start()
                return {"ok": True}

            if cmd == "STARTUP":
                threading.Thread(
                    target=self.moore6m.startup, daemon=True
                ).start()
                return {"ok": True}

            if cmd in ("SPA", "ESTOP"):
                return {"ok": self._send_spa_immediate()}

            if cmd == "GET_COMM_HISTORY":
                return {"ok": True,
                        "history": self.moore6m.get_recent_command_history(limit=20)}

            if cmd == "GET_SERIAL_COMM":
                return {"ok": True,
                        "serial": self.moore6m.get_recent_serial_communications(limit=20)}

            if cmd == "DAEMON_START":
                return {"ok": self.start_daemon()}

            if cmd == "DAEMON_STOP":
                self.stop_daemon()
                return {"ok": True}

            if cmd == "DASHBOARD_START":
                return {"ok": self.start_dashboard()}

            if cmd == "DASHBOARD_STOP":
                self.stop_dashboard()
                return {"ok": True}

            if cmd == "SYSTEM_STATUS":
                return {
                    "ok": True,
                    "daemon_running":    self.daemon_is_running(),
                    "dashboard_running": self.dashboard_is_running(),
                }

        except Exception:
            logging.exception("Error handling ZMQ command: %s", raw)
            return {"ok": False, "error": "exception"}

        return {"ok": False, "error": f"unknown command: {cmd}"}


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

def make_gui(controller: Moore6mController, poll_ms: int = GUI_POLL_MS):
    if tk is None:
        logging.warning("tkinter not available — running headless")
        return None

    root = tk.Tk()
    root.title("Moore 6m Dish Controller")
    root.resizable(True, True)

    def on_close():
        controller.stop()
        root.quit()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # ---- Antenna status ----
    status_frm = ttk.LabelFrame(root, text="Antenna Status", padding=6)
    status_frm.pack(fill="x", padx=8, pady=(8, 2))

    state_var = tk.StringVar(value="—")
    pos_var   = tk.StringVar(value="az: —  el: —")
    cal_var   = tk.StringVar(value="—")
    mode_var  = tk.StringVar(value="—")

    for col, (label, var) in enumerate([
        ("FSM State", state_var),
        ("Position",  pos_var),
        ("CalSts",    cal_var),
        ("Loop",      mode_var),
    ]):
        ttk.Label(status_frm, text=label, foreground="#555").grid(
            row=0, column=col * 2, sticky="w", padx=(8, 2))
        ttk.Label(status_frm, textvariable=var,
                  font=("TkFixedFont", 10, "bold")).grid(
            row=0, column=col * 2 + 1, sticky="w", padx=(0, 16))

    # ---- Services ----
    svc_frm = ttk.LabelFrame(root, text="Services", padding=6)
    svc_frm.pack(fill="x", padx=8, pady=2)

    daemon_status_var    = tk.StringVar(value="stopped")
    dashboard_status_var = tk.StringVar(value="stopped")

    def _refresh_svc():
        daemon_status_var.set(
            "running" if controller.daemon_is_running() else "stopped")
        dashboard_status_var.set(
            "running" if controller.dashboard_is_running() else "stopped")

    def _start_daemon():
        controller.start_daemon()
        _refresh_svc()

    def _stop_daemon():
        threading.Thread(target=controller.stop_daemon, daemon=True).start()
        root.after(500, _refresh_svc)

    def _start_dashboard():
        controller.start_dashboard()
        _refresh_svc()

    def _stop_dashboard():
        threading.Thread(target=controller.stop_dashboard, daemon=True).start()
        root.after(500, _refresh_svc)

    ttk.Button(svc_frm, text="Start Daemon",    command=_start_daemon).grid(
        row=0, column=0, padx=4, pady=2)
    ttk.Button(svc_frm, text="Stop Daemon",     command=_stop_daemon).grid(
        row=0, column=1, padx=4, pady=2)
    ttk.Label(svc_frm,  text="Daemon:").grid(row=0, column=2, padx=(12, 2))
    ttk.Label(svc_frm,  textvariable=daemon_status_var).grid(
        row=0, column=3, padx=(0, 12))

    ttk.Button(svc_frm, text="Start Dashboard", command=_start_dashboard).grid(
        row=1, column=0, padx=4, pady=2)
    ttk.Button(svc_frm, text="Stop Dashboard",  command=_stop_dashboard).grid(
        row=1, column=1, padx=4, pady=2)
    ttk.Label(svc_frm,  text="Dashboard:").grid(row=1, column=2, padx=(12, 2))
    ttk.Label(svc_frm,  textvariable=dashboard_status_var).grid(
        row=1, column=3, padx=(0, 12))

    # ---- Motor control ----
    motor_frm = ttk.LabelFrame(root, text="Motor Control", padding=6)
    motor_frm.pack(fill="x", padx=8, pady=2)

    estop_engaged = tk.BooleanVar(value=bool(controller.moore6m.safe_mode))

    # Forward-declare so the lambda can reference it after creation
    estop_btn_ref = [None]

    def _do_estop():
        btn = estop_btn_ref[0]
        if not estop_engaged.get():
            def _engage():
                m = controller.moore6m
                if hasattr(m, "serial") and m.serial:
                    try:
                        with m._serial_lock:
                            for _ in range(3):
                                m.serial.write(b"\r")
                                time.sleep(0.003)
                    except Exception:
                        pass
                for _ in range(3):
                    controller._send_spa_immediate()
                    time.sleep(0.05)
                controller.moore6m.safe_mode = True
                controller._log("E-STOP ENGAGED — motion commands blocked")
            threading.Thread(target=_engage, daemon=True).start()
            estop_engaged.set(True)
            if btn:
                btn.configure(
                    text="⚠ E-STOP ENGAGED — click to release",
                    fg="white", bg="#cc0000", activebackground="#990000",
                )
        else:
            controller.moore6m.safe_mode = False
            estop_engaged.set(False)
            if btn:
                btn.configure(
                    text="⚠ EMERGENCY STOP",
                    fg="red", bg="SystemButtonFace",
                    activebackground="SystemButtonFace",
                )
            controller._log("E-stop released — motion commands re-enabled")

    def _do_flush_startup():
        m = controller.moore6m
        if hasattr(m, "serial") and m.serial:
            try:
                with m._serial_lock:
                    for _ in range(8):
                        m.serial.write(b"\r")
                        time.sleep(0.003)
                threading.Thread(target=m.startup, daemon=True).start()
                controller._log("Flush+Startup initiated")
            except Exception:
                logging.exception("Flush+Startup failed")
                controller._log("ERROR: Flush+Startup failed")

    def _do_calibrate():
        threading.Thread(
            target=controller.moore6m.calibrate, daemon=True
        ).start()
        controller._log("Calibration initiated")

    estop_btn = tk.Button(
        motor_frm, text="⚠ EMERGENCY STOP", command=_do_estop,
        fg="red", font=("TkDefaultFont", 10, "bold"), relief="raised", bd=2,
    )
    estop_btn.grid(row=0, column=0, padx=8, pady=2)
    estop_btn_ref[0] = estop_btn

    ttk.Button(motor_frm, text="Flush + Startup",
               command=_do_flush_startup).grid(row=0, column=1, padx=4, pady=2)
    ttk.Button(motor_frm, text="Calibrate",
               command=_do_calibrate).grid(row=0, column=2, padx=4, pady=2)

    # Reflect initial safe_mode state in button appearance
    if bool(controller.moore6m.safe_mode):
        estop_btn.configure(
            text="⚠ E-STOP ENGAGED — click to release",
            fg="white", bg="#cc0000", activebackground="#990000",
        )

    # ---- Serial comms ----
    comm_frm = ttk.LabelFrame(root, text="Recent Serial Communications", padding=6)
    comm_frm.pack(fill="both", padx=8, pady=2)

    comm_list   = tk.Listbox(comm_frm, height=6, font=("TkFixedFont", 9))
    comm_scroll = ttk.Scrollbar(comm_frm, orient="vertical",
                                command=comm_list.yview)
    comm_list.configure(yscrollcommand=comm_scroll.set)
    comm_list.pack(side="left", fill="both", expand=True)
    comm_scroll.pack(side="right", fill="y")

    # ---- Log pane ----
    log_frm = ttk.LabelFrame(root, text="Controller Log", padding=6)
    log_frm.pack(fill="both", expand=True, padx=8, pady=(2, 8))

    log_text = scrolledtext.ScrolledText(
        log_frm, height=10, font=("TkFixedFont", 9),
        state="disabled", wrap="word",
    )
    log_text.pack(fill="both", expand=True)

    # Track how many lines already written to widget to avoid duplication
    _log_written = [0]

    def _append_new_log_lines():
        lines     = controller.get_log_lines()
        new_lines = lines[_log_written[0]:]
        if not new_lines:
            return
        log_text.configure(state="normal")
        for line in new_lines:
            log_text.insert("end", line + "\n")
        line_count = int(log_text.index("end-1c").split(".")[0])
        if line_count > LOG_MAXLINES:
            log_text.delete("1.0", f"{line_count - LOG_MAXLINES}.0")
        log_text.configure(state="disabled")
        log_text.see("end")
        _log_written[0] = len(lines)

    # ---- Periodic update ----
    def _update_gui():
        try:
            fsm = controller.moore6m.get_fsm_status()
            state_var.set(fsm.get("state", "—"))
            pos_var.set(
                f"az: {controller.moore6m.az:.3f}°  "
                f"el: {controller.moore6m.el:.3f}°"
            )
            cal_var.set(getattr(controller.moore6m, "CalSts", "—") or "—")
            mode_var.set(getattr(controller.moore6m, "mode",   "—") or "—")

            comms = controller.moore6m.get_recent_serial_communications(limit=10)
            comm_list.delete(0, tk.END)
            for c in reversed(comms):
                comm_list.insert(
                    tk.END,
                    f"{c.get('time', '')}  "
                    f"{c.get('direction', ''):4s}  "
                    f"{c.get('payload', '')}",
                )
            _refresh_svc()
        except Exception:
            pass

        if controller.log_is_dirty():
            _append_new_log_lines()

        root.after(poll_ms, _update_gui)

    root.after(poll_ms, _update_gui)
    return root


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    # Required on Windows for multiprocessing with spawn start method
    multiprocessing.freeze_support()

    parser = argparse.ArgumentParser(
        description="Moore 6m Controller — serial worker, daemon, dashboard"
    )
    parser.add_argument("--port",        default="COM1")
    parser.add_argument("--baud",        default=115200,            type=int)
    parser.add_argument("--config_dir",  default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--config_file", default=DEFAULT_CONFIG_FILE)
    parser.add_argument("--cmd-port",    default=DEFAULT_CMD_PORT,  type=int)
    parser.add_argument("--estop-port",  default=DEFAULT_ESTOP_PORT, type=int)
    parser.add_argument("--safe-mode",   action="store_true")
    parser.add_argument("--nogui",       action="store_true")
    parser.add_argument("--autostart",   action="store_true",
                        help="Auto-start daemon and dashboard on launch")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(threadName)s]: %(message)s",
    )

    try:
        controller = Moore6mController(
            serial_port=args.port,
            baudrate=args.baud,
            config_dir=args.config_dir,
            config_file=args.config_file,
            cmd_port=args.cmd_port,
            estop_port=args.estop_port,
            safe_mode=args.safe_mode,
        )
    except Exception as exc:
        logging.exception("Failed to initialise controller")
        if tk is not None:
            _root = tk.Tk()
            _root.withdraw()
            try:
                from tkinter import messagebox
                messagebox.showerror(
                    "Controller Init Failed",
                    f"Could not initialise Moore6mController:\n\n{exc}\n\n"
                    "Check the log for details.",
                )
            except Exception:
                pass
            _root.destroy()
        os._exit(1)

    controller.start()
    controller._log("Controller started")

    if args.safe_mode:
        controller._log("Safe mode enabled on startup — motion commands blocked")

    if args.autostart:
        controller.start_daemon()
        controller.start_dashboard()

    if args.nogui:
        controller._log("Headless mode — press Ctrl-C to quit")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        controller.stop()
    else:
        gui = make_gui(controller)
        if gui is None:
            logging.warning("GUI unavailable — running headless")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            controller.stop()
        else:
            try:
                gui.mainloop()
            except KeyboardInterrupt:
                pass
            controller.stop()

    # Bypass Python's shutdown sequence entirely. ZMQ C-level threads and
    # waitress/Dash internals cannot block os._exit().
    os._exit(0)


if __name__ == "__main__":
    main()