#!/usr/bin/env python3
"""Moore6mController.py

Single root process for all Caltech 6m telescope control activities.

Replaces rotor_worker.py + srt_runner.py.  Owns:
  - The serial connection to the Moore 6m servo controller (Moore6mSerial)
  - A ZMQ REP socket for queued command/response traffic (port 5566)
  - A ZMQ PULL socket for immediate e-stop bypass (port 5567)
  - The SRT daemon (SmallRadioTelescopeDaemon) as a daemon thread
  - The Waitress/Dash dashboard server as a daemon thread
  - A Tkinter GUI for local monitoring, manual control, and start/stop of the
    daemon and dashboard

Because the daemon and dashboard run as daemon=True threads inside this
process, they are guaranteed to die when the Tkinter window is closed.
No orphan processes, no port TIME_WAIT issues on restart.

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
import os
import threading
import time
from pathlib import Path
from typing import Optional

import zmq

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
except Exception:
    tk = None

from srt.daemon.rotor_control.moore6m_serial import Moore6mSerial
from srt import config_loader


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_CMD_PORT   = 5566
DEFAULT_ESTOP_PORT = 5567
DEFAULT_CONFIG_DIR  = "config"
DEFAULT_CONFIG_FILE = "config.yaml"

GUI_POLL_MS = 500          # how often the Tkinter GUI refreshes
LOG_MAXLINES = 500         # max lines kept in the GUI log widget


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_config(config_dir: str, config_file: str) -> Optional[dict]:
    """Load and validate the SRT YAML config.  Returns None on failure."""
    config_dir_path  = Path(config_dir)
    config_path      = config_dir_path / config_file
    schema_path      = config_dir_path / "schema.yaml"
    sky_coords_path  = config_dir_path / "sky_coords.csv"

    missing = []
    if not config_dir_path.is_dir():   missing.append(str(config_dir_path))
    if not sky_coords_path.is_file():  missing.append(str(sky_coords_path))
    if not schema_path.is_file():      missing.append(str(schema_path))
    if not config_path.is_file():      missing.append(str(config_path))
    if missing:
        logging.error("Missing required config paths: %s", missing)
        return None
    if not config_loader.validate_yaml_schema(config_path, schema_path):
        logging.error("Config YAML failed schema validation: %s", config_path)
        return None
    return config_loader.load_yaml(config_path)


# ---------------------------------------------------------------------------
# Main controller class
# ---------------------------------------------------------------------------

class Moore6mController:
    """
    Owns the serial port, ZMQ sockets, and child threads for the daemon and
    dashboard.  Intended to be the single root object for the entire system.
    """

    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        config_dir: str,
        config_file: str,
        cmd_port: int  = DEFAULT_CMD_PORT,
        estop_port: int = DEFAULT_ESTOP_PORT,
        safe_mode: bool = False,
    ):
        self.serial_port = serial_port
        self.baudrate    = baudrate
        self.config_dir  = config_dir
        self.config_file = config_file
        self.cmd_port    = cmd_port
        self.estop_port  = estop_port
        self.safe_mode   = safe_mode

        self._running = threading.Event()
        self._running.set()

        # Load config once; used by both daemon and dashboard threads
        self.config_dict = _load_config(config_dir, config_file)
        if self.config_dict is None:
            raise RuntimeError(
                f"Failed to load config from {config_dir}/{config_file}. "
                "Check logs for details."
            )

        # Log buffer for GUI (thread-safe append, read from GUI thread only)
        self._log_lock   = threading.Lock()
        self._log_lines  = []           # list[str]
        self._log_dirty  = False        # set True when new lines arrive

        # Serial interface — initialised here so startup errors are visible
        logging.warning("Initialising Moore6mSerial on %s @ %d baud", serial_port, baudrate)
        self.moore6m = Moore6mSerial(
            port=serial_port,
            baudrate=baudrate,
            safe_mode=bool(safe_mode),
        )
        logging.warning("Moore6mSerial ready")

        # ZMQ context and sockets
        self._zmq_ctx = zmq.Context()

        self._cmd_socket = self._zmq_ctx.socket(zmq.REP)
        self._cmd_socket.bind(f"tcp://*:{self.cmd_port}")

        self._estop_socket = self._zmq_ctx.socket(zmq.PULL)
        self._estop_socket.bind(f"tcp://*:{self.estop_port}")

        # Child thread handles
        self._threads: list[threading.Thread] = []

        # Daemon and dashboard state
        self._daemon = None
        self._daemon_thread: Optional[threading.Thread] = None
        self._dashboard_thread: Optional[threading.Thread] = None
        self._dashboard_server = None

    # ------------------------------------------------------------------
    # Logging helpers
    # ------------------------------------------------------------------

    def _log(self, msg: str):
        """Append a timestamped line to the internal log buffer and logging."""
        ts  = time.strftime("%H:%M:%S")
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
    # Start / stop
    # ------------------------------------------------------------------

    def start(self):
        """Start ZMQ listener threads.  Call before entering mainloop."""
        cmd_thread = threading.Thread(
            target=self._cmd_loop,
            name="zmq-cmd-loop",
            daemon=True,
        )
        cmd_thread.start()
        self._threads.append(cmd_thread)

        estop_thread = threading.Thread(
            target=self._estop_loop,
            name="zmq-estop-loop",
            daemon=True,
        )
        estop_thread.start()
        self._threads.append(estop_thread)

    def stop(self):
        """Clean shutdown: stop daemon, release serial, close ZMQ."""
        self._log("Controller shutting down")
        self._running.clear()

        # Stop daemon gracefully
        self.stop_daemon()

        # Stop dashboard server gracefully
        self.stop_dashboard()

        # Release serial port
        try:
            self.moore6m.cleanup()
        except Exception:
            logging.exception("Error during Moore6mSerial cleanup")

        # Close ZMQ with zero linger so sockets release ports immediately
        for sock in (self._cmd_socket, self._estop_socket):
            try:
                sock.setsockopt(zmq.LINGER, 0)
                sock.close()
            except Exception:
                pass
        try:
            self._zmq_ctx.term()
        except Exception:
            pass

        for t in list(self._threads):
            t.join(timeout=1.0)

    # ------------------------------------------------------------------
    # Daemon thread
    # ------------------------------------------------------------------

    def start_daemon(self) -> bool:
        """Start the SRT daemon as a daemon thread."""
        if self._daemon_thread and self._daemon_thread.is_alive():
            self._log("Daemon already running")
            return True
        try:
            from srt.daemon import daemon as srt_d
            self._daemon = srt_d.SmallRadioTelescopeDaemon(
                self.config_dir,
                self.config_dict,
            )
            self._daemon_thread = threading.Thread(
                target=self._daemon.srt_daemon_main,
                name="srt-daemon",
                daemon=True,   # guaranteed to die with this process
            )
            self._daemon_thread.start()
            self._log("SRT daemon started")
            return True
        except Exception:
            logging.exception("Failed to start daemon")
            self._log("ERROR: daemon failed to start — see log")
            return False

    def stop_daemon(self):
        """Signal the daemon to stop.  The thread is daemon=True so it will
        be forcibly reaped when the process exits even if this takes a moment."""
        if self._daemon is not None:
            try:
                self._daemon.keep_running = False
                self._log("Daemon stop signalled")
            except Exception:
                pass
        self._daemon = None
        # Note: we do NOT join the daemon thread here — it may be blocked on
        # a ZMQ recv or a long rotor move.  daemon=True ensures it is reaped.

    def daemon_is_running(self) -> bool:
        return bool(self._daemon_thread and self._daemon_thread.is_alive())

    # ------------------------------------------------------------------
    # Dashboard thread
    # ------------------------------------------------------------------

    def start_dashboard(self) -> bool:
        """Start the Waitress/Dash dashboard as a daemon thread."""
        if self._dashboard_thread and self._dashboard_thread.is_alive():
            self._log("Dashboard already running")
            return True
        try:
            from waitress import create_server
            from srt.dashboard import app as srt_app

            def _configure_waitress_queue_logging():
                logging.getLogger("waitress.queue").disabled = True

            _configure_waitress_queue_logging()
            app_server, _ = srt_app.generate_app(self.config_dir, self.config_dict)

            host = self.config_dict.get("DASHBOARD_HOST", "127.0.0.1")
            port = self.config_dict.get("DASHBOARD_PORT", 8080)

            self._dashboard_server = create_server(
                app_server,
                host=host,
                port=port,
            )
            self._dashboard_thread = threading.Thread(
                target=self._dashboard_server.run,
                name="srt-dashboard",
                daemon=True,
            )
            self._dashboard_thread.start()
            self._log(f"Dashboard started on http://{host}:{port}")
            return True
        except Exception:
            logging.exception("Failed to start dashboard")
            self._log("ERROR: dashboard failed to start — see log")
            return False

    def dashboard_is_running(self) -> bool:
        return bool(self._dashboard_thread and self._dashboard_thread.is_alive())

    def stop_dashboard(self):
        """Request the dashboard server to stop."""
        if self._dashboard_server is not None:
            try:
                self._dashboard_server.close()
                self._log("Dashboard stop signalled")
            except Exception:
                logging.exception("Failed to stop dashboard")
        self._dashboard_server = None

    # ------------------------------------------------------------------
    # ZMQ command loop  (REQ/REP — queued commands from Moore6mClient)
    # ------------------------------------------------------------------

    def _cmd_loop(self):
        while self._running.is_set():
            try:
                msg = self._cmd_socket.recv_string(flags=0)
            except zmq.error.ZMQError:
                break
            except Exception:
                break
            resp = self._handle_command(msg)
            try:
                self._cmd_socket.send_string(json.dumps(resp))
            except Exception:
                logging.exception("Failed to send ZMQ response")

    # ------------------------------------------------------------------
    # ZMQ e-stop loop  (PULL — immediate SPA bypass)
    # ------------------------------------------------------------------

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
    # E-stop implementation
    # ------------------------------------------------------------------

    def _send_spa_immediate(self) -> bool:
        """Write SPA directly to the serial port, bypassing the command queue.
        This is safe to call from any thread."""
        try:
            if (
                hasattr(self.moore6m, "_serial_lock")
                and hasattr(self.moore6m, "serial")
                and self.moore6m.serial
            ):
                with self.moore6m._serial_lock:
                    self.moore6m._record_serial_comm("sent", "SPA (immediate)")
                    self.moore6m.serial.write(b"SPA\r")
                    time.sleep(0.01)
                if self.moore6m.state not in (
                    Moore6mSerial.State.FAULT,
                    Moore6mSerial.State.SHUTDOWN,
                ):
                    self.moore6m._transition_state(
                        Moore6mSerial.State.READY, "stop-all immediate"
                    )
                self._log("SPA sent (immediate)")
                return True
        except Exception:
            logging.exception("Immediate SPA failed")
            self._log("ERROR: immediate SPA failed")
        return False

    # ------------------------------------------------------------------
    # Command dispatch (ZMQ REQ/REP)
    # ------------------------------------------------------------------

    def _handle_command(self, raw: str) -> dict:
        raw   = raw.strip()
        parts = raw.split()
        cmd   = parts[0].upper() if parts else ""
        try:
            if cmd in ("STATUS", "GET_STATUS"):
                az, el = self.moore6m.status()
                fsm = self.moore6m.get_fsm_status()
                diagnostics = {
                    "mode":        getattr(self.moore6m, "mode",        None),
                    "CalSts":      getattr(self.moore6m, "CalSts",      None),
                    "AzBrkOn":     getattr(self.moore6m, "AzBrkOn",     None),
                    "ElBrkOn":     getattr(self.moore6m, "ElBrkOn",     None),
                    "EmStopOn":    getattr(self.moore6m, "EmStopOn",    None),
                    "azerr":       getattr(self.moore6m, "azerr",       None),
                    "elerr":       getattr(self.moore6m, "elerr",       None),
                    "amp_currents":getattr(self.moore6m, "amp_currents",None),
                    "safe_mode":   getattr(self.moore6m, "safe_mode",   None),
                    "ElUpPreLim":  getattr(self.moore6m, "ElUpPreLim",  None),
                    "ElDnPreLim":  getattr(self.moore6m, "ElDnPreLim",  None),
                    "ElUpFinLim":  getattr(self.moore6m, "ElUpFinLim",  None),
                    "ElDnFinLim":  getattr(self.moore6m, "ElDnFinLim",  None),
                    "AzCwPreLim":  getattr(self.moore6m, "AzCwPreLim",  None),
                    "AzCcwPreLim": getattr(self.moore6m, "AzCcwPreLim", None),
                    "AzCwFinLim":  getattr(self.moore6m, "AzCwFinLim",  None),
                    "AzCcwFinLim": getattr(self.moore6m, "AzCcwFinLim", None),
                    "AzLT180":     getattr(self.moore6m, "AzLT180",     None),
                    "SimMode":     getattr(self.moore6m, "SimMode",     None),
                }
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
                ok = self._send_spa_immediate()
                return {"ok": ok}

            if cmd == "GET_COMM_HISTORY":
                return {
                    "ok": True,
                    "history": self.moore6m.get_recent_command_history(limit=20),
                }

            if cmd == "GET_SERIAL_COMM":
                return {
                    "ok": True,
                    "serial": self.moore6m.get_recent_serial_communications(limit=20),
                }

            if cmd == "DAEMON_START":
                ok = self.start_daemon()
                return {"ok": ok}

            if cmd == "DAEMON_STOP":
                self.stop_daemon()
                return {"ok": True}

            if cmd == "DASHBOARD_START":
                ok = self.start_dashboard()
                return {"ok": ok}

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
        if messagebox.askokcancel("Quit", "Close controller?\nThis will stop the daemon and dashboard."):
            controller.stop()
            root.quit()
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # ---- Top status bar ----
    status_frm = ttk.LabelFrame(root, text="Antenna Status", padding=6)
    status_frm.pack(fill="x", padx=8, pady=(8, 2))

    state_var = tk.StringVar(value="—")
    pos_var   = tk.StringVar(value="az: —  el: —")
    cal_var   = tk.StringVar(value="—")
    mode_var  = tk.StringVar(value="—")

    estop_engaged = tk.BooleanVar(value=bool(controller.moore6m.safe_mode))
    estop_btn = None # forward declaration

    for col, (label, var) in enumerate([
        ("FSM State", state_var),
        ("Position",  pos_var),
        ("CalSts",    cal_var),
        ("Loop",      mode_var),
    ]):
        ttk.Label(status_frm, text=label, foreground="#555").grid(
            row=0, column=col * 2, sticky="w", padx=(8, 2))
        ttk.Label(status_frm, textvariable=var, font=("TkFixedFont", 10, "bold")).grid(
            row=0, column=col * 2 + 1, sticky="w", padx=(0, 16))

    # ---- Service control ----
    svc_frm = ttk.LabelFrame(root, text="Services", padding=6)
    svc_frm.pack(fill="x", padx=8, pady=2)

    daemon_status_var    = tk.StringVar(value="stopped")
    dashboard_status_var = tk.StringVar(value="stopped")

    def _refresh_svc_labels():
        daemon_status_var.set(
            "running" if controller.daemon_is_running() else "stopped"
        )
        dashboard_status_var.set(
            "running" if controller.dashboard_is_running() else "stopped"
        )

    def _start_daemon():
        controller.start_daemon()
        _refresh_svc_labels()

    def _stop_daemon():
        controller.stop_daemon()
        _refresh_svc_labels()

    def _start_dashboard():
        controller.start_dashboard()
        _refresh_svc_labels()

    for col, (label, cmd, status_var) in enumerate([
        ("Daemon",    _start_daemon,    daemon_status_var),
        ("Dashboard", _start_dashboard, dashboard_status_var),
    ]):
        ttk.Button(svc_frm, text=f"Start {label}", command=cmd).grid(
            row=0, column=col * 3, padx=4, pady=2)
        if label == "Daemon":
            ttk.Button(svc_frm, text="Stop Daemon", command=_stop_daemon).grid(
                row=0, column=1, padx=4, pady=2)
        ttk.Label(svc_frm, textvariable=status_var).grid(
            row=0, column=col * 3 + 2, padx=4)

    # ---- Motor control ----
    motor_frm = ttk.LabelFrame(root, text="Motor Control", padding=6)
    motor_frm.pack(fill="x", padx=8, pady=2)

    safe_mode_var = tk.BooleanVar(value=bool(controller.moore6m.safe_mode))
    def _toggle_safe_mode():
        controller.moore6m.safe_mode = bool(safe_mode_var.get())
        controller._log(f"Safe mode {'enabled' if controller.moore6m.safe_mode else 'disabled'}")
    ttk.Checkbutton(
        motor_frm, text="Safe Mode (block motion commands)",
        variable=safe_mode_var, command=_toggle_safe_mode
    ).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=2)

    def _do_estop():
        if not estop_engaged.get():
            # Engage: flush buffer, send SPA multiple times, lock out motion
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
            estop_btn.configure(text="⚠ E-STOP ENGAGED — click to release",
                    fg="white", bg="#cc0000", activebackground="#990000")
        else:
            # Disengage: just re-enable motion, don't automatically restart anything
            controller.moore6m.safe_mode = False
            estop_engaged.set(False)
            estop_btn.configure(text="⚠ EMERGENCY STOP",
                    fg="red", bg="SystemButtonFace", activebackground="SystemButtonFace")
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
        threading.Thread(target=controller.moore6m.calibrate, daemon=True).start()
        controller._log("Calibration initiated")

    estop_btn = tk.Button(
        motor_frm,
        text="⚠ EMERGENCY STOP",
        command=_do_estop,
        fg="red", font=("TkDefaultFont", 10, "bold"),
        relief="raised", bd=2,
    ).grid(
        row=0, column=0, padx=8, pady=2)
    ttk.Button(motor_frm, text="Flush + Startup", command=_do_flush_startup).grid(
        row=0, column=3, padx=4, pady=2)
    ttk.Button(motor_frm, text="Calibrate", command=_do_calibrate).grid(
        row=0, column=4, padx=4, pady=2)


    # ---- Serial comms listbox ----
    comm_frm = ttk.LabelFrame(root, text="Recent Serial Communications", padding=6)
    comm_frm.pack(fill="both", padx=8, pady=2)

    comm_list = tk.Listbox(comm_frm, height=6, font=("TkFixedFont", 9))
    comm_scroll = ttk.Scrollbar(comm_frm, orient="vertical", command=comm_list.yview)
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

    def _append_log(lines):
        log_text.configure(state="normal")
        for line in lines:
            log_text.insert("end", line + "\n")
        # Keep at most LOG_MAXLINES lines in the widget
        line_count = int(log_text.index("end-1c").split(".")[0])
        if line_count > LOG_MAXLINES:
            log_text.delete("1.0", f"{line_count - LOG_MAXLINES}.0")
        log_text.configure(state="disabled")
        log_text.see("end")

    # ---- Periodic GUI update ----
    def _update_gui():
        try:
            # Antenna status
            fsm = controller.moore6m.get_fsm_status()
            state_var.set(fsm.get("state", "—"))
            az, el = controller.moore6m.az, controller.moore6m.el
            pos_var.set(f"az: {az:.3f}°  el: {el:.3f}°")
            cal_var.set(getattr(controller.moore6m, "CalSts", "—") or "—")
            mode_var.set(getattr(controller.moore6m, "mode", "—") or "—")

            # Serial comms listbox
            comms = controller.moore6m.get_recent_serial_communications(limit=10)
            comm_list.delete(0, tk.END)
            for c in reversed(comms):
                t  = c.get("time", "")
                d  = c.get("direction", "")
                p  = c.get("payload", "")
                comm_list.insert(tk.END, f"{t}  {d:4s}  {p}")

            # Service labels
            _refresh_svc_labels()
        except Exception:
            pass

        # Log pane — only redraw when new lines have arrived
        if controller.log_is_dirty():
            _append_log(controller.get_log_lines())

        root.after(poll_ms, _update_gui)

    root.after(poll_ms, _update_gui)
    return root


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Moore 6m Dish Controller — unified serial worker, daemon, and dashboard"
    )
    parser.add_argument("--port",        default="COM1",              help="Serial port (default COM1)")
    parser.add_argument("--baud",        default=115200,  type=int,   help="Baudrate (default 115200)")
    parser.add_argument("--config_dir",  default=DEFAULT_CONFIG_DIR,  help="SRT config directory")
    parser.add_argument("--config_file", default=DEFAULT_CONFIG_FILE, help="Config YAML filename")
    parser.add_argument("--cmd-port",    default=DEFAULT_CMD_PORT,    type=int, help="ZMQ command REP port")
    parser.add_argument("--estop-port",  default=DEFAULT_ESTOP_PORT,  type=int, help="ZMQ e-stop PULL port")
    parser.add_argument("--safe-mode",   action="store_true",         help="Block motion commands on startup")
    parser.add_argument("--nogui",       action="store_true",         help="Headless mode, no Tkinter window")
    parser.add_argument("--autostart",   action="store_true",         help="Auto-start daemon and dashboard on launch")
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
            # Show error in a dialog even if no GUI was requested, so a
            # shortcut-launched user sees the failure reason.
            _root = tk.Tk()
            _root.withdraw()
            messagebox.showerror(
                "Controller Init Failed",
                f"Could not initialise Moore6mController:\n\n{exc}\n\nCheck the log for details.",
            )
            _root.destroy()
        return 1

    controller.start()
    controller._log("Controller started")

    if args.autostart:
        controller.start_daemon()
        controller.start_dashboard()
    
    if args.safe_mode:
        controller._log("Safe mode enabled on startup — motion commands blocked")
        controller.moore6m.safe_mode = True

    if args.nogui:
        controller._log("Headless mode — press Ctrl-C to quit")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            controller.stop()
    else:
        gui = make_gui(controller)
        if gui is None:
            # tkinter unavailable at runtime despite being importable
            logging.warning("GUI unavailable, running headless")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                pass
            finally:
                controller.stop()
        else:
            try:
                gui.mainloop()
            except KeyboardInterrupt:
                pass
            finally:
                controller.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
