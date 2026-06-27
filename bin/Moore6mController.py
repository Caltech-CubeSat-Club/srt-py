"""Moore6mController.py

Single root process for all Caltech 6m telescope control activities.

Architecture
------------
- `Moore6mDriver`: a “singleton” class owned directly in this process and shared with whoever else needs to send commands to the telescope servo motor drivers. basically just the classic `telescope_control.py` with a built-in position and status polling loop. it is the only code that reads and writes directly to the telescope serial port — all public methods either read a copy of the latest received information, or add commands to the send queue.
- `ZMQ PULL` socket: reserved for telescope motor e-stop requests from the web app, processed immediately (port 5567)
- `SRT daemon`: part 1 of the original `srt-py` code, runs all of the science logic — i.e. calculating and updating azimuth/elevation positions of sky objects, reading from the spectrum analyzer,  type threading.Thread — shares `Moore6mDriver` instance; stopped via quit()
- `Dash dashboard`: the web server for the web app. runs the webcam feed. sends e-stop signals to the dedicated `ZMQ PULL` socket. all other info and commands come directly to/from the daemon via a separate ZMQ socket. type multiprocessing.Process — killed instantly on stop
- `Tkinter` GUI: runs on main thread

The daemon runs as a Thread (not a Process) so it can share the single
Moore6mDriver instance that owns the serial port. Stopping the daemon calls
SmallRadioTelescopeDaemon.quit(), which sets keep_running = False and lets
all daemon threads exit naturally within one loop iteration.

The dashboard remains a Process because it has no need for shared serial
state, and Process gives an instant, guaranteed OS-level kill on stop.

The controller itself exits via os._exit(0) which bypasses Python's
shutdown sequence entirely (ZMQ C-level threads cannot block it).

Usage (shortcut target):
    powershell.exe -ExecutionPolicy Bypass -File "C:/path/to/srt-py/run_controller.ps1"

CLI args (all optional, override config where applicable):
    --port        Serial port (default COM1)
    --baud        Baudrate   (default 115200)
    --config_dir  Path to SRT config directory (default ./config)
    --config_file Config YAML filename (default config.yaml)
    --estop-port  ZMQ e-stop PULL port (default 5567)
    --safe-mode   Start with motion/control commands blocked
    --nogui       Headless mode (no Tkinter window; run until Ctrl-C)
    --autostart   Auto-start daemon and dashboard on launch
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

from srt.daemon.rotor_control import make_driver
from srt.daemon import daemon as srt_d
from srt.daemon.telescope_types import LprParams
from srt import config_loader


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_ESTOP_PORT  = 5567
DEFAULT_CONFIG_DIR  = "config"
DEFAULT_CONFIG_FILE = "config.yaml"

GUI_POLL_MS  = 500
LOG_MAXLINES = 500


# ---------------------------------------------------------------------------
# Process target functions (must be module-level for multiprocessing spawn)
# ---------------------------------------------------------------------------

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
        baudrate: int,
        config_dir: str,
        config_file: str,
        estop_port: int = DEFAULT_ESTOP_PORT,
        safe_mode: bool = False,
    ):
        self.baudrate    = baudrate
        self.config_dir  = config_dir
        self.config_file = config_file
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

        lpr_dict = self.config_dict.get("MOTOR_LPR_PARAMS")
        if not lpr_dict:
            raise RuntimeError("MOTOR_LPR_PARAMS missing from config")
        lpr_params = LprParams.model_validate(lpr_dict)

        self.moore6m = make_driver(
            motor_type=self.config_dict.get("MOTOR_TYPE", "MOORE6M"),
            port=self.config_dict.get("MOTOR_PORT", "COM1"),
            baudrate=baudrate,
            az_limits=self.config_dict.get("AZIMUTH_LIMITS", (-89, 449)),
            el_limits=self.config_dict.get("ELEVATION_LIMITS", (15, 81)),
            lpr_params=lpr_params,
            safe_mode=bool(safe_mode),
        )

        self._zmq_ctx      = zmq.Context()
        self._estop_socket = self._zmq_ctx.socket(zmq.PULL)
        self._estop_socket.bind(f"tcp://*:{self.estop_port}")

        self._daemon_proc    : Optional[threading.Thread] = None
        self._daemon_obj     : Optional[srt_d.SmallRadioTelescopeDaemon] = None
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
        t = threading.Thread(target=self._estop_loop, name="zmq-estop-loop", daemon=True)
        t.start()

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
            d = srt_d.SmallRadioTelescopeDaemon(
                self.config_dir,
                self.config_dict,
                rotor=self.moore6m,   # <-- share the existing driver
            )
            t = threading.Thread(
                target=d.srt_daemon_main,
                name="srt-daemon",
                daemon=True,
            )
            t.start()
            self._daemon_proc = t   # rename to self._daemon_thread if you want clarity
            self._daemon_obj = d    # keep a reference so you can call d.quit() on stop
            self._log("Daemon started (thread)")
            return True
        except Exception:
            logging.exception("Failed to start daemon")
            self._log("ERROR: daemon failed to start")
            return False

    def stop_daemon(self):
        d = getattr(self, '_daemon_obj', None)
        t = getattr(self, '_daemon_proc', None)
        if d is None:
            return
        self._daemon_obj = None
        self._daemon_proc = None
        self._log("Stopping daemon")
        d.quit()                  # sets d.keep_running = False
        if t is not None and t.is_alive():
            t.join(timeout=5.0)
        self._log("Daemon stopped")

    def start_spectrum(self) -> bool:
        d = getattr(self, "_daemon_obj", None)
        if d is None:
            self._log("Spectrum start requested but daemon not running")
            return False
        try:
            d.spectrum_driver.start()
            self._log("Spectrum driver start requested")
            return True
        except Exception:
            logging.exception("Failed to start spectrum driver")
            self._log("ERROR: spectrum driver failed to start")
            return False

    def stop_spectrum(self):
        d = getattr(self, "_daemon_obj", None)
        if d is None:
            self._log("Spectrum stop requested but daemon not running")
            return
        try:
            d.spectrum_driver.stop()
            self._log("Spectrum driver stopped")
        except Exception:
            logging.exception("Failed to stop spectrum driver")
            self._log("ERROR: spectrum driver failed to stop")

    def spectrum_status(self) -> str:
        d = getattr(self, "_daemon_obj", None)
        if d is None:
            return "daemon stopped"
        driver = getattr(d, "spectrum_driver", None)
        if driver is None:
            return "unavailable"
        running = bool(getattr(driver, "running", False))
        if not running:
            return "stopped"
        return "connected" if driver.connected else "running"

    def daemon_is_running(self) -> bool:
        t = getattr(self, '_daemon_proc', None)
        return bool(t and t.is_alive())

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
            self.moore6m.spa()
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
            self.moore6m.shutdown()
        except Exception:
            logging.exception("Error during driver shutdown")
        try:
            self._estop_socket.setsockopt(zmq.LINGER, 0)
            self._estop_socket.close()
        except Exception:
            pass
        try:
            self._zmq_ctx.destroy(linger=0)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

def make_gui(controller: Moore6mController, poll_ms: int = GUI_POLL_MS):
    if tk is None:
        logging.warning("tkinter not available — running headless")
        return None

    root = tk.Tk()
    root.title("Moore 6m Dish Controller")
    root.iconbitmap(os.path.join(os.path.pardir, "Caltech6m.ico"))
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
    spectrum_status_var  = tk.StringVar(value="stopped")

    def _refresh_svc():
        daemon_status_var.set(
            "running" if controller.daemon_is_running() else "stopped")
        dashboard_status_var.set(
            "running" if controller.dashboard_is_running() else "stopped")
        spectrum_status_var.set(controller.spectrum_status())

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

    def _start_spectrum():
        controller.start_spectrum()
        _refresh_svc()

    def _stop_spectrum():
        threading.Thread(target=controller.stop_spectrum, daemon=True).start()
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

    ttk.Button(svc_frm, text="Start Spectrum", command=_start_spectrum).grid(
        row=2, column=0, padx=4, pady=2)
    ttk.Button(svc_frm, text="Stop Spectrum",  command=_stop_spectrum).grid(
        row=2, column=1, padx=4, pady=2)
    ttk.Label(svc_frm,  text="Spectrum:").grid(row=2, column=2, padx=(12, 2))
    ttk.Label(svc_frm,  textvariable=spectrum_status_var).grid(
        row=2, column=3, padx=(0, 12))

    # ---- Motor control ----
    motor_frm = ttk.LabelFrame(root, text="Motor Control", padding=6)
    motor_frm.pack(fill="x", padx=8, pady=2)

    estop_engaged = tk.BooleanVar(value=controller.moore6m.get_state().safe_mode)

    # Forward-declare so the lambda can reference it after creation
    estop_btn_ref = [tk.Button()]

    def _do_estop():
        btn = estop_btn_ref[0]
        if not estop_engaged.get():
            def _engage():
                controller.moore6m.spa()          # handles flush + SPA internally
                controller.moore6m.set_safe_mode(True)
                controller._log("E-STOP ENGAGED — motion commands blocked")
            threading.Thread(target=_engage, daemon=True).start()
            estop_engaged.set(True)
            if btn:
                btn.configure(
                    text="⚠ E-STOP ENGAGED — click to release",
                    fg="white", bg="#cc0000", activebackground="#990000",
                )
        else:
            controller.moore6m.set_safe_mode(False)
            estop_engaged.set(False)
            if btn:
                btn.configure(
                    text="⚠ EMERGENCY STOP",
                    fg="red", bg="SystemButtonFace",
                    activebackground="SystemButtonFace",
                )
            controller._log("E-stop released — motion commands re-enabled")

    def _do_flush_startup():
        threading.Thread(target=controller.moore6m.startup, daemon=True).start()
        controller._log("Startup initiated")

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
    if bool(controller.moore6m.get_state().safe_mode):
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
        if tk is None:
            return
        try:
            state = controller.moore6m.get_state()
            state_var.set(state.fsm_state.value)
            pos_var.set(f"az: {state.az:.3f}°  el: {state.el:.3f}°")
            cal_var.set(state.cal_sts)
            mode_var.set(state.loop_mode)

            comms = controller.moore6m.get_recent_serial_communications(limit=10)
            comm_list.delete(0, tk.END)
            for c in reversed(comms):
                comm_list.insert(tk.END,
                    f"{c.get('time','')}  {c.get('direction',''):4s}  {c.get('payload','')}")
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
    parser.add_argument("--baud",        default=115200,            type=int)
    parser.add_argument("--config_dir",  default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--config_file", default=DEFAULT_CONFIG_FILE)
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
            baudrate=args.baud,
            config_dir=args.config_dir,
            config_file=args.config_file,
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