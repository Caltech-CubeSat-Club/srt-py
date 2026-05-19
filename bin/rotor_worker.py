#!/usr/bin/env python3
"""rotor_worker.py

Standalone worker that owns the serial connection to the rotor controller.

Features:
- ZMQ command socket (request/response) for commands like STATUS, POINT, CALIBRATE, STARTUP
- ZMQ estop socket (PULL) for immediate SPA (stop-all) commands that bypass the command queue
- Runs a `Moore6mSerial` instance (from `srt.rotor_control`) to manage normal queued commands
- Tkinter GUI showing FSM state, position, recent serial comms, estop button, startup/refresh
- Buttons to start/stop/restart the `bin/srt_runner.py` web app and show its status

This is intentionally minimal but functional; adapt ports and paths via CLI args.
"""

import argparse
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Optional

import zmq

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:
    tk = None

from srt.daemon.rotor_control.moore6m_serial import Moore6mSerial


DEFAULT_CMD_PORT = 5566
DEFAULT_ESTOP_PORT = 5567


class Moore6mWorker:
    def __init__(self, port, baudrate, cmd_port=DEFAULT_CMD_PORT, estop_port=DEFAULT_ESTOP_PORT, runner_path=None, safe_mode=False):
        self.cmd_port = cmd_port
        self.estop_port = estop_port
        self.runner_path = runner_path
        self._ctx = zmq.Context()
        self._running = threading.Event()
        self._running.set()

        logging.warning('initializing Moore6mSerial on %s @ %d', port, baudrate)
        self.moore6m = Moore6mSerial(port=port, baudrate=baudrate, safe_mode=bool(safe_mode))

        # ZMQ sockets
        self.cmd_socket = self._ctx.socket(zmq.REP)
        self.cmd_socket.bind(f"tcp://*:{self.cmd_port}")

        self.estop_socket = self._ctx.socket(zmq.PULL)
        self.estop_socket.bind(f"tcp://*:{self.estop_port}")

        self._threads = []

        # process handle for srt_runner
        self.runner_proc: Optional[subprocess.Popen] = None

    def start(self):
        t = threading.Thread(target=self._cmd_loop, name='rotor-cmd-loop', daemon=True)
        t.start()
        self._threads.append(t)

        e = threading.Thread(target=self._estop_loop, name='rotor-estop-loop', daemon=True)
        e.start()
        self._threads.append(e)

    def stop(self):
        self._running.clear()
        try:
            self.cmd_socket.close()
            self.estop_socket.close()
            self._ctx.term()
        except Exception:
            pass
        # cleanup moore6m
        try:
            self.moore6m.cleanup()
        except Exception:
            pass

    def _cmd_loop(self):
        while self._running.is_set():
            try:
                msg = self.cmd_socket.recv_string(flags=0)
            except Exception:
                break
            resp = self._handle_command(msg)
            try:
                self.cmd_socket.send_string(json.dumps(resp))
            except Exception:
                logging.exception('failed to send response')

    def _estop_loop(self):
        poller = zmq.Poller()
        poller.register(self.estop_socket, zmq.POLLIN)
        while self._running.is_set():
            socks = dict(poller.poll(200))
            if self.estop_socket in socks:
                try:
                    msg = self.estop_socket.recv_string(flags=0)
                except Exception:
                    continue
                logging.warning('estop message received: %s', msg)
                # immediate SPA bypassing queue
                self._send_spa_immediate()

    def _send_spa_immediate(self):
        try:
            # Acquire the serial lock and write SPA directly to the device, bypassing the queue
            if hasattr(self.moore6m, '_serial_lock') and hasattr(self.moore6m, 'serial') and self.moore6m.serial:
                with self.moore6m._serial_lock:
                    self.moore6m._record_serial_comm('sent', 'SPA (immediate)')
                    try:
                        self.moore6m.serial.write(b'SPA\r')
                        # short sleep so hardware has time to act
                        time.sleep(0.01)
                    except Exception:
                        logging.exception('failed writing immediate SPA')
                # mirror behavior of spa() regarding state
                if self.moore6m.state not in (Moore6mSerial.State.FAULT, Moore6mSerial.State.SHUTDOWN):
                    self.moore6m._transition_state(Moore6mSerial.State.READY, 'stop-all immediate')
                return True
        except Exception:
            logging.exception('estop immediate failed')
        return False

    def _handle_command(self, raw: str):
        raw = raw.strip()
        parts = raw.split()
        cmd = parts[0].upper() if parts else ''
        try:
            if cmd in ('STATUS', 'GET_STATUS'):
                fsm = self.moore6m.get_fsm_status()
                az, el = self.moore6m.status()
                diagnostics = {
                    'mode': getattr(self.moore6m, 'mode', None),
                    'CalSts': getattr(self.moore6m, 'CalSts', None),
                    'AzBrkOn': getattr(self.moore6m, 'AzBrkOn', None),
                    'ElBrkOn': getattr(self.moore6m, 'ElBrkOn', None),
                    'EmStopOn': getattr(self.moore6m, 'EmStopOn', None),
                    'azerr': getattr(self.moore6m, 'azerr', None),
                    'elerr': getattr(self.moore6m, 'elerr', None),
                    'amp_currents': getattr(self.moore6m, 'amp_currents', None),
                    'safe_mode': getattr(self.moore6m, 'safe_mode', None),
                }
                return {'ok': True, 'fsm': fsm, 'az': az, 'el': el, 'diagnostics': diagnostics}
            if cmd == 'POINT' and len(parts) >= 3:
                az = float(parts[1]); el = float(parts[2])
                res = self.moore6m.point(az, el)
                return {'ok': True, 'response': res}
            if cmd == 'POINT_RADEC' and len(parts) >= 3:
                ra = float(parts[1]); dec = float(parts[2])
                res = self.moore6m.point_radec(ra, dec)
                return {'ok': True, 'response': res}
            if cmd == 'CALIBRATE':
                threading.Thread(target=self.moore6m.calibrate, daemon=True).start()
                return {'ok': True}
            if cmd == 'STARTUP':
                threading.Thread(target=self.moore6m.startup, daemon=True).start()
                return {'ok': True}
            if cmd == 'SPA' or cmd == 'ESTOP':
                # route to immediate estop
                ok = self._send_spa_immediate()
                return {'ok': ok}
            if cmd == 'GET_COMM_HISTORY':
                entries = self.moore6m.get_recent_command_history(limit=20)
                return {'ok': True, 'history': entries}
            if cmd == 'GET_SERIAL_COMM':
                entries = self.moore6m.get_recent_serial_communications(limit=20)
                return {'ok': True, 'serial': entries}
            if cmd == 'RUNNER_START':
                ok = self.start_runner()
                return {'ok': ok}
            if cmd == 'RUNNER_STOP':
                ok = self.stop_runner()
                return {'ok': ok}
            if cmd == 'RUNNER_RESTART':
                ok = self.restart_runner()
                return {'ok': ok}
        except Exception:
            logging.exception('error handling command: %s', raw)
            return {'ok': False, 'error': 'exception'}
        return {'ok': False, 'error': 'unknown command'}

    # srt_runner process control
    def start_runner(self):
        if self.runner_proc and self.runner_proc.poll() is None:
            return True
        if not self.runner_path:
            logging.error('no runner path configured')
            return False
        try:
            self.runner_proc = subprocess.Popen(
                [sys.executable, self.runner_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            return True
        except Exception:
            logging.exception('failed to start runner')
            return False

    def stop_runner(self):
        if not self.runner_proc:
            return True
        try:
            if self.runner_proc.poll() is None:
                try:
                    os.killpg(self.runner_proc.pid, signal.SIGTERM)
                except Exception:
                    self.runner_proc.terminate()
                try:
                    self.runner_proc.wait(timeout=3)
                except Exception:
                    try:
                        os.killpg(self.runner_proc.pid, signal.SIGKILL)
                    except Exception:
                        self.runner_proc.kill()
            self.runner_proc = None
            return True
        except Exception:
            logging.exception('failed to stop runner')
            return False

    def restart_runner(self):
        ok = self.stop_runner()
        if not ok:
            return False
        return self.start_runner()


def make_gui(worker: Moore6mWorker, poll_ms=500):
    if tk is None:
        logging.warning('tkinter not available, skipping GUI')
        return None

    root = tk.Tk()
    root.title('Moore 6m Dish Controller')

    frm = ttk.Frame(root, padding=8)
    frm.grid()

    state_var = tk.StringVar(value='-')
    pos_var = tk.StringVar(value='az: -, el: -')

    ttk.Label(frm, text='State:').grid(column=0, row=0, sticky='w')
    ttk.Label(frm, textvariable=state_var).grid(column=1, row=0, sticky='w')
    ttk.Label(frm, text='Position:').grid(column=0, row=1, sticky='w')
    ttk.Label(frm, textvariable=pos_var).grid(column=1, row=1, sticky='w')

    comm_list = tk.Listbox(frm, height=8, width=60)
    comm_list.grid(column=0, row=2, columnspan=3, pady=(6, 6))

    safe_mode_var = tk.BooleanVar(value=bool(getattr(worker.moore6m, 'safe_mode', False)))
    def toggle_safe_mode():
        worker.moore6m.safe_mode = bool(safe_mode_var.get())
    ttk.Checkbutton(frm, text='Safe Mode', variable=safe_mode_var, command=toggle_safe_mode).grid(column=0, row=3, sticky='w')

    def do_estop():
        ok = worker._send_spa_immediate()
        messagebox.showinfo('Estop', 'sent' if ok else 'failed')

    def do_flush_and_startup():
        # flush serial by sending newlines and then run startup
        if hasattr(worker.moore6m, 'serial') and worker.moore6m.serial:
            try:
                with worker.moore6m._serial_lock:
                    for _ in range(6):
                        worker.moore6m.serial.write(b'\r')
                        time.sleep(0.003)
                threading.Thread(target=worker.moore6m.startup, daemon=True).start()
            except Exception:
                logging.exception('flush failed')

    ttk.Button(frm, text='EMERGENCY STOP', command=do_estop).grid(column=1, row=3)
    ttk.Button(frm, text='Flush+Startup', command=do_flush_and_startup).grid(column=2, row=3)

    runner_status_var = tk.StringVar(value='stopped')
    def runner_start():
        worker.start_runner(); runner_status_var.set('running' if worker.runner_proc and worker.runner_proc.poll() is None else 'stopped')
    def runner_stop():
        worker.stop_runner(); runner_status_var.set('stopped')

    def runner_restart():
        worker.restart_runner()
        runner_status_var.set('running' if worker.runner_proc and worker.runner_proc.poll() is None else 'stopped')

    ttk.Button(frm, text='Runner Start', command=runner_start).grid(column=0, row=4)
    ttk.Button(frm, text='Runner Stop', command=runner_stop).grid(column=1, row=4)
    ttk.Button(frm, text='Runner Restart', command=runner_restart).grid(column=2, row=4)
    ttk.Label(frm, textvariable=runner_status_var).grid(column=3, row=4)

    def update_gui():
        try:
            fsm = worker.moore6m.get_fsm_status()
            state_var.set(fsm.get('state', '-'))
            az, el = worker.moore6m.status()
            pos_var.set(f'az: {az:.3f}, el: {el:.3f}')
            comms = worker.moore6m.get_recent_serial_communications(limit=12)
            comm_list.delete(0, tk.END)
            for c in comms[::-1]:
                comm_list.insert(tk.END, f"{c.get('time','')}: {c.get('direction','')}: {c.get('payload','')}")
            runner_status_var.set('running' if worker.runner_proc and worker.runner_proc.poll() is None else 'stopped')
            safe_mode_var.set(bool(getattr(worker.moore6m, 'safe_mode', False)))
        except Exception:
            pass
        root.after(poll_ms, update_gui)

    root.after(poll_ms, update_gui)
    return root


def main():
    parser = argparse.ArgumentParser(description='Rotor serial worker with ZMQ & GUI')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='serial port')
    parser.add_argument('--baud', default=115200, type=int, help='baudrate')
    parser.add_argument('--cmd-port', default=DEFAULT_CMD_PORT, type=int, help='ZMQ command port')
    parser.add_argument('--estop-port', default=DEFAULT_ESTOP_PORT, type=int, help='ZMQ estop port')
    parser.add_argument('--runner', default=os.path.join(os.path.dirname(__file__), 'srt_runner.py'), help='path to srt_runner.py')
    parser.add_argument('--safe-mode', action='store_true', help='start with motion/control commands blocked')
    parser.add_argument('--nogui', action='store_true', help='do not show Tkinter GUI')
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s')

    worker = Moore6mWorker(
        port=args.port,
        baudrate=args.baud,
        cmd_port=args.cmd_port,
        estop_port=args.estop_port,
        runner_path=args.runner,
        safe_mode=args.safe_mode,
    )
    worker.start()

    gui = None
    if not args.nogui:
        gui = make_gui(worker)

    try:
        if gui:
            gui.mainloop()
        else:
            # keep running until interrupted
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()


if __name__ == '__main__':
    main()
