"""motors.py

Daemon-side motor interfaces and ZMQ-backed motor clients.

"""
import json
import logging
from datetime import datetime
from collections import deque
from threading import Lock
from abc import ABC, abstractmethod

import zmq


class Motor(ABC):
    """Abstract Class for All Motors Types

    Attributes
    ----------
    port : str
        Serial Port Identifier String for Communicating with the Motor
    baudrate : int
        Baudrate for serial connection
    az_limits : (float, float)
        Tuple of Lower and Upper Azimuth Limits
    el_limits : (float, float)
        Tuple of Lower and Upper Elevation Limits
    serial : serial.Serial
        Serial Object for Communicating with the Motor

    See Also
    --------
    <https://pyserial.readthedocs.io/en/latest/pyserial_api.html>
    """

    def __init__(self, port, baudrate, az_limits, el_limits):
        """Constructor for the Abstract Motor Class

        Parameters
        ----------
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        """
        self.port = port
        self.baudrate = baudrate
        self.az_limits = az_limits
        self.el_limits = el_limits
        self.serial = None
        self.serial_communications = deque(maxlen=200)

    def _record_serial_comm(self, direction, payload):
        """Record a serial communication event for dashboard diagnostics."""
        self.serial_communications.append({
            "time": datetime.now().astimezone().isoformat(timespec="seconds"),
            "direction": direction,
            "payload": str(payload),
        })

    def get_recent_serial_communications(self, limit=12):
        """Returns the latest serial communication entries."""
        if limit <= 0:
            return []
        return list(self.serial_communications)[-limit:]

    @abstractmethod
    def point(self, az, el):
        """Abstract Method Prototype for Pointing a Motor at an AzEl Coordinate

        Parameters
        ----------
        az : float
            Azimuth Coordinate Value to Point At
        el : float
            Elevation Coordinate Value to Point At

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        pass

    @abstractmethod
    def status(self):
        """Abstract Method Prototype for Getting a Motor's Current AzEl Position

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        pass

    def __del__(self):
        """Override of Motor Delete Method to Close Serial Port if Necessary

        Returns
        -------
        None
        """
        if self.serial is not None and self.serial.is_open:
            self.serial.close()


class NoMotor(Motor):
    """
    Class for Simulating a Motor or Using a Stationary Telescope
    """

    def __init__(self, port, baudrate, az_limits, el_limits):
        """
        Initializer for Rot2Motor

        Parameters
        ----------
        port : str
            NOT USED - Needed For Abstract Motor Initializer
        baudrate : int
            Baudrate for serial connection
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits (if Stationary, both should be the same value)
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits (if Stationary, both should be the same value)
        """
        super().__init__(port, baudrate, az_limits, el_limits)
        self.position = (az_limits[0], el_limits[0])
        self.azerr = 0.0
        self.elerr = 0.0

    def point(self, az, el):
        """Changes the Unchanging Position of the Stationary / Simulated Motor

        Parameters
        ----------
        az : float
            Azimuth Coordinate to Point At
        el : float
            Elevation Coordinate to Point At

        Returns
        -------
        None
        """
        self._record_serial_comm("sent", f"SIM_POINT {az:.3f} {el:.3f}")
        self.position = (az, el)

    def status(self):
        """Returns the Unchanging Position of the Stationary / Simulated Motor

        Returns
        -------
        (float, float)
            Current Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        return self.position

DEFAULT_CMD_ENDPOINT = "tcp://127.0.0.1:5566"
DEFAULT_ESTOP_ENDPOINT = "tcp://127.0.0.1:5567"


class Moore6mClient(Motor):
    """ZMQ-backed client that proxies Moore6m commands to the worker."""

    def __init__(
        self,
        port,
        baudrate,
        az_limits,
        el_limits,
        cmd_endpoint=None,
        estop_endpoint=None,
        timeout_s=2.0,
    ):
        super().__init__(port, baudrate, az_limits, el_limits)
        self.cmd_endpoint = cmd_endpoint or DEFAULT_CMD_ENDPOINT
        self.estop_endpoint = estop_endpoint or DEFAULT_ESTOP_ENDPOINT
        self._ctx = zmq.Context.instance()
        self._cmd_socket = self._ctx.socket(zmq.REQ)
        self._cmd_socket.connect(self.cmd_endpoint)
        self._cmd_socket.SNDTIMEO = int(timeout_s * 1000)
        self._cmd_socket.RCVTIMEO = int(timeout_s * 1000)
        self._estop_socket = self._ctx.socket(zmq.PUSH)
        self._estop_socket.connect(self.estop_endpoint)
        self._cmd_lock = Lock()
        self.last_error = ""
        self._fsm_status = {}
        self.az = float(az_limits[0])
        self.el = float(el_limits[0])
        self.azerr = None
        self.elerr = None
        self.mode = None
        self.CalSts = None
        self.AzBrkOn = None
        self.ElBrkOn = None
        self.EmStopOn = None
        self.amp_currents = None
        self.ElUpPreLim  = None
        self.ElDnPreLim  = None
        self.ElUpFinLim  = None
        self.ElDnFinLim  = None
        self.AzCwPreLim  = None
        self.AzCcwPreLim = None
        self.AzCwFinLim  = None
        self.AzCcwFinLim = None
        self.AzLT180     = None
        self.SimMode     = None
        self.safe_mode   = False
        self.last_transition = None
        self.retry_count = 0

    def _send_cmd(self, text):
        with self._cmd_lock:
            try:
                self._cmd_socket.send_string(text)
                raw = self._cmd_socket.recv_string()
            except zmq.error.Again:
                self.last_error = "command timeout"
                return None
            except Exception:
                logging.exception("command failed: %s", text)
                self.last_error = "command exception"
                return None
        try:
            return json.loads(raw)
        except Exception:
            logging.warning("bad response payload: %r", raw)
            return None

    def _apply_status(self, resp):
        if not resp or not resp.get("ok"):
            return
        self._fsm_status = resp.get("fsm", {}) or {}
        self.az = resp.get("az", self.az)
        self.el = resp.get("el", self.el)
        diagnostics = resp.get("diagnostics") or {}
        self.mode = diagnostics.get("mode", self.mode)
        self.CalSts = diagnostics.get("CalSts", self.CalSts)
        self.AzBrkOn = diagnostics.get("AzBrkOn", self.AzBrkOn)
        self.ElBrkOn = diagnostics.get("ElBrkOn", self.ElBrkOn)
        self.EmStopOn = diagnostics.get("EmStopOn", self.EmStopOn)
        self.azerr = diagnostics.get("azerr", self.azerr)
        self.elerr = diagnostics.get("elerr", self.elerr)
        self.amp_currents = diagnostics.get("amp_currents", self.amp_currents)
        self.ElUpPreLim   = diagnostics.get("ElUpPreLim",   self.ElUpPreLim)
        self.ElDnPreLim   = diagnostics.get("ElDnPreLim",   self.ElDnPreLim)
        self.ElUpFinLim   = diagnostics.get("ElUpFinLim",   self.ElUpFinLim)
        self.ElDnFinLim   = diagnostics.get("ElDnFinLim",   self.ElDnFinLim)
        self.AzCwPreLim   = diagnostics.get("AzCwPreLim",   self.AzCwPreLim)
        self.AzCcwPreLim  = diagnostics.get("AzCcwPreLim",  self.AzCcwPreLim)
        self.AzCwFinLim   = diagnostics.get("AzCwFinLim",   self.AzCwFinLim)
        self.AzCcwFinLim  = diagnostics.get("AzCcwFinLim",  self.AzCcwFinLim)
        self.AzLT180      = diagnostics.get("AzLT180",      self.AzLT180)
        self.SimMode      = diagnostics.get("SimMode",      self.SimMode)
        self.safe_mode    = diagnostics.get("safe_mode",    self.safe_mode)
        # FSM metadata forwarded from the fsm sub-dict
        self.last_transition = self._fsm_status.get("last_transition", self.last_transition)
        self.last_error      = self._fsm_status.get("last_error",      self.last_error)
        self.retry_count     = self._fsm_status.get("retry_count",     self.retry_count)

    def status(self) -> tuple[float, float]:
        resp = self._send_cmd("STATUS")
        self._apply_status(resp)
        return self.az, self.el

    def point(self, az, el):
        return self._send_cmd(f"POINT {float(az)} {float(el)}")

    def point_radec(self, ra, dec):
        return self._send_cmd(f"POINT_RADEC {float(ra)} {float(dec)}")

    def calibrate(self):
        return self._send_cmd("CALIBRATE")

    def startup(self):
        return self._send_cmd("STARTUP")

    def spa(self):
        try:
            self._estop_socket.send_string("SPA")
            return True
        except Exception:
            logging.exception("failed to send estop")
            return False

    def stop(self):
        return self.spa()

    def get_recent_serial_communications(self, limit=12):
        resp = self._send_cmd("GET_SERIAL_COMM")
        if not resp or not resp.get("ok"):
            return []
        entries = resp.get("serial") or []
        return entries[-limit:]

    def get_recent_command_history(self, limit=12):
        resp = self._send_cmd("GET_COMM_HISTORY")
        if not resp or not resp.get("ok"):
            return []
        entries = resp.get("history") or []
        return entries[-limit:]

    def get_fsm_status(self) -> dict[str, any]:
        if not self._fsm_status:
            resp = self._send_cmd("STATUS")
            self._apply_status(resp)
        return self._fsm_status

    def cleanup(self):
        try:
            self._cmd_socket.close()
            self._estop_socket.close()
        except Exception:
            pass