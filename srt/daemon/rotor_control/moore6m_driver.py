"""moore6m_driver.py

Serial driver for the Moore/Caltech 6m rotor.

Architecture
------------
One class owns everything related to the physical serial link:

  _serial_communication_loop  — the only thread that ever calls
                                serial.write() or serial.read_until().
                                Dequeues request dicts, calls
                                _send_command_direct, signals the Future.

  _polling_loop               — runs on a fixed timer, enqueues the
                                standard poll commands (STS, GAE, ERR,
                                amp currents), and applies each response
                                to self._state under the state lock.

All public methods that need a serial response enqueue a request dict
containing a concurrent.futures.Future and return it. Callers that need
to block call fut.result(); callers that don't (e.g. the poll loop)
fire and handle the future in the poll callback.

State is always accessed through get_state(), which returns a copy of
self._state under self._state_lock.
"""

import copy
import logging
import serial
from collections import deque
from concurrent.futures import Future
from dataclasses import replace
from datetime import datetime
from enum import Enum
from queue import Empty, Queue
from threading import Event, RLock, Thread
from time import monotonic, sleep
from typing import Any, Callable, Dict, Optional, Tuple

from parse import parse

from .types import LprParams, RotorState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _local_iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _parse_named(pattern: str, text: str) -> Dict[str, Any]:
    parsed = parse(pattern, text)
    if parsed is None:
        return {}
    named = getattr(parsed, "named", None)
    return named if isinstance(named, dict) else {}


# ---------------------------------------------------------------------------
# FSM state enum
# ---------------------------------------------------------------------------

class DriverState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING   = "connecting"
    STARTUP_SYNC = "startup_sync"
    READY        = "ready"
    SLEWING      = "slewing"
    TRACKING     = "tracking"
    CALIBRATING  = "calibrating"
    FAULT        = "fault"
    RECOVERING   = "recovering"
    SHUTDOWN     = "shutdown"


# ---------------------------------------------------------------------------
# ACK/command metadata
# ---------------------------------------------------------------------------

READ_ONLY_COMMANDS = {
    "STS", "STX", "ERR", "GAE", "VER", "ENC", "NER",
    "2A01SIC", "2A01SIA", "2A02SIC", "2A02SIA", "2A03SIC", "2A03SIA",
}

ACK_PREFIX_BY_COMMAND = {
    "AZEL": "AZEL", "AZL": "AZL", "ZPN": "ZPN", "TON": "TON",
    "LPR": "LPR",   "CLE": "CLE", "SPA": "SPA", "CLO": "CLO",
    "POF": "POF",   "VOF": "VOF", "SEN": "SEN", "TMD": "TMD",
    "EBN": "EBN",   "EBF": "EBF", "ABN": "ABN", "ABF": "ABF",
    "AZV": "AZV",   "ELV": "ELV", "AVS": "AZS",
}

# Delays used for standard commands vs the startup/calibration sequence
SEND_DELAY_NORMAL  = 0.008
RECV_DELAY_NORMAL  = 0.008
SEND_DELAY_STARTUP = 0.008
RECV_DELAY_STARTUP = 0.105

POLL_INTERVAL_S    = 0.2   # how often the poll loop fires
MAX_COMMAND_RETRIES = 2
STARTUP_RETRIES    = 2


# ---------------------------------------------------------------------------
# Main driver class
# ---------------------------------------------------------------------------

class Moore6mDriver:
    """Owns the serial port and all communication with the servo controller.

    Public API
    ----------
    get_state()        -> RotorState   (thread-safe snapshot copy)
    enqueue(cmd, ...)  -> Future[str]  (low-level, used internally)
    startup()
    point(az, el)
    brakes_on() / brakes_off()
    calibrate()
    spa()
    set_safe_mode(bool)
    shutdown()
    get_recent_serial_communications(limit)
    get_recent_command_history(limit)
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        az_limits: Tuple[float, float],
        el_limits: Tuple[float, float],
        lpr_params: LprParams,
        safe_mode: bool = False,
    ):
        if not lpr_params.is_loaded:
            raise ValueError(
                "lpr_params is not fully loaded — ensure all MOTOR_LPR_PARAMS "
                "are present in config before constructing Moore6mDriver."
            )

        self.port      = port
        self.baudrate  = baudrate
        self.az_limits = az_limits
        self.el_limits = el_limits
        self.lpr_params = lpr_params

        # ---- Internal state ----
        self._state      = RotorState(
            fsm_state=DriverState.DISCONNECTED.value,
            lpr=lpr_params,
        )
        self._state_lock  = RLock()
        self._fsm_state   = DriverState.DISCONNECTED   # kept in sync with _state.fsm_state
        self._last_error  = ""
        self._retry_count = 0

        # ---- Safe mode ----
        # Held separately from _state so it can be toggled without locking _state
        self._safe_mode = bool(safe_mode)

        # ---- Serial ----
        self._serial_lock = RLock()
        self.serial_communications = deque(maxlen=200)
        self._comm_lock = RLock()

        self.command_history = deque(maxlen=200)
        self._history_lock = RLock()

        # ---- Command queue ----
        # Each item is a dict:
        #   cmd           bytes
        #   command_text  str
        #   expected_prefix str | None
        #   retries       int
        #   fail_on_error bool
        #   future        concurrent.futures.Future
        #   enqueued_mono float
        #   send_delay    float
        #   recv_delay    float
        self._command_queue = Queue()
        self._stop_event    = Event()

        # ---- Open serial port ----
        self._transition(DriverState.CONNECTING, "opening serial port")
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        logging.warning("Moore6mDriver: serial port %s opened", port)

        # ---- Start background threads ----
        self._serial_thread = Thread(
            target=self._serial_communication_loop,
            name="moore6m-serial",
            daemon=True,
        )
        self._poll_thread = Thread(
            target=self._polling_loop,
            name="moore6m-poll",
            daemon=True,
        )
        self._serial_thread.start()
        self._poll_thread.start()

        # ---- Initial startup (unless safe mode) ----
        try:
            if self._safe_mode:
                logging.warning("Moore6mDriver: safe mode — skipping startup sequence")
                self._transition(DriverState.READY, "safe mode startup")
            else:
                self.startup()
        except Exception:
            self._stop_event.set()
            if self.serial.is_open:
                self.serial.close()
            raise

    # ------------------------------------------------------------------
    # State machine helpers
    # ------------------------------------------------------------------

    def _transition(self, new_state: DriverState, reason: str = ""):
        if self._fsm_state == new_state:
            return
        prev = self._fsm_state
        self._fsm_state = new_state
        ts = _local_iso_now()
        msg = f"state transition: {prev.value} -> {new_state.value}"
        if reason:
            msg += f" ({reason})"
        logging.warning("Moore6mDriver: %s", msg)
        with self._state_lock:
            self._state.fsm_state       = new_state.value
            self._state.last_transition = ts
            if reason and new_state == DriverState.FAULT:
                self._state.last_error = reason

    def _set_fault(self, message: str):
        self._last_error = message
        logging.error("Moore6mDriver fault: %s", message)
        self._transition(DriverState.FAULT, message)

    # ------------------------------------------------------------------
    # Serial I/O (called only from _serial_communication_loop)
    # ------------------------------------------------------------------

    def _record_serial_comm(self, direction: str, payload: str):
        with self._comm_lock:
            self.serial_communications.append({
                "time":      _local_iso_now(),
                "direction": direction,
                "payload":   str(payload),
            })

    def _read_response_line(self) -> str:
        try:
            response = self.serial.read_until(b"\r")
        except (serial.SerialException, serial.portNotOpenError):
            return ""
        response = response.strip().decode("utf-8", errors="replace")
        if response:
            self._record_serial_comm("recv", response)
        return response

    def _verify_response(
        self,
        command_text: str,
        response: str,
        expected_prefix: Optional[str],
    ) -> bool:
        if expected_prefix:
            return response.startswith(expected_prefix)
        if not response:
            return False
        token = command_text.split(",")[0].upper()
        ack = ACK_PREFIX_BY_COMMAND.get(token)
        return response.startswith(ack) if ack else True

    def _read_matching_response(
        self,
        command_text: str,
        expected_prefix: Optional[str],
        max_reads: int = 4,
        max_wait_s: float = 0.25,
    ) -> Tuple[str, bool]:
        deadline     = monotonic() + max(0.01, max_wait_s)
        reads        = 0
        last_response = ""
        while reads < max(1, max_reads) and monotonic() <= deadline:
            response = self._read_response_line()
            reads += 1
            if not response:
                continue
            last_response = response
            if self._verify_response(command_text, response, expected_prefix):
                return response, True
            logging.warning(
                "Moore6mDriver: out-of-sync response waiting for %s: %r",
                command_text, response,
            )
        return last_response, False

    def _send_command_direct(
        self,
        cmd: bytes,
        command_text: str,
        expected_prefix: Optional[str],
        retries: int,
        fail_on_error: bool,
        send_delay: float = SEND_DELAY_NORMAL,
        recv_delay: float = RECV_DELAY_NORMAL,
    ) -> Tuple[str, int, str]:
        """Send bytes, read response, retry on failure. Called only from
        _serial_communication_loop — serial lock is acquired internally."""
        last_response = ""
        for attempt in range(retries + 1):
            with self._serial_lock:
                self._record_serial_comm("sent", command_text)
                self.serial.write(cmd)
                logging.debug("Moore6mDriver sent: %s", cmd)
                sleep(send_delay)
                response, matched = self._read_matching_response(
                    command_text, expected_prefix,
                    max_reads=5, max_wait_s=max(0.05, recv_delay + 0.05),
                )
                logging.debug("Moore6mDriver recv: %s", response)
                if not matched and not response:
                    self.serial.write(b"\r")
                    sleep(send_delay)
                    response, matched = self._read_matching_response(
                        command_text, expected_prefix,
                        max_reads=4, max_wait_s=max(0.05, recv_delay + 0.05),
                    )
                sleep(recv_delay)
                if not matched:
                    extra, extra_matched = self._read_matching_response(
                        command_text, expected_prefix,
                        max_reads=5,
                        max_wait_s=max(0.08, float(self.serial.timeout or 0.5)),
                    )
                    if extra:
                        response = extra
                    matched = extra_matched

            last_response = response
            if matched:
                return response, attempt, ""

            logging.warning(
                "Moore6mDriver: command %s failed attempt %d/%d, response=%r",
                command_text, attempt + 1, retries + 1, response,
            )

        err = f"command failed after retries: {command_text} response={last_response!r}"
        if fail_on_error:
            self._set_fault(err)
        else:
            logging.error("Moore6mDriver: %s", err)
        return "", retries + 1, err

    def _record_command_history(
        self,
        command_text: str,
        expected_prefix: Optional[str],
        response: str,
        retries: int,
        error: str,
        start_mono: float,
        end_mono: float,
        enqueued_mono: float,
    ):
        with self._history_lock:
            self.command_history.append({
                "time":          _local_iso_now(),
                "command":       command_text,
                "expected":      expected_prefix,
                "response":      str(response),
                "retries":       int(retries),
                "success":       (error == ""),
                "error":         str(error),
                "queue_wait_ms": round(max(0.0, (start_mono - enqueued_mono) * 1000), 2),
                "duration_ms":   round(max(0.0, (end_mono - start_mono) * 1000), 2),
                "total_ms":      round(max(0.0, (end_mono - enqueued_mono) * 1000), 2),
            })

    # ------------------------------------------------------------------
    # Serial communication loop (background thread)
    # ------------------------------------------------------------------

    def _serial_communication_loop(self):
        """The only thread that touches serial.write() / serial.read_until().
        Dequeues request dicts, executes them, and resolves their Future."""
        while not self._stop_event.is_set():
            try:
                req = self._command_queue.get(timeout=0.1)
            except Empty:
                continue
            if req is None:
                self._command_queue.task_done()
                break

            enqueued_mono = req["enqueued_mono"]
            start_mono    = monotonic()
            response, retry_count, error = self._send_command_direct(
                req["cmd"],
                req["command_text"],
                req["expected_prefix"],
                req["retries"],
                req["fail_on_error"],
                req.get("send_delay", SEND_DELAY_NORMAL),
                req.get("recv_delay", RECV_DELAY_NORMAL),
            )
            end_mono = monotonic()

            self._record_command_history(
                command_text=req["command_text"],
                expected_prefix=req["expected_prefix"],
                response=response,
                retries=retry_count,
                error=error,
                start_mono=start_mono,
                end_mono=end_mono,
                enqueued_mono=enqueued_mono,
            )

            # Resolve the future so the caller (or poll callback) can proceed
            fut: Future = req["future"]
            if error and not fut.cancelled():
                fut.set_exception(RuntimeError(error))
            elif not fut.cancelled():
                fut.set_result(response)

            self._command_queue.task_done()

    # ------------------------------------------------------------------
    # Public enqueue — builds a request dict and returns a Future
    # ------------------------------------------------------------------

    def enqueue(
        self,
        cmd: str,
        expected_prefix: Optional[str] = None,
        retries: int = MAX_COMMAND_RETRIES,
        fail_on_error: bool = True,
        send_delay: float = SEND_DELAY_NORMAL,
        recv_delay: float = RECV_DELAY_NORMAL,
    ) -> Future:
        """Enqueue a command and return a Future for its response string.

        The caller may call fut.result(timeout=...) to block, or attach a
        callback via fut.add_done_callback(fn) to handle it asynchronously.
        Returns a pre-resolved empty-string Future if the command is blocked
        by safe mode or shutdown state.
        """
        command_token = cmd.split(",")[0].upper()

        # Blocked conditions — return a resolved empty Future
        if self._safe_mode and command_token not in READ_ONLY_COMMANDS:
            logging.warning("Moore6mDriver: safe mode blocked: %s", cmd)
            fut: Future = Future()
            fut.set_result("")
            return fut
        if self._fsm_state == DriverState.SHUTDOWN:
            logging.error("Moore6mDriver: command rejected (shutdown): %s", cmd)
            fut = Future()
            fut.set_result("")
            return fut

        raw = cmd if isinstance(cmd, bytes) else cmd.encode("utf-8")
        if not raw.endswith(b"\r"):
            raw += b"\r"

        fut = Future()
        self._command_queue.put({
            "cmd":            raw,
            "command_text":   cmd,
            "expected_prefix": expected_prefix,
            "retries":        retries,
            "fail_on_error":  fail_on_error,
            "send_delay":     send_delay,
            "recv_delay":     recv_delay,
            "future":         fut,
            "enqueued_mono":  monotonic(),
        })
        return fut

    def enqueue_blocking(
        self,
        cmd: str,
        expected_prefix: Optional[str] = None,
        retries: int = MAX_COMMAND_RETRIES,
        fail_on_error: bool = True,
        send_delay: float = SEND_DELAY_NORMAL,
        recv_delay: float = RECV_DELAY_NORMAL,
        timeout: float = 5.0,
    ) -> str:
        """Enqueue a command and block until the response arrives."""
        serial_timeout = float(self.serial.timeout or 0.5)
        wait = timeout or (retries + 1) * (send_delay + recv_delay + 2 * serial_timeout + 0.2) + 1.0
        fut = self.enqueue(
            cmd, expected_prefix=expected_prefix, retries=retries,
            fail_on_error=fail_on_error, send_delay=send_delay, recv_delay=recv_delay,
        )
        try:
            return fut.result(timeout=wait) or ""
        except Exception:
            return ""

    # ------------------------------------------------------------------
    # Poll parsers — each takes (state, response) and mutates state
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_sts(state: RotorState, response: str):
        named = _parse_named(
            "STS,{mode:d}{ElUpPreLim:l}{ElDnPreLim:l}{ElUpFinLim:l}{ElDnFinLim:l}"
            "{AzCwPreLim:l}{AzCcwPreLim:l}{AzCwFinLim:l}{AzCcwFinLim:l}"
            "{AzLT180:l}{AzBrkOn:l}{ElBrkOn:l}{EmStopOn:l}{CalSts:d}{SimMode:l}",
            response,
        )
        if not named:
            return
        bool_map = {
            "ElUpPreLim": "el_up_pre", "ElDnPreLim": "el_dn_pre",
            "ElUpFinLim": "el_up_fin", "ElDnFinLim": "el_dn_fin",
            "AzCwPreLim": "az_cw_pre", "AzCcwPreLim": "az_ccw_pre",
            "AzCwFinLim": "az_cw_fin", "AzCcwFinLim": "az_ccw_fin",
            "AzLT180":    "az_lt_180",
            "AzBrkOn":    "az_brake",  "ElBrkOn":    "el_brake",
            "EmStopOn":   "estop",     "SimMode":    "sim_mode",
        }
        for src, dst in bool_map.items():
            v = named.get(src)
            if isinstance(v, str) and len(v) == 1:
                setattr(state, dst, v == "T")

        mode_int = named.get("mode")
        if isinstance(mode_int, int):
            state.loop_mode = {0: "Stop", 1: "Track"}.get(mode_int, f"Unknown({mode_int})")

        cal_int = named.get("CalSts")
        if isinstance(cal_int, int):
            state.cal_sts = {
                0: "Not Calibrated",
                1: "Calibrating Now",
                2: "Calibration OK",
            }.get(cal_int, f"Unknown({cal_int})")

    @staticmethod
    def _parse_gae(state: RotorState, response: str):
        named = _parse_named("AZEL,{az:f},{el:f}", response)
        if named:
            state.az = float(named.get("az", state.az))
            state.el = float(named.get("el", state.el))

    @staticmethod
    def _parse_err(state: RotorState, response: str):
        named = _parse_named("ERR,{az_err:f},{el_err:f}", response)
        if named:
            state.az_err = float(named.get("az_err", state.az_err))
            state.el_err = float(named.get("el_err", state.el_err))

    @staticmethod
    def _parse_amp(prefix: str, key: str, state: RotorState, response: str):
        named = _parse_named(f"{prefix}{{current:d}}", response)
        if named:
            curr = int(named.get("current", -999999))
            if prefix not in state.amp_currents:
                state.amp_currents[prefix] = {"commanded": None, "actual": None}
            state.amp_currents[prefix][key] = curr

    # ------------------------------------------------------------------
    # Polling loop (background thread)
    # ------------------------------------------------------------------

    def _apply_sts_fsm(self, state: RotorState):
        """After updating state from STS, update the FSM accordingly."""
        if state.cal_sts == "Calibrating Now":
            self._transition(DriverState.CALIBRATING, "CalSts indicates calibration in progress")
        elif state.loop_mode == "Track":
            if self._fsm_state != DriverState.SLEWING:
                self._transition(DriverState.TRACKING, "RunningTask=1 position loop active")
        elif self._fsm_state not in (
            DriverState.SHUTDOWN, DriverState.FAULT,
            DriverState.SLEWING,  DriverState.CALIBRATING,
        ):
            self._transition(DriverState.READY, "status refresh")

    def _polling_loop(self):
        """Enqueues poll commands on a fixed timer and applies responses to state."""

        # Each entry: (command, expected_prefix, parser_fn)
        # parser_fn signature: (state: RotorState, response: str) -> None
        poll_cmds = [
            ("STS", "STS",  self._parse_sts),
            ("GAE", "AZEL", self._parse_gae),
            ("ERR", "ERR",  self._parse_err),
            ("2A01SIC", "2A01", lambda s, r: self._parse_amp("2A01", "commanded", s, r)),
            ("2A01SIA", "2A01", lambda s, r: self._parse_amp("2A01", "actual",    s, r)),
            ("2A02SIC", "2A02", lambda s, r: self._parse_amp("2A02", "commanded", s, r)),
            ("2A02SIA", "2A02", lambda s, r: self._parse_amp("2A02", "actual",    s, r)),
            # ("2A03SIC", "2A03", lambda s, r: self._parse_amp("2A03", "commanded", s, r)),
            # ("2A03SIA", "2A03", lambda s, r: self._parse_amp("2A03", "actual",    s, r)),
        ]

        while not self._stop_event.is_set():
            tick_start = monotonic()

            # Take a working copy of state, apply all poll responses to it,
            # then write it back under the lock in one operation per poll cycle.
            with self._state_lock:
                working = copy.copy(self._state)

            sts_response = None
            for cmd, prefix, parser in poll_cmds:
                fut = self.enqueue(cmd, expected_prefix=prefix, fail_on_error=False)
                try:
                    response = fut.result(timeout=2.0)
                except Exception:
                    response = ""
                if response:
                    parser(working, response)
                    if cmd == "STS":
                        sts_response = response

            # Apply FSM transitions from STS (done outside lock, uses self._fsm_state)
            if sts_response:
                self._apply_sts_fsm(working)

            # Sync fsm_state field in working copy
            working.fsm_state = self._fsm_state.value
            working.last_poll_time = monotonic()

            with self._state_lock:
                self._state = working

            # Sleep for remainder of poll interval
            elapsed = monotonic() - tick_start
            remaining = POLL_INTERVAL_S - elapsed
            if remaining > 0:
                sleep(remaining)

    # ------------------------------------------------------------------
    # Public state accessor
    # ------------------------------------------------------------------

    def get_state(self) -> RotorState:
        """Return a thread-safe snapshot of current rotor state."""
        with self._state_lock:
            s = copy.copy(self._state)
            # LprParams is only written during calibrate(), not during polling,
            # so shallow copy is safe. Keep lpr reference pointing to the
            # same object — it won't mutate under us.
            return s

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def startup(self):
        self._transition(DriverState.STARTUP_SYNC, "running startup sequence")
        # Flush serial buffer
        with self._serial_lock:
            for _ in range(8):
                self.serial.write(b"\r")
                sleep(0.003)
            self.serial.read_until()

        seq = [
            ("2A01RST", None), ("2A01RST", None),
            ("2A02RST", None), ("2A03RST", None),
            ("2A01EN1", None), ("2A02EN1", None), ("2A03EN1", None),
            ("VER", None), ("VER", "Ver"),
        ]
        startup_ok = False
        for attempt in range(STARTUP_RETRIES + 1):
            startup_ok = True
            for cmd, expected in seq:
                resp = self.enqueue_blocking(
                    cmd, expected_prefix=expected,
                    retries=MAX_COMMAND_RETRIES, fail_on_error=False,
                    send_delay=SEND_DELAY_STARTUP, recv_delay=RECV_DELAY_STARTUP,
                )
                if not resp:
                    startup_ok = False
                    break
                if expected == "Ver" and not resp.startswith("Ver"):
                    startup_ok = False
                    break
            if startup_ok:
                break
            self._transition(
                DriverState.RECOVERING,
                f"startup retry {attempt + 1}/{STARTUP_RETRIES + 1}",
            )
            sleep(0.1)

        if not startup_ok:
            self._set_fault("startup sequence failed")
            raise RuntimeError("Moore6mDriver: startup sequence failed")
        self._transition(DriverState.READY, "startup sequence complete")
        logging.warning("Moore6mDriver: startup complete")

    # ------------------------------------------------------------------
    # Brakes
    # ------------------------------------------------------------------

    def brakes_on(self):
        state = self.get_state()
        if not state.az_brake:
            self.enqueue_blocking("ABN", expected_prefix="ABN")
        if not state.el_brake:
            self.enqueue_blocking("EBN", expected_prefix="EBN")

    def brakes_off(self):
        state = self.get_state()
        if state.az_brake:
            self.enqueue_blocking("ABF", expected_prefix="ABF")
        if state.el_brake:
            self.enqueue_blocking("EBF", expected_prefix="EBF")

    # ------------------------------------------------------------------
    # Pointing
    # ------------------------------------------------------------------

    def point(self, az: float, el: float):
        state = self.get_state()
        if not state.calibrated:
            logging.error("Moore6mDriver: cannot point — not calibrated")
            return
        if not (self.az_limits[0] <= az <= self.az_limits[1]):
            logging.error(
                "Moore6mDriver: az %.3f outside limits [%s, %s]",
                az, self.az_limits[0], self.az_limits[1],
            )
            return
        if el < self.el_limits[0]:
            logging.error("Moore6mDriver: el %.3f below minimum %s", el, self.el_limits[0])
            return
        if el > self.el_limits[1]:
            logging.error("Moore6mDriver: el %.3f above maximum %s", el, self.el_limits[1])
            return

        self.brakes_off()
        if state.loop_mode != "Track":
            self.enqueue_blocking("TON", expected_prefix="TON")
        self._transition(DriverState.SLEWING, f"point az={az:.3f} el={el:.3f}")
        with self._state_lock:
            self._state.az_cmd = az
            self._state.el_cmd = el
        return self.enqueue_blocking(f"AZL,{az},{el}", expected_prefix="AZL")

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def calibrate(self):
        if not self.lpr_params.is_loaded:
            self._set_fault("Cannot calibrate: LPR parameters not fully loaded")
            return

        lpr_cmd = self.lpr_params.to_command_string()
        seq = [
            ("SPA", "SPA"),
            (lpr_cmd, "LPR"),
            ("CLE", "CLE"),
        ]
        logging.warning("Moore6mDriver: starting calibration")
        self._transition(DriverState.CALIBRATING, "calibration started")

        for cmd, expected in seq:
            resp = self.enqueue_blocking(
                cmd, expected_prefix=expected,
                retries=MAX_COMMAND_RETRIES,
                send_delay=SEND_DELAY_STARTUP, recv_delay=RECV_DELAY_STARTUP,
            )
            if not resp or not resp.startswith(expected):
                logging.error("Moore6mDriver: calibration bad response to %s: %r", expected, resp)
                self._set_fault("calibration command failed")
                return

        # Poll until CalSts == Calibration OK (poll loop updates _state automatically)
        start     = monotonic()
        timeout_s = 180.0
        while True:
            state = self.get_state()
            if state.cal_sts == "Calibration OK":
                break
            if monotonic() - start > timeout_s:
                self._set_fault("calibration timed out")
                return
            sleep(0.5)

        logging.warning("Moore6mDriver: calibration complete")

        # Update lpr field in state now that calibration is confirmed
        with self._state_lock:
            self._state.lpr = self.lpr_params

        self.enqueue_blocking("TMD,0", expected_prefix="TMD")
        self.brakes_off()
        self.enqueue_blocking("TON", expected_prefix="TON")
        self._transition(DriverState.READY, "calibration complete")

    # ------------------------------------------------------------------
    # Stop-all / safe mode
    # ------------------------------------------------------------------

    def spa(self):
        """Send SPA directly (bypasses safe mode check — always permitted)."""
        raw = b"SPA\r"
        with self._serial_lock:
            self._record_serial_comm("sent", "SPA (immediate)")
            self.serial.write(raw)
            sleep(0.01)
        # Pessimistically invalidate calibration if it was in progress
        with self._state_lock:
            if self._state.cal_sts == "Calibrating Now":
                self._state.cal_sts = "Not Calibrated"
        if self._fsm_state not in (DriverState.FAULT, DriverState.SHUTDOWN):
            self._transition(DriverState.READY, "stop-all command")

    def set_safe_mode(self, enabled: bool):
        self._safe_mode = enabled
        with self._state_lock:
            self._state.safe_mode = enabled
        logging.warning(
            "Moore6mDriver: safe mode %s", "enabled" if enabled else "disabled"
        )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self):
        if self._fsm_state == DriverState.SHUTDOWN:
            return
        logging.warning("Moore6mDriver: shutting down")
        try:
            self.spa()
            self.brakes_on()
        except Exception:
            pass
        self._transition(DriverState.SHUTDOWN, "shutdown")
        self._stop_event.set()
        self._command_queue.put(None)   # unblock serial loop
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # History accessors
    # ------------------------------------------------------------------

    def get_recent_serial_communications(self, limit: int = 12):
        if limit <= 0:
            return []
        with self._comm_lock:
            return list(self.serial_communications)[-limit:]

    def get_recent_command_history(self, limit: int = 12):
        if limit <= 0:
            return []
        with self._history_lock:
            return list(self.command_history)[-limit:]