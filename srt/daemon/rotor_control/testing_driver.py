"""testing_driver.py

Simulated rotor driver for use when MOTOR_TYPE is NONE.
Accepts point commands and updates state instantly with no serial I/O.
"""

import copy
from threading import RLock
from time import monotonic

from ..types import LprParams, RotorState


class TestingDriver:
    """Drop-in replacement for Moore6mDriver with no hardware dependency.

    Starts pre-calibrated and pre-tracking so the daemon command loop
    can exercise pointing logic without any special-casing.
    """

    def __init__(
        self,
        az_limits=(-89, 449),
        el_limits=(15, 81),
        lpr_params: LprParams = None,
    ):
        self.az_limits  = az_limits
        self.el_limits  = el_limits
        self.lpr_params = lpr_params or LprParams()
        self._lock      = RLock()
        self._state     = RotorState(
            az=float(az_limits[0]),
            el=float(el_limits[0]),
            fsm_state="ready",
            cal_sts="Calibration OK",
            loop_mode="Track",
            az_brake=False,
            el_brake=False,
            lpr=self.lpr_params,
            last_poll_time=monotonic(),
        )

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_state(self) -> RotorState:
        with self._lock:
            return copy.copy(self._state)

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def point(self, az: float, el: float):
        with self._lock:
            if not self._state.calibrated:
                return
            self._state.az     = float(az)
            self._state.el     = float(el)
            self._state.az_cmd = float(az)
            self._state.el_cmd = float(el)
            self._state.az_err = 0.0
            self._state.el_err = 0.0
            self._state.last_poll_time = monotonic()

    def brakes_on(self):
        with self._lock:
            self._state.az_brake = True
            self._state.el_brake = True

    def brakes_off(self):
        with self._lock:
            self._state.az_brake = False
            self._state.el_brake = False

    # ------------------------------------------------------------------
    # Calibration / control
    # ------------------------------------------------------------------

    def startup(self):
        pass

    def calibrate(self):
        with self._lock:
            self._state.cal_sts   = "Calibration OK"
            self._state.loop_mode = "Track"
            self._state.lpr       = self.lpr_params

    def spa(self):
        with self._lock:
            if self._state.cal_sts == "Calibrating Now":
                self._state.cal_sts = "Not Calibrated"
            self._state.loop_mode = "Stop"
            self._state.fsm_state = "ready"

    def set_safe_mode(self, enabled: bool):
        with self._lock:
            self._state.safe_mode = bool(enabled)

    def shutdown(self):
        with self._lock:
            self._state.fsm_state = "shutdown"

    # ------------------------------------------------------------------
    # History (no-ops — nothing to record)
    # ------------------------------------------------------------------

    def get_recent_serial_communications(self, limit: int = 12):
        return []

    def get_recent_command_history(self, limit: int = 12):
        return []