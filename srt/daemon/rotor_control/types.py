"""telescope_state.py

Canonical dataclass definitions for all telescope state.

Three classes:
  LprParams     — the 37 servo loop parameters sent via the LPR command
  RotorState    — complete motor/servo snapshot (replaces rotor_diagnostics
                  + rotor_fsm_status dicts)
  DaemonStatus  — everything the daemon publishes (replaces the ad-hoc dict
                  in update_status)

No dependencies on any other part of the codebase — safe to import anywhere.

Serialisation
-------------
Every class has to_dict() / from_dict() so the existing ZMQ JSON publish
path keeps working.  DaemonStatus.to_dict() also emits legacy flat keys
(rotor_diagnostics, rotor_fsm_status, center_frequency, …) so dashboard
callbacks need not all change at once.
"""

from __future__ import annotations

import time as _time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# LPR parameters
# ---------------------------------------------------------------------------

# Ordered parameter names matching LPR command positions 1-37 (from SAW source)
_LPR_PARAM_ORDER: Tuple[str, ...] = (
    "pAzKo",   "pElKo",
    "pAzKv",   "pElKv",
    "pAze",    "pEle",
    "pAzAmax", "pElAmax",
    "pAzVmax", "pElVmax",
    "pAzImax", "pElImax",
    "pAzKpp",  "pElKpp",
    "pAzKpi",  "pElKpi",
    "pAzKvp",  "pElKvp",
    "pAzKvi",  "pElKvi",
    "pAzKff",  "pElKff",
    "pAzTmax", "pElTmax",
    "pAzTmin", "pElTmin",
    "pAzTsgn", "pElTsgn",
    "pAzTbias",
    "pAzEcr",  "pElEcr",
    "pAzMEcr", "pElMEcr",
    "pAzEpo",  "pElEpo",
    "pAzAr",   "pElAr",
)


@dataclass
class LprParams:
    """Servo controller loop parameters loaded by the LPR command.

    All 37 values default to None, meaning "not yet loaded from config".
    The dashboard antenna page shows — for None values.

    Values are set from config at startup via LprParams.from_dict(config_dict["MOTOR_LPR_PARAMS"]).
    The to_command_string() method renders the full LPR,v1,v2,... string
    so moore6m_serial.py never needs to hardcode the values.
    """

    # Outer position loop gains
    pAzKo:    Optional[float] = None
    pElKo:    Optional[float] = None

    # Velocity observer gains
    pAzKv:    Optional[float] = None
    pElKv:    Optional[float] = None

    # Position error deadbands (antenna deg)
    pAze:     Optional[float] = None
    pEle:     Optional[float] = None

    # Acceleration limits (antenna deg/s²)
    pAzAmax:  Optional[float] = None
    pElAmax:  Optional[float] = None

    # Velocity ceilings (antenna deg/s) — raise to allow faster slews
    pAzVmax:  Optional[float] = None
    pElVmax:  Optional[float] = None

    # Position integrator anti-windup clamps
    pAzImax:  Optional[float] = None
    pElImax:  Optional[float] = None

    # Position loop P and I gains
    pAzKpp:   Optional[float] = None
    pElKpp:   Optional[float] = None
    pAzKpi:   Optional[float] = None
    pElKpi:   Optional[float] = None

    # Velocity loop P and I gains — reduce pAzKvp to fix azimuth overshoot
    pAzKvp:   Optional[float] = None
    pElKvp:   Optional[float] = None
    pAzKvi:   Optional[float] = None
    pElKvi:   Optional[float] = None

    # Feedforward gains
    pAzKff:   Optional[float] = None
    pElKff:   Optional[float] = None

    # Torque limits (DAC counts, ±2047 max)
    pAzTmax:  Optional[float] = None
    pElTmax:  Optional[float] = None
    pAzTmin:  Optional[float] = None
    pElTmin:  Optional[float] = None

    # Torque sign correction and Y/Z bias
    pAzTsgn:  Optional[float] = None
    pElTsgn:  Optional[float] = None
    pAzTbias: Optional[float] = None

    # Encoder counts per revolution (antenna axis)
    pAzEcr:   Optional[float] = None
    pElEcr:   Optional[float] = None

    # Encoder counts per revolution (motor axis)
    pAzMEcr:  Optional[float] = None
    pElMEcr:  Optional[float] = None

    # Encoder phase offsets (deg) — critical for calibration accuracy
    pAzEpo:   Optional[float] = None
    pElEpo:   Optional[float] = None

    # Axis ratios (motor deg/s per antenna deg/s)
    pAzAr:    Optional[float] = None
    pElAr:    Optional[float] = None

    @property
    def is_loaded(self) -> bool:
        """True if all parameters have been set from config."""
        return all(getattr(self, p) is not None for p in _LPR_PARAM_ORDER)

    def to_command_string(self) -> str:
        """Render as the full LPR,v1,v2,... command string.

        Raises ValueError if any parameter is still None.
        """
        missing = [p for p in _LPR_PARAM_ORDER if getattr(self, p) is None]
        if missing:
            raise ValueError(
                f"Cannot build LPR command — parameters not set: {missing}"
            )
        vals = ",".join(str(getattr(self, p)) for p in _LPR_PARAM_ORDER)
        return f"LPR,{vals}"

    def to_list(self) -> List[Optional[float]]:
        """Values in LPR command order (for antenna_page param table)."""
        return [getattr(self, p) for p in _LPR_PARAM_ORDER]

    def to_dict(self) -> Dict[str, Optional[float]]:
        return {p: getattr(self, p) for p in _LPR_PARAM_ORDER}

    @classmethod
    def from_command_string(cls, lpr_str: str) -> "LprParams":
        """Parse 'LPR,v1,v2,...' back into an LprParams instance."""
        parts = lpr_str.strip().split(",")
        values = parts[1:] if parts[0].upper() == "LPR" else parts
        inst = cls()
        for name, raw in zip(_LPR_PARAM_ORDER, values):
            try:
                setattr(inst, name, float(raw))
            except (ValueError, TypeError):
                pass
        return inst

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "LprParams":
        """Build from a dict, e.g. config_dict['MOTOR_LPR_PARAMS']."""
        if not d:
            return cls()
        inst = cls()
        for name in _LPR_PARAM_ORDER:
            if name in d and d[name] is not None:
                try:
                    setattr(inst, name, float(d[name]))
                except (ValueError, TypeError):
                    pass
        return inst

    @classmethod
    def param_order(cls) -> Tuple[str, ...]:
        """The ordered list of parameter names for table display."""
        return _LPR_PARAM_ORDER


# ---------------------------------------------------------------------------
# Rotor state
# ---------------------------------------------------------------------------

@dataclass
class RotorState:
    """Complete snapshot of motor/servo controller state.

    Single definition shared by:
      - moore6m_serial  (writes via get_rotor_state())
      - rotors.py       (get_diagnostics returns one of these)
      - daemon          (reads for status publish)
      - Moore6mController GUI (reads directly)
      - dashboard antenna panel (reads via DaemonStatus)
      - Moore6mClient._apply_status (writes from ZMQ response)
    """

    # ---- Position ----
    az:     float = 0.0
    el:     float = 0.0
    az_err: float = 0.0
    el_err: float = 0.0
    az_cmd: float = 0.0
    el_cmd: float = 0.0

    # ---- FSM ----
    fsm_state:       str  = "disconnected"
    last_transition: str  = ""
    last_error:      str  = ""
    retry_count:     int  = 0
    safe_mode:       bool = False

    # ---- Calibration / tracking loop ----
    # cal_sts:   mirrors CalComplete  0=Not Calibrated 1=Calibrating Now 2=Calibration OK
    # loop_mode: mirrors RunningTask  0=Stop 1=Track
    cal_sts:   str = "Not Calibrated"
    loop_mode: str = "Stop"

    # ---- Brakes ----
    az_brake: bool = True
    el_brake: bool = True

    # ---- Safety flags ----
    estop:    bool = False
    sim_mode: bool = False

    # ---- Limit switches ----
    el_up_pre:  bool = False
    el_dn_pre:  bool = False
    el_up_fin:  bool = False
    el_dn_fin:  bool = False
    az_cw_pre:  bool = False
    az_ccw_pre: bool = False
    az_cw_fin:  bool = False
    az_ccw_fin: bool = False
    az_lt_180:  bool = False

    # ---- Amplifier currents ----
    # {"2A01": {"commanded": int|None, "actual": int|None}, ...}
    amp_currents: Dict = field(default_factory=lambda: {
        "2A01": {"commanded": None, "actual": None},
        "2A02": {"commanded": None, "actual": None},
        "2A03": {"commanded": None, "actual": None},
    })

    # ---- LPR parameters ----
    lpr: LprParams = field(default_factory=LprParams)

    # ---- Timestamps ----
    last_poll_time:    float = 0.0
    last_command_time: float = 0.0

    # ------------------------------------------------------------------
    # Derived properties
    # ------------------------------------------------------------------

    @property
    def calibrated(self) -> bool:
        return self.cal_sts == "Calibration OK"

    @property
    def any_limit_active(self) -> bool:
        return any([
            self.el_up_pre, self.el_dn_pre,
            self.el_up_fin, self.el_dn_fin,
            self.az_cw_pre, self.az_ccw_pre,
            self.az_cw_fin, self.az_ccw_fin,
        ])

    @property
    def any_final_limit_active(self) -> bool:
        return any([
            self.el_up_fin, self.el_dn_fin,
            self.az_cw_fin, self.az_ccw_fin,
        ])

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        d = asdict(self)
        # Replace nested lpr dataclass dict with our ordered form
        d["lpr"] = self.lpr.to_dict()
        # Add flat ordered list for the antenna page param table
        d["lpr_list"] = self.lpr.to_list()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "RotorState":
        if not d:
            return cls()
        lpr_raw = d.pop("lpr", None)
        d.pop("lpr_list", None)
        valid = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        inst = cls(**valid)
        if isinstance(lpr_raw, dict):
            inst.lpr = LprParams.from_dict(lpr_raw)
        return inst


# ---------------------------------------------------------------------------
# Radio state
# ---------------------------------------------------------------------------

@dataclass
class RadioState:
    """Radio / SDR subsystem state."""
    center_frequency:     float = 0.0
    bandwidth:            float = 0.0
    frequency_correction: float = 0.0
    temp_sys:             float = 0.0
    temp_cal:             float = 0.0
    cal_power:            float = 1.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "RadioState":
        if not d:
            return cls()
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


# ---------------------------------------------------------------------------
# Daemon status publish
# ---------------------------------------------------------------------------

@dataclass
class DaemonStatus:
    """Complete snapshot published by the daemon on each status tick.

    Replaces the ad-hoc dict assembled in update_status().

    to_dict() emits both the new nested structure AND legacy flat keys so
    existing dashboard callbacks keep working without change.
    from_dict() handles both old and new format transparently.
    """

    # ---- Rotor (replaces rotor_diagnostics + rotor_fsm_status) ----
    rotor: RotorState = field(default_factory=RotorState)

    # ---- Radio ----
    radio: RadioState = field(default_factory=RadioState)

    # ---- Antenna geometry ----
    beam_width:     float         = 0.0
    az_limits:      Tuple         = (0.0, 360.0)
    el_limits:      Tuple         = (0.0, 90.0)
    stow_loc:       Tuple         = (180.0, 81.0)
    cal_loc:        Tuple         = (0.0, 0.0)
    horizon_points: List          = field(default_factory=list)

    # ---- Location ----
    location: Dict = field(default_factory=lambda: {
        "latitude": 0.0, "longitude": 0.0, "elevation": 0.0,
    })

    # ---- Ephemeris ----
    object_locs:      Dict  = field(default_factory=dict)
    object_time_locs: Dict  = field(default_factory=dict)
    vlsr:             float = 0.0

    # ---- Pointing ----
    motor_offsets:          Tuple         = (0.0, 0.0)
    pointing_error_history: List          = field(default_factory=list)
    amp_current_history:    List          = field(default_factory=list)

    # ---- Command queue ----
    queued_item: str = "None"
    queue_size:  int = 0

    # ---- Logs and events ----
    error_logs:            List = field(default_factory=list)
    observation_events:    List = field(default_factory=list)
    serial_communications: List = field(default_factory=list)
    command_history:       List = field(default_factory=list)

    # ---- Observation data ----
    n_point_data:     List = field(default_factory=list)
    beam_switch_data: List = field(default_factory=list)
    cal_values:       List = field(default_factory=list)

    # ---- System ----
    emergency_contact: Dict  = field(default_factory=dict)
    time:              float = field(default_factory=_time.time)

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------

    @property
    def az(self) -> float:
        return self.rotor.az

    @property
    def el(self) -> float:
        return self.rotor.el

    @property
    def calibrated(self) -> bool:
        return self.rotor.calibrated

    @property
    def lpr(self) -> LprParams:
        return self.rotor.lpr

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        d = asdict(self)
        d["rotor"] = self.rotor.to_dict()
        d["radio"] = self.radio.to_dict()

        # Legacy flat keys — keeps existing dashboard callbacks working
        d["rotor_diagnostics"] = self.rotor.to_dict()
        d["rotor_fsm_status"] = {
            "state":           self.rotor.fsm_state,
            "last_transition": self.rotor.last_transition,
            "last_error":      self.rotor.last_error,
            "retry_count":     self.rotor.retry_count,
            "safe_mode":       self.rotor.safe_mode,
        }
        d["center_frequency"]     = self.radio.center_frequency
        d["bandwidth"]            = self.radio.bandwidth
        d["frequency_correction"] = self.radio.frequency_correction
        d["temp_sys"]             = self.radio.temp_sys
        d["temp_cal"]             = self.radio.temp_cal
        d["cal_power"]            = self.radio.cal_power
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "DaemonStatus":
        """Reconstruct from JSON-deserialised dict. Handles both new and
        legacy formats (separate rotor_diagnostics / rotor_fsm_status)."""
        if not d:
            return cls()

        # Rotor: prefer new nested key, fall back to legacy
        rotor_raw = d.get("rotor") or d.get("rotor_diagnostics")
        if isinstance(rotor_raw, dict):
            # Merge legacy fsm fields if present
            fsm = d.get("rotor_fsm_status") or {}
            rotor_raw.setdefault("fsm_state",       fsm.get("state", "unknown"))
            rotor_raw.setdefault("last_transition",  fsm.get("last_transition", ""))
            rotor_raw.setdefault("last_error",       fsm.get("last_error", ""))
            rotor_raw.setdefault("retry_count",      fsm.get("retry_count", 0))
            rotor_raw.setdefault("safe_mode",        fsm.get("safe_mode", False))

        # Radio: prefer nested, fall back to legacy flat keys
        radio_raw = d.get("radio") or {
            "center_frequency":     d.get("center_frequency", 0.0),
            "bandwidth":            d.get("bandwidth", 0.0),
            "frequency_correction": d.get("frequency_correction", 0.0),
            "temp_sys":             d.get("temp_sys", 0.0),
            "temp_cal":             d.get("temp_cal", 0.0),
            "cal_power":            d.get("cal_power", 1.0),
        }

        skip = {
            "rotor", "radio", "rotor_diagnostics", "rotor_fsm_status",
            "center_frequency", "bandwidth", "frequency_correction",
            "temp_sys", "temp_cal", "cal_power",
        }
        kwargs = {
            k: v for k, v in d.items()
            if k in cls.__dataclass_fields__ and k not in skip
        }
        inst = cls(**kwargs)
        inst.rotor = RotorState.from_dict(dict(rotor_raw)) if rotor_raw else RotorState()
        inst.radio = RadioState.from_dict(radio_raw)
        return inst