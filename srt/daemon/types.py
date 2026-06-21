"""telescope_state.py

Canonical dataclass definitions for all telescope state.

Key classes:
    LprParams       — the 37 servo loop parameters sent via the LPR command
    RotorState      — complete motor/servo snapshot (replaces rotor_diagnostics
                      + rotor_fsm_status dicts)
    SpectrumConfig  — user-configurable spectrum analyzer settings
    SpectrumFrame   — one acquired spectrum snapshot
    DaemonStatus    — everything the daemon publishes (replaces the ad-hoc dict
                      in update_status)

No dependencies on any other part of the codebase — safe to import anywhere.

Serialisation
-------------
Every class has to_dict() / from_dict() so the existing ZMQ JSON publish
path keeps working.  DaemonStatus.to_dict() also emits legacy rotor
keys (rotor_diagnostics, rotor_fsm_status) so dashboard callbacks need
not all change at once.
"""

from __future__ import annotations

import time as _time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np


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
    amp_currents: Dict[str, Dict[str, Optional[int]]] = field(default_factory=lambda: {
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
# Spectrum analyzer types
# ---------------------------------------------------------------------------

def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return False


def _coerce_optional_bool(value: Any) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, str) and value.strip().lower() in {"none", "null", ""}:
        return None
    return _coerce_bool(value)


def _coerce_tuple(value: Any, default: Tuple[float, float]) -> Tuple[float, float]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return (float(value[0]), float(value[1]))
    if isinstance(value, str) and "," in value:
        parts = [p.strip() for p in value.split(",") if p.strip()]
        if len(parts) == 2:
            return (float(parts[0]), float(parts[1]))
    return default


@dataclass
class SpectrumConfig:
    """All user-configurable settings. Live-updatable."""

    instrument_serial: str = "SSA3PCED7R1040"
    freq_mode: str = "start_stop"
    start_hz: float = 1.0e9
    stop_hz: float = 1.9e9
    center_hz: float = 1.4205e9
    span_hz: float = 200.0e6
    rbw_hz: float = 1.0e6
    vbw_hz: float = 100.0e3
    ref_level_dbm: float = -30.0
    y_axis_mode: str = "db_per_div"
    y_db_per_div: float = 10.0
    y_num_divs: int = 10
    y_lim_dbm: Tuple[float, float] = (-120.0, -30.0)
    atten_auto: bool = True
    atten_db: float = 0.0
    preamp_on: Optional[bool] = True
    trace_type: str = "clear_write"
    num_averages: int = 300
    x_units: str = "MHz"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instrument_serial": self.instrument_serial,
            "freq_mode": self.freq_mode,
            "start_hz": float(self.start_hz),
            "stop_hz": float(self.stop_hz),
            "center_hz": float(self.center_hz),
            "span_hz": float(self.span_hz),
            "rbw_hz": float(self.rbw_hz),
            "vbw_hz": float(self.vbw_hz),
            "ref_level_dbm": float(self.ref_level_dbm),
            "y_axis_mode": self.y_axis_mode,
            "y_db_per_div": float(self.y_db_per_div),
            "y_num_divs": int(self.y_num_divs),
            "y_lim_dbm": (float(self.y_lim_dbm[0]), float(self.y_lim_dbm[1])),
            "atten_auto": bool(self.atten_auto),
            "atten_db": float(self.atten_db),
            "preamp_on": self.preamp_on,
            "trace_type": self.trace_type,
            "num_averages": int(self.num_averages),
            "x_units": self.x_units,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SpectrumConfig":
        if not d:
            return cls()
        base = cls()
        return cls(
            instrument_serial=str(d.get("instrument_serial", base.instrument_serial)),
            freq_mode=str(d.get("freq_mode", base.freq_mode)),
            start_hz=float(d.get("start_hz", base.start_hz)),
            stop_hz=float(d.get("stop_hz", base.stop_hz)),
            center_hz=float(d.get("center_hz", base.center_hz)),
            span_hz=float(d.get("span_hz", base.span_hz)),
            rbw_hz=float(d.get("rbw_hz", base.rbw_hz)),
            vbw_hz=float(d.get("vbw_hz", base.vbw_hz)),
            ref_level_dbm=float(d.get("ref_level_dbm", base.ref_level_dbm)),
            y_axis_mode=str(d.get("y_axis_mode", base.y_axis_mode)),
            y_db_per_div=float(d.get("y_db_per_div", base.y_db_per_div)),
            y_num_divs=int(d.get("y_num_divs", base.y_num_divs)),
            y_lim_dbm=_coerce_tuple(d.get("y_lim_dbm"), base.y_lim_dbm),
            atten_auto=_coerce_bool(d.get("atten_auto", base.atten_auto)),
            atten_db=float(d.get("atten_db", base.atten_db)),
            preamp_on=_coerce_optional_bool(d.get("preamp_on", base.preamp_on)),
            trace_type=str(d.get("trace_type", base.trace_type)),
            num_averages=int(d.get("num_averages", base.num_averages)),
            x_units=str(d.get("x_units", base.x_units)),
        )


@dataclass
class SpectrumFrame:
    """One acquired and processed spectrum snapshot."""

    freq_hz: list = field(default_factory=list)
    power_dbm: list = field(default_factory=list)
    raw_dbm: list = field(default_factory=list)
    sweep_index: int = 0
    timestamp: float = 0.0
    config: SpectrumConfig = field(default_factory=SpectrumConfig)
    avg_count: int = 1
    connected: bool = False

    def to_dict(self) -> Dict[str, Any]:
        def _to_list(value: Any) -> List[float]:
            if hasattr(value, "tolist"):
                return value.tolist()
            return list(value)

        return {
            "freq_hz": _to_list(self.freq_hz),
            "power_dbm": _to_list(self.power_dbm),
            "raw_dbm": _to_list(self.raw_dbm),
            "sweep_index": int(self.sweep_index),
            "timestamp": float(self.timestamp),
            "avg_count": int(self.avg_count),
            "config": self.config.to_dict(),
            "connected": bool(self.connected),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "SpectrumFrame":
        if not d:
            raise ValueError("SpectrumFrame.from_dict requires a dict")
        return cls(
            freq_hz=d.get("freq_hz", []),
            power_dbm=d.get("power_dbm", []),
            raw_dbm=d.get("raw_dbm", []),
            sweep_index=int(d.get("sweep_index", 0)),
            timestamp=float(d.get("timestamp", 0.0)),
            config=SpectrumConfig.from_dict(d.get("config") or {}),
            avg_count=int(d.get("avg_count", 1)),
            connected=bool(d.get("connected", False)),
        )

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

    # ---- Spectrum ----
    spectrum: Optional[SpectrumFrame] = field(default_factory=SpectrumFrame)

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
    vlsr:             Dict = field(default_factory=dict)

    # ---- Pointing ----
    motor_offsets:          Tuple         = (0.0, 0.0)
    pointing_error_history: List          = field(default_factory=list)
    amp_current_history:    List[dict[str, Any]]          = field(default_factory=list)

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

        # Legacy flat keys — keeps existing dashboard callbacks working
        d["rotor_diagnostics"] = self.rotor.to_dict()
        d["rotor_fsm_status"] = {
            "state":           self.rotor.fsm_state,
            "last_transition": self.rotor.last_transition,
            "last_error":      self.rotor.last_error,
            "retry_count":     self.rotor.retry_count,
            "safe_mode":       self.rotor.safe_mode,
        }
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

        skip = {
            "rotor", "rotor_diagnostics", "rotor_fsm_status",
            "radio", "center_frequency", "bandwidth", "frequency_correction",
            "temp_sys", "temp_cal", "cal_power",
        }
        kwargs = {
            k: v for k, v in d.items()
            if k in cls.__dataclass_fields__ and k not in skip
        }
        inst = cls(**kwargs)
        inst.rotor = RotorState.from_dict(dict(rotor_raw)) if rotor_raw else RotorState()
        return inst