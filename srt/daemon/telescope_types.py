"""
telescope_state.py — Pydantic models for all telescope state.

Strict conversion from the original dataclass version: no legacy
flat-key emission, no lenient string coercion (e.g. "yes"/"on" -> bool).
Bad input raises a ValidationError rather than being silently dropped
or defaulted. This is intentional per the decision to prioritize
correctness over backward compatibility with old ZMQ payload shapes.
"""

from __future__ import annotations

import time as _time
from enum import Enum
from typing import Optional, Literal

from pydantic import BaseModel, Field, model_validator

# ---------------------------------------------------------------------------
# LPR parameters
# ---------------------------------------------------------------------------

# Ordered parameter names matching LPR command positions 1-37 (from SAW source)
_LPR_PARAM_ORDER: tuple[str, ...] = (
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


class LprParams(BaseModel):
    """Servo controller loop parameters loaded by the LPR command.

    All 37 values default to None, meaning "not yet loaded from config".
    The dashboard antenna page shows — for None values.

    Values are set from config at startup via
    LprParams.model_validate(config_dict["MOTOR_LPR_PARAMS"]).
    to_command_string() renders the full LPR,v1,v2,... string so
    moore6m_serial.py never needs to hardcode the values.

    VALIDATION POLICY (applies to all models in this file): strict
    validation happens at construction and at deserialization
    (model_validate / model_validate_json) — NOT on every attribute
    assignment. The poll loop in moore6m_driver.py mutates a working
    RotorState copy via repeated setattr() many times per second;
    re-validating on every one of those assignments would be both
    slow and the wrong place to catch bad data. Validate at the
    boundary (when a fresh model is built from parsed/incoming data),
    trust it internally afterward.
    """

    # Outer position loop gains
    pAzKo: Optional[float] = None
    pElKo: Optional[float] = None

    # Velocity observer gains
    pAzKv: Optional[float] = None
    pElKv: Optional[float] = None

    # Position error deadbands (antenna deg)
    pAze: Optional[float] = None
    pEle: Optional[float] = None

    # Acceleration limits (antenna deg/s²)
    pAzAmax: Optional[float] = None
    pElAmax: Optional[float] = None

    # Velocity ceilings (antenna deg/s) — raise to allow faster slews
    pAzVmax: Optional[float] = None
    pElVmax: Optional[float] = None

    # Position integrator anti-windup clamps
    pAzImax: Optional[float] = None
    pElImax: Optional[float] = None

    # Position loop P and I gains
    pAzKpp: Optional[float] = None
    pElKpp: Optional[float] = None
    pAzKpi: Optional[float] = None
    pElKpi: Optional[float] = None

    # Velocity loop P and I gains — reduce pAzKvp to fix azimuth overshoot
    pAzKvp: Optional[float] = None
    pElKvp: Optional[float] = None
    pAzKvi: Optional[float] = None
    pElKvi: Optional[float] = None

    # Feedforward gains
    pAzKff: Optional[float] = None
    pElKff: Optional[float] = None

    # Torque limits (DAC counts, ±2047 max)
    pAzTmax: Optional[float] = None
    pElTmax: Optional[float] = None
    pAzTmin: Optional[float] = None
    pElTmin: Optional[float] = None

    # Torque sign correction and Y/Z bias
    pAzTsgn: Optional[float] = None
    pElTsgn: Optional[float] = None
    pAzTbias: Optional[float] = None

    # Encoder counts per revolution (antenna axis)
    pAzEcr: Optional[float] = None
    pElEcr: Optional[float] = None

    # Encoder counts per revolution (motor axis)
    pAzMEcr: Optional[float] = None
    pElMEcr: Optional[float] = None

    # Encoder phase offsets (deg) — critical for calibration accuracy
    pAzEpo: Optional[float] = None
    pElEpo: Optional[float] = None

    # Axis ratios (motor deg/s per antenna deg/s)
    pAzAr: Optional[float] = None
    pElAr: Optional[float] = None

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

    def to_list(self) -> list[Optional[float]]:
        """Values in LPR command order (for antenna_page param table)."""
        return [getattr(self, p) for p in _LPR_PARAM_ORDER]

    @classmethod
    def from_command_string(cls, lpr_str: str) -> "LprParams":
        """Parse 'LPR,v1,v2,...' back into an LprParams instance.

        Strict: raises ValueError if a value can't be parsed as a float,
        rather than silently leaving that parameter as None.
        """
        parts = lpr_str.strip().split(",")
        values = parts[1:] if parts[0].upper() == "LPR" else parts
        kwargs = {}
        for name, raw in zip(_LPR_PARAM_ORDER, values):
            kwargs[name] = float(raw)  # raises ValueError on bad input
        return cls(**kwargs)

    @classmethod
    def param_order(cls) -> tuple[str, ...]:
        """The ordered list of parameter names for table display."""
        return _LPR_PARAM_ORDER


# ---------------------------------------------------------------------------
# Rotor state
# ---------------------------------------------------------------------------

class AmpCurrent(BaseModel):
    """Commanded vs. actual current reading for one amplifier channel."""

    commanded: Optional[int] = None
    actual: Optional[int] = None


class DriverState(Enum):
    """Moore6mDriver's internal FSM states.

    Moved here from moore6m_driver.py so RotorState.fsm_state can use
    it directly as the field type, rather than maintaining a separate
    hand-written Literal[...] list that has to be kept in sync by hand
    with this enum. moore6m_driver.py now imports DriverState from here
    instead of defining it.
    """

    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    STARTUP_SYNC = "startup_sync"
    READY = "ready"
    SLEWING = "slewing"
    TRACKING = "tracking"
    CALIBRATING = "calibrating"
    FAULT = "fault"
    RECOVERING = "recovering"
    SHUTDOWN = "shutdown"


class RotorState(BaseModel):
    """Complete snapshot of motor/servo controller state.

    Single definition shared by:
      - moore6m_serial  (writes via get_rotor_state())
      - rotors.py       (get_diagnostics returns one of these)
      - daemon          (reads for status publish)
      - Moore6mController GUI (reads directly)
      - dashboard antenna panel (reads via DaemonStatus)
    """

    # ---- Position ----
    az: float = 0.0
    el: float = 0.0
    az_err: float = 0.0
    el_err: float = 0.0
    az_cmd: float = 0.0
    el_cmd: float = 0.0

    # ---- FSM ----
    # DriverState handles both directions: accepts the .value string
    # (e.g. "tracking") on construction/deserialization and serializes
    # back to that same string via model_dump_json(). No separate
    # Literal[...] list to keep in sync — DriverState is the only
    # definition of these strings now.
    fsm_state: DriverState = DriverState.DISCONNECTED
    last_transition: str = ""
    last_error: str = ""
    retry_count: int = 0
    safe_mode: bool = False

    # ---- Calibration / tracking loop ----
    # cal_sts:   set by _parse_sts from CalSts: 0=Not Calibrated 1=Calibrating Now 2=Calibration OK
    # loop_mode: set by _parse_sts from mode:   0=Stop 1=Track
    # Confirmed exact strings from moore6m_driver.py's _parse_sts mapping.
    cal_sts: Literal["Not Calibrated", "Calibrating Now", "Calibration OK"] = "Not Calibrated"
    loop_mode: Literal["Stop", "Track"] = "Stop"

    # ---- Brakes ----
    az_brake: bool = True
    el_brake: bool = True

    # ---- Safety flags ----
    estop: bool = False
    sim_mode: bool = False

    # ---- Limit switches ----
    el_up_pre: bool = False
    el_dn_pre: bool = False
    el_up_fin: bool = False
    el_dn_fin: bool = False
    az_cw_pre: bool = False
    az_ccw_pre: bool = False
    az_cw_fin: bool = False
    az_ccw_fin: bool = False
    az_lt_180: bool = False

    # ---- Amplifier currents ----
    amp_currents: dict[str, AmpCurrent] = Field(
        default_factory=lambda: {
            "2A01": AmpCurrent(),
            "2A02": AmpCurrent(),
            "2A03": AmpCurrent(),
        }
    )

    # ---- LPR parameters ----
    lpr: LprParams = Field(default_factory=LprParams)

    # ---- Timestamps ----
    last_poll_time: float = 0.0
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


# ---------------------------------------------------------------------------
# Spectrum analyzer types
# ---------------------------------------------------------------------------

class SpectrumConfig(BaseModel):
    """All user-configurable settings. Live-updatable."""

    instrument_serial: str = "SSA3PCED7R1040"
    freq_mode: Literal["start_stop", "center_span"] = "start_stop"
    start_hz: float = 1.0e9
    stop_hz: float = 1.9e9
    center_hz: float = 1.4205e9
    span_hz: float = 200.0e6
    rbw_hz: float = 1.0e6
    vbw_hz: float = 100.0e3
    ref_level_dbm: float = -30.0
    y_axis_mode: Literal["auto", "fixed_limits", "db_per_div"] = "db_per_div"
    y_db_per_div: float = 10.0
    y_num_divs: int = 10
    y_lim_dbm: tuple[float, float] = (-120.0, -30.0)
    atten_auto: bool = True
    atten_db: float = 0.0
    preamp_on: Optional[bool] = True
    trace_type: Literal["clear_write", "average"] = "clear_write"
    num_averages: int = 300
    x_units: Literal["Hz", "kHz", "MHz", "GHz"] = "MHz"

    @model_validator(mode="after")
    def _check_freq_range(self) -> "SpectrumConfig":
        if self.freq_mode == "start_stop" and self.start_hz >= self.stop_hz:
            raise ValueError("start_hz must be less than stop_hz")
        return self


class SpectrumFrame(BaseModel):
    """One acquired and processed spectrum snapshot."""

    freq_hz: list[float] = Field(default_factory=list)
    power_dbm: list[float] = Field(default_factory=list)
    raw_dbm: list[float] = Field(default_factory=list)
    sweep_index: int = 0
    timestamp: float = 0.0
    config: SpectrumConfig = Field(default_factory=SpectrumConfig)
    avg_count: int = 1
    connected: bool = False

    @model_validator(mode="after")
    def _check_array_lengths_match(self) -> "SpectrumFrame":
        lengths = {len(self.freq_hz), len(self.power_dbm)}
        if self.raw_dbm:
            lengths.add(len(self.raw_dbm))
        if len(lengths) > 1:
            raise ValueError(
                f"freq_hz/power_dbm/raw_dbm length mismatch: "
                f"freq_hz={len(self.freq_hz)}, power_dbm={len(self.power_dbm)}, "
                f"raw_dbm={len(self.raw_dbm)}"
            )
        return self


# ---------------------------------------------------------------------------
# Daemon status publish
# ---------------------------------------------------------------------------

class EmergencyContact(BaseModel):
    name: str
    email: str
    phone_number: str


class Location(BaseModel):
    latitude: float = 0.0
    longitude: float = 0.0
    elevation: float = 0.0
    name: Optional[str] = None


class SerialCommunication(BaseModel):
    """One logged serial line. Shape confirmed from
    moore6m_driver.py's _record_serial_comm."""

    time: str
    direction: Literal["sent", "recv"]
    payload: str


class CommandHistoryEntry(BaseModel):
    """One completed command's timing/result record. Shape confirmed
    from moore6m_driver.py's _record_command_history."""

    time: str
    command: str
    expected: Optional[str] = None
    response: str
    retries: int
    success: bool
    error: str
    queue_wait_ms: float
    duration_ms: float
    total_ms: float


class ObservationEvent(BaseModel):
    """One observation lifecycle event. Shape confirmed from
    daemon.py's _record_observation_event. `metadata` is left as a
    loose dict since its contents vary by event type (sequence,
    object, target_azel, point_index, point_total, end_reason, etc.
    — see get_observation_events_csv for the full set of keys ever
    read back out of it)."""

    time: str
    event: str
    metadata: dict = Field(default_factory=dict)


class DaemonStatus(BaseModel):
    """Complete snapshot published by the daemon on each status tick.

    Strict conversion: no legacy flat-key emission (rotor_diagnostics /
    rotor_fsm_status), no dual-format from_dict. Use model_dump_json()
    to serialize and model_validate_json() to parse — both ends of the
    ZMQ/WebSocket boundary should be updated together.
    """

    # ---- Rotor ----
    rotor: RotorState = Field(default_factory=RotorState)

    # ---- Spectrum ----
    spectrum: Optional[SpectrumFrame] = Field(default_factory=SpectrumFrame)

    # ---- Antenna geometry ----
    beam_width: float = 0.0
    az_limits: tuple[float, float] = (0.0, 360.0)
    el_limits: tuple[float, float] = (0.0, 90.0)
    stow_loc: tuple[float, float] = (180.0, 81.0)
    cal_loc: tuple[float, float] = (0.0, 0.0)
    horizon_points: list[tuple[float, float]] = Field(default_factory=list)

    # ---- Location ----
    location: Location = Field(default_factory=Location)

    # ---- Ephemeris ----
    object_locs: dict[str, tuple[float, float]] = Field(default_factory=dict)
    object_time_locs: dict[str, list[tuple[float, float]]] = Field(default_factory=dict)
    vlsr: dict[str, float] = Field(default_factory=dict)

    # ---- Pointing ----
    motor_offsets: tuple[float, float] = (0.0, 0.0)
    pointing_error_history: list[tuple[float, float]] = Field(default_factory=list)
    amp_current_history: list[dict[str, AmpCurrent]] = Field(default_factory=list)

    # ---- Command queue ----
    queued_item: str = "None"
    queue_size: int = 0

    # ---- Logs and events ----
    # log_message() appends (iso_timestamp, message) tuples, not bare
    # strings — confirmed from daemon.py's actual self.command_error_logs
    # usage. The list[str] guess was wrong; fixed after seeing log_message.
    error_logs: list[tuple[str, str]] = Field(default_factory=list)
    observation_events: list[ObservationEvent] = Field(default_factory=list)
    serial_communications: list[SerialCommunication] = Field(default_factory=list)
    command_history: list[CommandHistoryEntry] = Field(default_factory=list)

    # ---- Observation data ----
    n_point_data: list = Field(default_factory=list)
    beam_switch_data: list = Field(default_factory=list)
    cal_values: list[float] = Field(default_factory=list)

    # ---- System ----
    emergency_contact: Optional[EmergencyContact] = None
    time: float = Field(default_factory=_time.time)

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


# ---------------------------------------------------------------------------
# Daemon configuration (config.yaml) — replaces schema.yaml/yamale +
# config_loader.py's load_yaml() returning an untyped dict.
#
# Field types/required-ness below are transcribed directly from the real schema.yaml
# ---------------------------------------------------------------------------

class Limit(BaseModel):
    """AZLIMITS / ELLIMITS — schema.yaml's `limit` include."""

    lower_bound: float
    upper_bound: float

    def to_tuple(self) -> tuple[float, float]:
        return (self.lower_bound, self.upper_bound)


class AzElPoint(BaseModel):
    """STOW_LOCATION / CAL_LOCATION / HORIZON_POINTS entries —
    schema.yaml's `az_el_point` include."""

    azimuth: float
    elevation: float

    def to_tuple(self) -> tuple[float, float]:
        return (self.azimuth, self.elevation)


class DaemonConfig(BaseModel):
    """Top-level config.yaml structure. Load with:

        DaemonConfig.model_validate(config_loader.load_yaml(path))

    This replaces yamale's role (schema.yaml + validate_yaml_schema())
    as well as the untyped dict that load_yaml() previously handed to
    SmallRadioTelescopeDaemon.__init__ — every config_dict["KEY"] access
    in __init__ becomes a typed, autocompleted, statically-checked
    attribute access instead.

    Required-ness mirrors schema.yaml's required=False markers exactly:
    fields with no default below are required (yamale's default), and
    fields with `= None` / `= <value>` correspond to schema.yaml's
    explicit required=False entries, using the SAME fallback values
    daemon.py's __init__ already applies via config_dict.get(key, default)
    where one exists (e.g. OBSERVATION_DWELL_TIME defaults to 5).
    """

    STATION: Location
    EMERGENCY_CONTACT: EmergencyContact
    AZLIMITS: Limit
    ELLIMITS: Limit
    STOW_LOCATION: AzElPoint
    CAL_LOCATION: AzElPoint
    HORIZON_POINTS: list[AzElPoint] = Field(default_factory=list)

    MOTOR_TYPE: Literal["NONE", "CALTECH6M"]
    MOTOR_BAUDRATE: int
    MOTOR_PORT: str
    NUM_BEAMSWITCHES: int
    BEAMWIDTH: float

    OBSERVATION_DWELL_TIME: int = 5
    SCAN_SETTLE_TIME: float = 1.0
    ROTOR_MOVE_TIMEOUT: float = 180.0
    TRACKING_COMMAND_DEADBAND_MDEG: float = 150.0
    POINTING_ERROR_THRESHOLD_MDEG: float = 150.0
    POINTING_ERROR_STABLE_CYCLES: int = 4

    SAVE_DIRECTORY: str
    RUN_HEADLESS: bool
    DASHBOARD_PORT: int
    DASHBOARD_HOST: str 
    DASHBOARD_DOWNLOADS: bool
    DASHBOARD_REFRESH_MS: int
    DASHBOARD_REQUIRE_AUTH: Optional[bool] = None
    DASHBOARD_USERNAME: Optional[str] = None
    DASHBOARD_PASSWORD: Optional[str] = None

    WEBCAM_ENABLE: Optional[bool] = None
    WEBCAM_DEVICE_INDEX: Optional[int] = None
    WEBCAM_TARGET_FPS: Optional[float] = None
    WEBCAM_JPEG_QUALITY: Optional[int] = None
    WEBCAM_MAX_WIDTH: Optional[int] = None

    SPECTRUM_ANALYZER: Optional[SpectrumConfig] = None
    SPECTRUM_ANALYZER_SERIAL: Optional[str] = None
    SPECTRUM_ANALYZER_START_HZ: Optional[float] = None
    SPECTRUM_ANALYZER_STOP_HZ: Optional[float] = None
    SPECTRUM_ANALYZER_RBW_HZ: Optional[float] = None
    SPECTRUM_ANALYZER_VBW_HZ: Optional[float] = None
    SPECTRUM_ANALYZER_REF_LEVEL_DBM: Optional[float] = None

    END_OBSERVATION_ON_OOB: Optional[bool] = True
    STOW_ON_OOB: Optional[bool] = False

    MOTOR_LPR_PARAMS: LprParams

    @model_validator(mode="after")
    def _check_az_limits_ordered(self) -> "DaemonConfig":
        if self.AZLIMITS.lower_bound >= self.AZLIMITS.upper_bound:
            raise ValueError("AZLIMITS.lower_bound must be less than upper_bound")
        if self.ELLIMITS.lower_bound >= self.ELLIMITS.upper_bound:
            raise ValueError("ELLIMITS.lower_bound must be less than upper_bound")
        return self
