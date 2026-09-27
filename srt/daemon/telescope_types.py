"""
telescope_state.py — Pydantic models for all telescope state.

Strict conversion from the original dataclass version: no legacy
flat-key emission, no lenient string coercion (e.g. "yes"/"on" -> bool).
Bad input raises a ValidationError rather than being silently dropped
or defaulted. This is intentional per the decision to prioritize
correctness over backward compatibility with old ZMQ payload shapes.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Annotated, Optional, Literal, Union, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

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

    All 37 values default to None, meaning "not yet loaded".
    The dashboard antenna page shows — for None values.

    Assembled by from_parts() from the runtime-editable tuning
    (RuntimeSettings.lpr) and the static encoder geometry
    (DaemonConfig.MOTOR_ENCODER_PARAMS). to_command_string() renders the full LPR,v1,v2,... string so
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

    # A typo'd name in a settings edit should fail, not vanish.
    model_config = ConfigDict(extra="forbid")

    # Meanings come from the controller's own source, recovered from the
    # Servo Automation Workbench project: 6m_docs/6mv105c.saw, as text in
    # 6m_docs/memos/2026-03-28-6mv105c-plaintext-strings.txt (procedures
    # TaskPositionLoop, GetData, SetTorque, CmdLPR); the reverse-engineering
    # memo alongside it has the equations and a glossary.
    #
    # The loop runs every 0.005 s. Per tick, per axis:
    #   command pre-filter  smooths CmdAz into CppAz, producing a rate Azu
    #   position PI + FF    CmdVel = Kpp*err + Kpi*∫err + Kff*Azu  (antenna deg/s)
    #   axis → motor        CmdVel *= Ar, then Amax/Vmax limits    (motor deg/s)
    #   velocity PI         CmdTorque = Kvp*velerr + Kvi*∫velerr   (DAC counts)
    #   torque clamp        Tmin..Tmax, then Tbias split across the two az motors

    # Command pre-filter gains (1/s). The filter tracks the raw command with
    # a rate of Kp*lag plus the command's own rate; Kp is Ko while the lag
    # exceeds e, Kv once within it — a stiffer gain for slews, a gentler one
    # for tracking.
    pAzKo: Optional[float] = None
    pElKo: Optional[float] = None
    pAzKv: Optional[float] = None
    pElKv: Optional[float] = None

    # Pre-filter lag threshold (antenna deg) switching Kp between Ko and Kv.
    # Not a pointing deadband.
    pAze: Optional[float] = None
    pEle: Optional[float] = None

    # Acceleration limits (antenna deg/s²). Applied to the motor velocity
    # command as Amax*Ar, and to the pre-filter at 0.8*Amax.
    pAzAmax: Optional[float] = None
    pElAmax: Optional[float] = None

    # Velocity ceilings (antenna deg/s) — raise to allow faster slews.
    # Applied as Vmax*Ar to the motor command, and at 0.9*Vmax to the
    # pre-filter.
    pAzVmax: Optional[float] = None
    pElVmax: Optional[float] = None

    # Position integrator anti-windup clamps (deg·s): ∫err is clipped to ±Imax.
    pAzImax: Optional[float] = None
    pElImax: Optional[float] = None

    # Position loop P and I gains: antenna deg/s per deg of error, and per
    # deg·s of integrated error.
    pAzKpp: Optional[float] = None
    pElKpp: Optional[float] = None
    pAzKpi: Optional[float] = None
    pElKpi: Optional[float] = None

    # Velocity loop P and I gains, motor deg/s error → DAC torque counts —
    # reduce pAzKvp to fix azimuth overshoot. The velocity integrator is
    # unclamped.
    pAzKvp: Optional[float] = None
    pElKvp: Optional[float] = None
    pAzKvi: Optional[float] = None
    pElKvi: Optional[float] = None

    # Feedforward of the pre-filter's rate (Azu) into the velocity command.
    # Dimensionless; 1.0 passes the planned rate straight through.
    pAzKff: Optional[float] = None
    pElKff: Optional[float] = None

    # Torque command limits (DAC counts; 2047 = 10 V at the DAC).
    pAzTmax: Optional[float] = None
    pElTmax: Optional[float] = None
    pAzTmin: Optional[float] = None
    pElTmin: Optional[float] = None

    # Torque sign. Loaded and displayed, but nothing in the recovered source
    # reads it — the sign reversal in TaskPositionLoop is commented out.
    pAzTsgn: Optional[float] = None
    pElTsgn: Optional[float] = None

    # Anti-backlash bias (DAC counts). Azimuth is driven by two motors; one
    # gets torque + Tbias and the other torque − Tbias, so they preload the
    # gear train against each other instead of both letting go at zero.
    pAzTbias: Optional[float] = None

    # Antenna-axis encoder counts per revolution. Position is
    # counts * 360/Ecr; 144000 on the 6 m, i.e. 400 counts/deg.
    pAzEcr: Optional[float] = None
    pElEcr: Optional[float] = None

    # Motor encoder counts per revolution, for motor velocity from count
    # deltas: 360*ΔC / (0.005 * MEcr). 20000 on the 6 m.
    pAzMEcr: Optional[float] = None
    pElMEcr: Optional[float] = None

    # Encoder phase offsets (deg) — critical for calibration accuracy. CLE
    # uses them to reconstruct absolute position from the index pulse:
    # round(400*(Theta − Epo)) + position − index position. The 400 is
    # hardcoded in the controller, so these assume Ecr = 144000.
    pAzEpo: Optional[float] = None
    pElEpo: Optional[float] = None

    # Axis ratios: motor deg per antenna deg (gear ratio). 1522.5 az and
    # 19749 el on the 6 m.
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

    @classmethod
    def from_parts(cls, tuning: "LprTuning", encoder: "LprEncoderParams") -> "LprParams":
        """Reassemble the full set from its runtime and static halves."""
        return cls(**tuning.model_dump(), **encoder.model_dump())


class LprEncoderParams(BaseModel):
    """The encoder-geometry slice of LPR: counts per revolution and phase
    offsets. Calibration results rather than tuning, so static config
    (DaemonConfig.MOTOR_ENCODER_PARAMS) — a bad phase offset mis-points the
    dish from the next calibration on.
    """

    model_config = ConfigDict(extra="forbid")

    # Encoder counts per revolution (antenna axis, then motor axis)
    pAzEcr: float
    pElEcr: float
    pAzMEcr: float
    pElMEcr: float

    # Encoder phase offsets (deg)
    pAzEpo: float
    pElEpo: float


class LprTuning(BaseModel):
    """The rest of LPR: servo gains and limits. Runtime-editable
    (RuntimeSettings.lpr). Field meanings are on LprParams.

    Together with LprEncoderParams this must cover LprParams exactly;
    tests/test_settings.py checks.
    """

    model_config = ConfigDict(extra="forbid")

    pAzKo: float
    pElKo: float
    pAzKv: float
    pElKv: float
    pAze: float
    pEle: float
    pAzAmax: float
    pElAmax: float
    pAzVmax: float
    pElVmax: float
    pAzImax: float
    pElImax: float
    pAzKpp: float
    pElKpp: float
    pAzKpi: float
    pElKpi: float
    pAzKvp: float
    pElKvp: float
    pAzKvi: float
    pElKvi: float
    pAzKff: float
    pElKff: float
    pAzTmax: float
    pElTmax: float
    pAzTmin: float
    pElTmin: float
    pAzTsgn: float
    pElTsgn: float
    pAzTbias: float
    pAzAr: float
    pElAr: float


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

class SpecanSettings(BaseModel):
    """Siglent spectrum analyzer instrument settings. Live-updatable.

    Down here rather than in radio_control/driver.py because SpectrumFrame
    embeds it, and radio_control imports this module. driver.py re-exports.

    Replaced the old SpectrumConfig, which also carried plot display
    preferences (never reached the analyzer) and the instrument serial
    (hardware identity — DaemonConfig.SPECTRUM_ANALYZER_SERIAL now).
    """

    # Typos in operator-typed updates should fail, not be silently dropped.
    model_config = ConfigDict(extra="forbid")

    driver: Literal["specan"] = "specan"

    # Frequency can be expressed either way; the daemon currently runs in
    # start_stop mode, so both pairs have to survive.
    freq_mode: Literal["start_stop", "center_span"] = "start_stop"
    start_hz: Optional[float] = Field(default=None, gt=0)
    stop_hz: Optional[float] = Field(default=None, gt=0)
    center_frequency_hz: Optional[float] = Field(default=None, gt=0)
    span_hz: Optional[float] = Field(default=None, gt=0)

    resolution_bandwidth_hz: float = Field(gt=0)
    video_bandwidth_hz: float = Field(gt=0)
    reference_level_dbm: float
    num_averages: int = Field(default=300, ge=1)
    trace_type: Literal["clear_write", "average"] = "clear_write"

    attenuation_db: Optional[float] = None
    attenuation_auto: bool = True
    preamp_on: Optional[bool] = True

    # Not honored by SiglentDriver yet — it still sends :SWE:TIME:AUTO ON
    # and takes the instrument's point count.
    sweep_time_seconds: Optional[float] = None
    number_of_points: Optional[int] = None

    @model_validator(mode="after")
    def _check_freq_range(self) -> "SpecanSettings":
        if self.freq_mode == "start_stop":
            if self.start_hz is None or self.stop_hz is None:
                raise ValueError("start_stop mode needs start_hz and stop_hz")
            if self.start_hz >= self.stop_hz:
                raise ValueError("start_hz must be less than stop_hz")
        elif self.center_frequency_hz is None or self.span_hz is None:
            raise ValueError("center_span mode needs center_frequency_hz and span_hz")
        return self

    @property
    def center_hz(self) -> float:
        if self.freq_mode == "center_span":
            return cast(float, self.center_frequency_hz)
        return (cast(float, self.start_hz) + cast(float, self.stop_hz)) / 2


class RfsocSettings(BaseModel):
    """RFSoC. Extend when the board arrives.

    switch1/switch2 from the original sketch are band-select RF switches on
    the board's GPIOs, so they're not fields here — `band` is what a person
    chooses and the mapping belongs in the driver. Two ways to say one thing
    is two ways to disagree.

    The consequence that does reach the observing layer: throwing those
    switches changes the RF path and invalidates calibration.
    """

    model_config = ConfigDict(extra="forbid")

    driver: Literal["rfsoc"] = "rfsoc"

    attenuation_db: float
    # TODO(shaurya/danica): real field list once the interface is known.


SpectrumSettings = Annotated[
    Union[SpecanSettings, RfsocSettings],
    Field(discriminator="driver"),
]


# Shared vocabulary lives in common.py — the bottom of the graph.
from .common import Band, CommandState, FrameRole, OutputFormat  # noqa: F401  (re-export)


class FrameMetadata(BaseModel):
    """Where the dish was and what it was doing when a frame was taken.

    Without this a frame is unreducible: it has power vs frequency and no
    record of what was being looked at. Everything here has to be captured
    per integration, not per observation, because the dish is tracking
    throughout.

    Optional on SpectrumFrame because the driver free-runs outside any
    observation — frames from the idle acquisition loop legitimately have no
    observing context.
    """

    azimuth_deg: float
    elevation_deg: float

    # Which leg of a switching cycle. Y-factor reduction is impossible
    # without it, and it cannot be reconstructed afterwards from pointing
    # alone once the dish has moved on.
    role: FrameRole = "source"

    object_id: Optional[str] = None
    band: Optional[Band] = None

    # An az-el mount rotates the feed relative to the sky while tracking, so
    # a fixed source rotates through the beam's polarization axes over a long
    # integration. Uncorrected, that gain drift is indistinguishable from the
    # source's flux actually changing.
    parallactic_angle_deg: Optional[float] = None


class SpectrumFrame(BaseModel):
    """One acquired and processed spectrum snapshot."""

    freq_hz: list[float] = Field(default_factory=list)
    power_dbm: list[float] = Field(default_factory=list)
    raw_dbm: list[float] = Field(default_factory=list)
    sweep_index: int = 0
    # Unix time the integration ended.
    timestamp: float = 0.0
    # What the instrument was set to for this sweep. None only on the
    # placeholder frame DaemonStatus defaults to.
    config: Optional[SpectrumSettings] = None
    avg_count: int = 1
    # Measured, not requested: the radiometer equation needs how long the
    # receiver actually integrated. None only on the placeholder frame.
    integration_seconds: Optional[float] = None
    connected: bool = False
    metadata: Optional[FrameMetadata] = None

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
    """Top-level config.yaml structure: hardware and site facts, fixed at
    startup. Load with config_loader.load_config(path).

    Knobs an operator changes mid-session live in settings.RuntimeSettings
    (config/settings.yaml, machine-written) instead.

    AZLIMITS, ELLIMITS, STOW_LOCATION, CAL_LOCATION and HORIZON_POINTS stay
    here on purpose: they're safety and site-survey values, and a web UI
    that can widen the mount's limits at runtime is a way to drive the dish
    into a hard stop.
    """

    # Pydantic ignores unknown keys by default, which would let a field that
    # moved to settings.yaml linger here, load fine, and do nothing.
    model_config = ConfigDict(extra="forbid")

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

    # Not a beamwidth: that depends on frequency, so it's derived per use —
    # see beamwidth_deg().
    DISH_DIAMETER_M: float = Field(gt=0)

    SAVE_DIRECTORY: str
    RUN_HEADLESS: bool
    DASHBOARD_PORT: int
    DASHBOARD_HOST: str
    DASHBOARD_REQUIRE_AUTH: Optional[bool] = None
    DASHBOARD_USERNAME: Optional[str] = None
    DASHBOARD_PASSWORD: Optional[str] = None

    WEBCAM_ENABLE: Optional[bool] = None
    WEBCAM_DEVICE_INDEX: Optional[int] = None

    SPECTRUM_ANALYZER_SERIAL: str

    # The static slice of the servo controller's LPR parameters; the tuning
    # half is RuntimeSettings.lpr.
    MOTOR_ENCODER_PARAMS: LprEncoderParams

    @model_validator(mode="after")
    def _check_az_limits_ordered(self) -> "DaemonConfig":
        if self.AZLIMITS.lower_bound >= self.AZLIMITS.upper_bound:
            raise ValueError("AZLIMITS.lower_bound must be less than upper_bound")
        if self.ELLIMITS.lower_bound >= self.ELLIMITS.upper_bound:
            raise ValueError("ELLIMITS.lower_bound must be less than upper_bound")
        return self

    def beamwidth_deg(self, freq_hz: float) -> float:
        """Half-power beamwidth. 1.22 λ/D is the usual figure for a
        taper-illuminated dish, not a measurement of this one."""
        wavelength_m = 299_792_458.0 / freq_hz
        return math.degrees(1.22 * wavelength_m / self.DISH_DIAMETER_M)
