"""
What the daemon publishes, once per tick, over ZMQ 5555.

Top of the dependency graph, and the only module importing both
telescope_types and command_types — which is what lets DaemonStatus carry
live plans without a cycle.
"""

from __future__ import annotations

import time as _time

from typing import Optional, Union

from pydantic import BaseModel, Field

from .command_types import ObservationPlan
from .settings import RuntimeSettings
from .telescope_types import (
    AmpCurrent,
    Location,
    LprParams,
    RotorState,
    SpectrumFrame,
    SerialCommunication,
    EmergencyContact
)

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
    # Derived from DISH_DIAMETER_M at the live analyzer's center frequency.
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
    object_time_locs: dict[int, dict[str, tuple[float, float]]] = Field(default_factory=dict)
    vlsr: dict[str, float] = Field(default_factory=dict)

    # ---- Pointing ----
    motor_offsets: tuple[float, float] = (0.0, 0.0)
    pointing_error_history: list[dict[str, float]] = Field(default_factory=list)
    amp_current_history: list[dict[str, Union[float, AmpCurrent]]] = Field(default_factory=list)

    # ---- Command queue ----
    queued_item: str = "None"
    queue_size: int = 0

    # Queued, running, and finished-but-not-yet-saved. Commands carry their
    # own state and frames, so this is both progress and history — no
    # separate log to keep in sync.
    #
    # A finished plan is dropped once its frames reach SAVE_DIRECTORY. Not a
    # count or an age: "still here" means "not yet safely on disk".
    plans: list[ObservationPlan] = Field(default_factory=list)

    # ---- Logs and events ----
    # log_message() appends (iso_timestamp, message) tuples, not bare
    # strings — confirmed from daemon.py's actual self.command_error_logs
    # usage. The list[str] guess was wrong; fixed after seeing log_message.
    error_logs: list[tuple[str, str]] = Field(default_factory=list)
    serial_communications: list[SerialCommunication] = Field(default_factory=list)
    command_history: list[CommandHistoryEntry] = Field(default_factory=list)

    # ---- Observation data ----
    n_point_data: list = Field(default_factory=list)
    beam_switch_data: list = Field(default_factory=list)
    cal_values: list[float] = Field(default_factory=list)

    # ---- Runtime settings ----
    # The daemon's current copy; the browser edits it with a partial patch.
    # None only on a status not built by the daemon.
    settings: Optional[RuntimeSettings] = None

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


# Frames live on the commands that produced them, which is right everywhere
# except this 2 Hz tick — a long observation holds thousands.
#
#     status.model_dump_json(exclude=TICK_EXCLUDE)
#
# Both lists: integrations happen on the expanded `steps`, and an aggregate
# may hold frames too. Only ObservingCommand subclasses have the field —
# Pydantic tolerates excluding an absent one, but verify the dump really
# drops them. The failure mode is a silently enormous broadcast, not an
# error.
_NO_FRAMES = {"__all__": {"frames"}}
TICK_EXCLUDE = {"plans": {"__all__": {"commands": _NO_FRAMES, "steps": _NO_FRAMES}}}
