from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Annotated, Any, Literal, Optional, Union
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator, model_validator

from .common import Band, CommandState, FrameRole, OutputFormat
from .scheduling import PlanState, Span
from .telescope_types import SpectrumFrame, SpectrumSettings

class CommandBase(BaseModel):
    """Every command carries its own identity and execution status.
    Fields default to a not-yet-run state."""

    # Edits, aborts and spans all address commands by this. Index can't
    # work: inserting at position 2 shifts every reference past it, so two
    # clients editing at once would act on different things.
    uuid: UUID = Field(default_factory=uuid4)

    state: CommandState = CommandState.PENDING
    detail: str = ""
    progress: float = Field(default=0.0, ge=0.0, le=1.0)



class PrimitiveCommand(CommandBase):
    """Occupies one span on one resource row (see `scheduling.py`).
    What `observing.routines.advance()` actually runs.

    Aggregates expand into these, and operators can author them directly —
    Saren's "observing file with manual pointings" is a primitive sequence.

    Nearly a marker — it adds `from_command` and nothing else. The layout
    logic is `observing.routines.span_for()`, kept out of this module so the
    models don't import driver or observing code.
    """

    # The authored command this was expanded from, so the web app can
    # collapse a long expansion back into the one line somebody wrote.
    # Unset when authored directly.
    from_command: Optional[UUID] = None


class AggregateCommand(CommandBase):
    """A preset pattern that expands into primitives across several rows.

    `advance()` never sees these, only the expansion — so a new observing
    mode is a new pattern, not a new execution path.

    A pure marker: no fields, no methods. The patterns are in
    `observing.routines.expand()`.
    """



class ObservingCommand(CommandBase):
    """Commands that point at a band and produce spectra.

    Separate from CommandBase so `stow` and `wait` don't carry an empty
    frames list forever — including in the generated TypeScript, where every
    interface would claim a results field it can't populate.

    Keeping the 2 Hz tick small is `status.TICK_EXCLUDE`'s job, not a reason
    to store results elsewhere.
    """

    band: Band
    frames: list[SpectrumFrame] = Field(default_factory=list)

    # None means the band's default (ObservingSettings.spectrum_settings_per_band),
    # filled in at submission by routines.resolve_spectrum_settings — so after
    # that, every observing command carries the settings it actually uses, and
    # editing the band defaults doesn't change a plan already queued.
    spectrum_settings: Optional[SpectrumSettings] = None

class PointAtObject(PrimitiveCommand):
    """Point telescope at named astronomical object."""

    command: Literal["point_at_object"] = "point_at_object"
    object_id: str

class PointAtAzEl(PrimitiveCommand):
    """Point telescope at absolute azimuth/elevation position."""

    command: Literal["point_at_azel"] = "point_at_azel"

    azimuth: float = Field(
        description="Target azimuth in degrees"
    )

    elevation: float = Field(
        description="Target elevation in degrees"
    )

class PointAtOffset(PrimitiveCommand):
    """Move to offset from current position."""

    command: Literal["point_at_offset"] = "point_at_offset"

    azimuth_offset: float = Field(
        description="Azimuth offset in degrees"
    )

    elevation_offset: float = Field(
        description="Elevation offset in degrees"
    )

class Wait(PrimitiveCommand):
    """Wait for a specified duration."""

    command: Literal["wait"] = "wait"
    
    duration_seconds: float = Field(
        gt=0,
        description="Duration in seconds"
    )

class WaitUntil(PrimitiveCommand):
    """Wait until a specific absolute time."""
    command: Literal["wait_until"] = "wait_until"

    time: datetime = Field(
        description="Absolute date/time to wait until. Naive values are read as UTC."
    )

    @field_validator("time")
    @classmethod
    def _to_utc(cls, v: datetime) -> datetime:
        return v.replace(tzinfo=timezone.utc) if v.tzinfo is None else v.astimezone(timezone.utc)

class Stow(PrimitiveCommand):
    """Move telescope to stow position."""

    command: Literal["stow"] = "stow"

class CalibrateEncoders(PrimitiveCommand):
    """Run telescope encoder calibration."""

    command: Literal["calibrate_encoders"] = "calibrate_encoders"

class SpectrumStart(PrimitiveCommand):
    """Start spectrum analyzer."""

    command: Literal["spectrum_start"] = "spectrum_start"

class SpectrumStop(PrimitiveCommand):
    """Stop spectrum analyzer."""

    command: Literal["spectrum_stop"] = "spectrum_stop"

class EmergencyStop(PrimitiveCommand):
    """Emergency stop telescope."""

    command: Literal["emergency_stop"] = "emergency_stop"


class Integrate(PrimitiveCommand, ObservingCommand):
    """Integrate wherever the dish is pointing, producing `integrations`
    frames.

    Runs in whole integrations, because that's all the driver can do: each
    lasts integration_duration(spectrum_settings) — for the Siglent,
    num_averages sweeps. Give either the count or `total_seconds`; seconds
    become a count at submission (routines.resolve_integration_counts),
    rounded up, so after that every Integrate carries a concrete count and
    the frame count and duration are exact.

    The only primitive that uses the receiver during an observation.
    Aggregates expand into these, interleaved with pointing commands; an
    operator can also write them by hand between pointing steps.

    Says nothing about pointing — `role` records what the pointing *meant*,
    and goes into the frame's metadata.

    Which polarization row it occupies is span_for's call for now; it needs
    a field once a dual-polarization receiver exists.
    """

    command: Literal["integrate"] = "integrate"

    # Exactly one of these. total_seconds is replaced by a count at submission.
    integrations: Optional[int] = Field(
        default=None, ge=1, description="Frames to produce, one per driver integration."
    )
    total_seconds: Optional[float] = Field(
        default=None, gt=0, description="Minimum total; rounded up to whole integrations."
    )
    role: FrameRole = "source"

    @model_validator(mode="after")
    def _exactly_one_length(self) -> "Integrate":
        if (self.integrations is None) == (self.total_seconds is None):
            raise ValueError("set exactly one of integrations or total_seconds")
        return self


class ObserveObject(AggregateCommand, ObservingCommand):
    """Track a source, position-switching against a nearby reference."""

    command: Literal["observe_object"] = "observe_object"

    object_id: str

    # Exactly one of these; validated below rather than as an overloaded
    # single field, so a bad request fails at the websocket with a field path.
    total_time_seconds: float | None = Field(default=None, gt=0)
    desired_snr: float | None = Field(default=None, gt=0)

    output_format: OutputFormat = "raw_spectra"

    @model_validator(mode="after")
    def _exactly_one_stop_condition(self) -> "ObserveObject":
        if (self.total_time_seconds is None) == (self.desired_snr is None):
            raise ValueError("set exactly one of total_time_seconds or desired_snr")
        return self


class ParkedScan(AggregateCommand, ObservingCommand):
    """Park at a fixed az/el and integrate. Covers static point and drift
    scan — same mechanical operation, different reduction."""

    command: Literal["parked_scan"] = "parked_scan"

    azimuth: float
    elevation: float
    total_time_seconds: float = Field(gt=0)
    drift: bool = False
    output_format: OutputFormat = "raw_spectra"


class GridScan(AggregateCommand, ObservingCommand):
    """Raster a region around a center point."""

    command: Literal["grid_scan"] = "grid_scan"

    center_object_id: str | None = None
    center_azimuth: float | None = None
    center_elevation: float | None = None

    ra_span_deg: float = Field(gt=0)
    dec_span_deg: float = Field(gt=0)
    resolution_deg: float = Field(gt=0)
    output_format: OutputFormat = "raw_spectra"

    @model_validator(mode="after")
    def _one_center(self) -> "GridScan":
        by_object = self.center_object_id is not None
        by_azel = self.center_azimuth is not None and self.center_elevation is not None
        if by_object == by_azel:
            raise ValueError("give either center_object_id or center az/el, not both")
        return self


class HotColdTest(AggregateCommand, ObservingCommand):
    """Y-factor calibration. object_id omitted = pick the most-preferred
    calibrator that is actually observable."""

    command: Literal["hot_cold_test"] = "hot_cold_test"

    object_id: str | None = None


TelescopeCommand = Annotated[
    Union[
        PointAtObject,
        PointAtAzEl,
        PointAtOffset,
        Wait,
        WaitUntil,
        Stow,
        CalibrateEncoders,
        SpectrumStart,
        SpectrumStop,
        EmergencyStop,
        Integrate,
        ObserveObject,
        ParkedScan,
        GridScan,
        HotColdTest,
    ],
    Field(discriminator="command")
]


# Observation plan
class ObservationPlan(BaseModel):
    """A sequence of commands with a claim on the timeline.

    Publishing progress is just publishing this — commands carry their own
    state and results, so there's no parallel progress model to sync.
    """

    uuid: UUID = Field(default_factory=uuid4)
    name: str | None = None

    # commands -> steps -> spans, strictly in that order.
    #
    #   commands  what was asked for    may include aggregates
    #   steps     what it turned into   always primitives; what runs
    #   spans     when each step runs   one per step
    #
    # Only `commands` is authored. The other two are derived at submission,
    # because the timeline needs concrete spans to reject double-bookings,
    # and re-derived whenever the plan is edited.
    commands: list[TelescopeCommand] = Field(
        min_length=1,
        description="List of telescope commands to execute"
    )

    steps: list[PrimitiveCommand] = Field(default_factory=list)
    spans: list[Span] = Field(default_factory=list)

    state: PlanState = PlanState.QUEUED

    # When it runs, and the longest it may take. End time is derived, not
    # stored — three clocks invited them to disagree.
    # max_duration is a hard cap, not a guess — the plan is aborted at it.
    start_time: Optional[datetime] = None
    max_duration_seconds: Optional[float] = Field(default=None, gt=0)

    @property
    def end_time(self) -> Optional[datetime]:
        if self.start_time is None or self.max_duration_seconds is None:
            return None
        return self.start_time + timedelta(seconds=self.max_duration_seconds)

    # Resources are whatever the spans occupy; a separate claims field would
    # be a second answer that can disagree. Acquisition is atomic — taking
    # one, finding another busy and holding the first is how two plans
    # deadlock on each other's half.


# ---------------------------------------------------------------------------
# What the browser sends on /ws/command: one envelope, discriminated by
# `kind`. The backend validates it, pre-checks it against the latest
# DaemonStatus, and answers ok/error before anything reaches the daemon.
# ---------------------------------------------------------------------------


class CommandRequest(BaseModel):
    """Run one command now, outside any plan."""

    kind: Literal["command"] = "command"
    command: TelescopeCommand


class SettingsPatch(BaseModel):
    """A partial settings.RuntimeSettings, e.g.
    {"scans": {"dwell_time_seconds": 10}}. Nested dicts merge; anything else
    replaces.

    A dict rather than a model because a model would be a second copy of
    RuntimeSettings with every field optional at every depth. It's validated
    by merging into the current settings (settings.merge_settings) — in the
    backend against the last status tick, and again in the daemon.
    """

    kind: Literal["settings"] = "settings"
    patch: dict[str, Any] = Field(min_length=1)


# Plan operations — the protocol is operations, not "here is a plan"; see
# docs/command-execution-design.md. Commands are addressed by uuid, never by
# index (see CommandBase.uuid). Only PENDING commands can be edited; cancel
# works at any point.


class _PlanOperationBase(BaseModel):
    kind: Literal["plan"] = "plan"


class SubmitPlan(_PlanOperationBase):
    op: Literal["submit"] = "submit"
    plan: ObservationPlan


class CancelPlan(_PlanOperationBase):
    """Cancel a whole plan, or one command in it if command_uuid is set."""

    op: Literal["cancel"] = "cancel"
    plan_uuid: UUID
    command_uuid: Optional[UUID] = None


class InsertCommand(_PlanOperationBase):
    op: Literal["insert"] = "insert"
    plan_uuid: UUID
    after: Optional[UUID] = Field(default=None, description="None inserts at the start.")
    command: TelescopeCommand


class RemoveCommand(_PlanOperationBase):
    op: Literal["remove"] = "remove"
    plan_uuid: UUID
    command_uuid: UUID


class ReplaceCommand(_PlanOperationBase):
    op: Literal["replace"] = "replace"
    plan_uuid: UUID
    command_uuid: UUID
    command: TelescopeCommand


class ReorderCommands(_PlanOperationBase):
    """The plan's PENDING commands, by uuid, in their new order."""

    op: Literal["reorder"] = "reorder"
    plan_uuid: UUID
    order: list[UUID] = Field(min_length=1)


PlanOperation = Annotated[
    Union[SubmitPlan, CancelPlan, InsertCommand, RemoveCommand, ReplaceCommand, ReorderCommands],
    Field(discriminator="op"),
]

ClientRequest = Annotated[
    Union[CommandRequest, SettingsPatch, PlanOperation],
    Field(discriminator="kind"),
]
