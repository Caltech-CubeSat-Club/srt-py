"""
The timeline: one row per resource, spans laid out on it.

Sits below command_types so a Span can be a plan's field without a cycle —
a Span names its command by uuid, not by object.

See docs/command-execution-design.md. STATUS: stubs.
"""

from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field, model_validator

from .common import Resource


class PlanState(Enum):
    """A running plan holds its resources until it finishes or hits
    max_duration; one that can't get them fails rather than waiting. So a
    plan is never paused part-way, and there's no suspended state."""

    QUEUED = "queued"
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"
    CANCELLED = "cancelled"


class RotorMode(Enum):
    """What the dish is doing during a rotor span. All three are claims —
    an occupied rotor row is never idleness.

    Separate because they're separate daemon state: TRACK sets
    `ephemeris_cmd_location` and lets the ephemeris thread drive, HOLD sets
    `rotor_cmd_location` and clears it.
    """

    SLEW = "slew"    # moving to a target, ends on settle
    TRACK = "track"  # following an object; dish continuously moving
    HOLD = "hold"    # stationary at fixed az/el; a drift scan's own motion


class Span(BaseModel):
    """One `Resource`, occupied for one time range.

    Derived by `observing.routines.span_for()`, never authored. Times are
    absolute so a span means the same thing after a plan is rescheduled,
    edited or reloaded from disk.
    """

    resource: Resource
    command_uuid: UUID

    start: datetime
    end: datetime

    # Rotor spans only. Meaningless on a polarization row.
    rotor_mode: Optional[RotorMode] = None

    @model_validator(mode="after")
    def _check(self) -> "Span":
        if self.end <= self.start:
            raise ValueError("span end must be after start")
        if (self.resource is Resource.ROTOR) != (self.rotor_mode is not None):
            raise ValueError("rotor_mode is required on ROTOR spans and invalid elsewhere")
        return self

    @property
    def duration_seconds(self) -> float:
        return (self.end - self.start).total_seconds()

    def overlaps(self, other: "Span") -> bool:
        """Determines whether `self` overlaps with `other`, i.e. same resource and intersecting time. 
        A rotor span and a polarization span at the same instant don't overlap since they occupy different resources."""
        return self.resource == other.resource and not(self.end<=other.start or other.end<=self.start)


class Timeline(BaseModel):
    """Every span currently claimed, across all plans.

    Owned by the command-handler thread — single writer, no locking. The
    status thread publishes a snapshot rather than reading this.
    """

    spans: list[Span] = Field(default_factory=list)

    def conflicts(self, candidate: list[Span]) -> list[Span]:
        """Returns a list of `Span` objects that `candidate` would double-book. Empty means admissible."""
        raise NotImplementedError

    def row(self, resource: Resource) -> list[Span]:
        """Returns the list of `Span` objects scheduled for the given `Resource`, in time order."""
        raise NotImplementedError
