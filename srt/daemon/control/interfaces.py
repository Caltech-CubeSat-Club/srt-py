"""Abstract control interfaces for telescope pointing and tracking.

These interfaces define a backend-agnostic contract so real hardware
(Caltech6m) and simulation can share the same command paths.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Iterable, Optional


@dataclass
class PointingState:
    """Snapshot of telescope control state."""

    azimuth_deg: float
    elevation_deg: float
    target_name: Optional[str] = None
    is_tracking: bool = False
    backend_state: str = "READY"
    last_error: Optional[str] = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class TrackingSequence:
    """Container for pre-programmed tracking steps.

    Each step is intentionally lightweight and backend-agnostic.
    The expected shape for each element is a mapping with keys such as:
    - mode: "altaz" | "radec" | "object"
    - values: tuple/list payload for that mode
    - dwell_s: optional float dwell time at each point
    """

    name: str
    steps: list[dict[str, Any]]
    repeat: bool = False


class PointingController(ABC):
    """Core az/el control contract used by daemon command handlers."""

    @abstractmethod
    def point_azel(self, azimuth_deg: float, elevation_deg: float) -> None:
        """Point telescope at a direct azimuth/elevation target."""

    @abstractmethod
    def set_offset(self, azimuth_offset_deg: float, elevation_offset_deg: float) -> None:
        """Apply pointing offsets relative to the active destination."""

    @abstractmethod
    def stow(self) -> None:
        """Move telescope to configured stow position."""

    @abstractmethod
    def stop(self) -> None:
        """Stop any in-progress slew/track operation."""

    @abstractmethod
    def get_state(self) -> PointingState:
        """Return the latest pointing and control state."""


class TrackingProgrammer(ABC):
    """Astropy-friendly abstraction for target tracking workflows."""

    @abstractmethod
    def track_object(self, name: str, cadence_s: float = 1.0, horizon_deg: float = 15.0) -> None:
        """Track a named celestial object (resolved via astropy tooling)."""

    @abstractmethod
    def track_radec(self, ra_deg: float, dec_deg: float, cadence_s: float = 1.0) -> None:
        """Track a fixed RA/Dec target converted to Alt/Az over time."""

    @abstractmethod
    def track_altaz_sequence(
        self, points: Iterable[tuple[float, float]], dwell_s: float = 0.0
    ) -> None:
        """Execute an ordered Alt/Az sequence, optionally dwelling at points."""

    @abstractmethod
    def run_sequence(self, sequence: TrackingSequence) -> None:
        """Execute a pre-defined mixed-mode tracking sequence."""

    @abstractmethod
    def stop_tracking(self) -> None:
        """Stop active tracking/sequence execution."""
