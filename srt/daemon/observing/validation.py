"""
Answering "can this plan run" before committing the dish to it.

The predicate that matters is terrain, not elevation. `HORIZON_POINTS` in
config.yaml is an az/el obstruction profile currently published to the
frontend and used in no predicate anywhere. CasA and CygA can sit above
ELLIMITS and still be behind a ridge, making the "hot" load a ~300 K
mountain and T_receiver come out wildly low.

Worth checking against the existing non-reproducible hot/cold data before
hunting for a subtler cause — both transit at northern azimuths, so
contamination would depend on where you pointed, which is the symptom.

STATUS: stubs.
"""

from __future__ import annotations

from typing import Optional

from ..telescope_types import DaemonConfig
from ..utilities.object_tracker import EphemerisTracker
from .settings import HotCalibrator
from ..command_types import ObservationPlan


def is_clear_of_terrain(
    azimuth_deg: float,
    elevation_deg: float,
    config: DaemonConfig,
    margin_deg: float = 0.0,
) -> bool:
    """Above the interpolated HORIZON_POINTS profile at this azimuth.

    HORIZON_POINTS is a sparse list of (az, el) samples, so this has to
    interpolate between them, and wrap correctly across 0/360.
    """
    raise NotImplementedError


def will_source_be_in_sky_for_whole_observation(
    object_id: str,
    duration_seconds: float,
    tracker: EphemerisTracker,
    config: DaemonConfig,
    sample_interval_seconds: float = 60.0,
) -> bool:
    """Sample the source's track and check every sample clears terrain and
    the mount limits.

    Implement by walking EphemerisTracker.get_azimuth_elevation(name,
    offset) over the duration. Do NOT use get_all_azel_time()'s cached
    grid: it samples 0-60 s at 5 s steps and then jumps straight to 1 h,
    so it has no resolution at all in the range that matters here.

    astroplan is a declared dependency that nothing currently imports and
    would give rise/set times directly — worth using rather than
    hand-rolling, but it means this module owns the astroplan Observer.
    """
    raise NotImplementedError


def select_hot_calibrator(
    candidates: list[HotCalibrator],
    duration_seconds: float,
    tracker: EphemerisTracker,
    config: DaemonConfig,
) -> Optional[HotCalibrator]:
    """First candidate in preference order that stays observable for the
    whole calibration. None if nothing qualifies."""
    raise NotImplementedError


def validate_observation_plan(
    plan: ObservationPlan,
    tracker: EphemerisTracker,
    config: DaemonConfig,
) -> list[str]:
    """Returns a list of problems; empty means the plan is runnable.

    Returning reasons rather than the pseudocode's bool because "no" is
    useless to an operator at 2am. Checks to cover:
      - every tracked source stays observable for its step's duration
      - every commanded az/el is within mount limits and clear of terrain
      - requested output format is producible by the chosen driver
        (stokes needs two polarizations; the Siglent has one)
      - every observing command has spectrum settings: its own override, or
        a default for its band
    """
    raise NotImplementedError
