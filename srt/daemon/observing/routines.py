"""
Turning commands into spans, and running them.

Functions rather than methods on the command models, so `command_types`
stays free of driver and observing imports. `PrimitiveCommand` and
`AggregateCommand` are markers saying which treatment applies.

**Submission**, when a `submit` operation comes off the queue:

    validate_observation_plan(plan, ...)   -> problems? reject
    expand(cmd, ...) for each aggregate    -> plan.steps
    span_for(step, ...) for each step      -> plan.spans
    timeline.conflicts(plan.spans)         -> double-booked? reject
    plan.state = QUEUED                       admitted; slot is held

Expansion happens here, not later: the timeline needs concrete spans to
reject double-bookings.

**Execution**, in the command-handler thread's loop:

    acquire plan.spans' resources atomically -> failed? cancel the plan
    advance(step, ...) for each running step -> dispatch or poll; no blocking
    drain the worker result queue            -> append frames, update state
    now > plan.end_time                      -> abort
    all steps DONE and frames on disk        -> drop the plan

The handler never blocks. Work happens in per-resource workers; `advance()`
starts it and later calls check on it.

STATUS: stubs.
"""

from __future__ import annotations

from datetime import datetime
from typing import TYPE_CHECKING

from ..common import Band
from ..scheduling import Span
from .settings import ObservingSettings

if TYPE_CHECKING:
    from ..command_types import AggregateCommand, ObservationPlan, PrimitiveCommand
    from ..radio_control.driver import SpectrumDriver
    from ..telescope_types import DaemonConfig


def expand(
    command: "AggregateCommand",
    start: datetime,
    driver: "SpectrumDriver",
    settings: ObservingSettings,
    config: "DaemonConfig",
) -> list["PrimitiveCommand"]:
    """The primitive sequence an aggregate stands for.

    Patterns:

        ObserveObject  alternates on both rows — track on source, integrate,
                       slew by cold_offset_deg, track off source, integrate.
                       Position switching moves the dish, so the rotor is
                       occupied throughout, not just for the opening slew.
        ParkedScan     slew, then hold; the sky drifts through
        GridScan       slew then hold, once per grid point
        HotColdTest    ObserveObject's alternation against a calibrator

    Rotor spans must cover the whole time a pointing is depended on,
    including the holds. A gap invites another plan to take the dish.

    Each returned primitive sets `from_command` to this command's uuid.
    """
    raise NotImplementedError


def span_for(
    command: "PrimitiveCommand",
    start: datetime,
    config: "DaemonConfig",
) -> Span:
    """The single span a primitive occupies, starting at `start`.

    Deterministic except slews. Integration duration is
    `num_averages * sweep_time_seconds`, both chosen; waits are stated. Slew
    time comes from `config.MOTOR_LPR_PARAMS` — distance over `pAzVmax` plus
    the `pAzAmax` ramp, plus settle. Margin on top is open (see design doc).

    Rotor spans need a `rotor_mode`: TRACK for `PointAtObject` (dish keeps
    moving), HOLD for `PointAtAzEl` and `Wait` (stationary), SLEW for the
    approach to either.
    """
    raise NotImplementedError


def advance(
    command: "PrimitiveCommand",
    driver: "SpectrumDriver",
    settings: ObservingSettings,
) -> None:
    """Start or poll a primitive. Never blocks; returns immediately.

    Updates `state`, `progress` and `detail` in place. Called repeatedly —
    the first call dispatches work to the resource's worker, later ones
    check whether it finished.

    **Every row has a worker; the handler only sets intent and polls.**

        rotor   `update_ephemeris_location` and `update_rotor_status`
                already do this. TRACK sets `ephemeris_cmd_location`; HOLD
                sets `rotor_cmd_location` and clears the ephemeris one.
        radio   one worker per polarization, so POL_X and POL_Y can
                integrate at the same time — which the resource model
                requires and a blocking handler makes impossible.

    The worker owns the whole slow part: `do_one_integration` plus writing
    the frame to `SAVE_DIRECTORY`. Neither the instrument nor the disk ever
    stalls scheduling.

    Completed frames come back through a queue rather than being appended
    by the worker, so the plan set keeps its single-writer property.

    Builds the `FrameMetadata` it hands the driver — pointing, role, band,
    parallactic angle. The driver knows none of that.
    """
    raise NotImplementedError


def needs_calibration(
    plan: "ObservationPlan", band: Band, settings: ObservingSettings
) -> bool:
    """True if the next command in `band` has no valid calibration behind it.

    Two triggers: elapsed time past `settings.calibration_interval_seconds`,
    or the last calibration was in a different band — a different RF switch
    position, so a different signal path, which recency doesn't rescue.

    Track validity per band. One "last calibrated" timestamp would make a
    plan alternating L and S look continuously calibrated while being
    calibrated for neither.
    """
    raise NotImplementedError


# A plan's resources are `{span.resource for span in plan.spans}`. Don't add
# a per-command claims() — a second answer to the same question can disagree
# with the first.
