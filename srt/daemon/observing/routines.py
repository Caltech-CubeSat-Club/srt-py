"""
Turning commands into spans, and running them.

Functions rather than methods on the command models, so `command_types`
stays free of driver and observing imports. `PrimitiveCommand` and
`AggregateCommand` are markers saying which treatment applies.

**Submission**, when a `submit` operation comes off the queue:

    validate_observation_plan(plan, ...)   -> problems? reject
    resolve_spectrum_settings(plan, ...)   -> every observing command concrete
    resolve_integration_counts(plan, ...)  -> every Integrate a count
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

STATUS: stubs, except the two resolve_* functions.
"""

from __future__ import annotations

import math
from datetime import datetime
from typing import TYPE_CHECKING

from ..command_types import Integrate, ObservingCommand
from ..common import Band
from ..scheduling import Span
from .settings import ObservingSettings

if TYPE_CHECKING:
    from ..command_types import AggregateCommand, ObservationPlan, PrimitiveCommand
    from ..radio_control.driver import SpectrumDriver
    from ..telescope_types import DaemonConfig


def resolve_spectrum_settings(plan: "ObservationPlan", settings: ObservingSettings) -> None:
    """Give every observing command without its own spectrum_settings its
    band's default, in place.

    At submission rather than run time, so a queued plan keeps its settings
    if someone edits the band defaults, and its frames record what was used.
    Runs after validation, which guarantees each band used has a default.
    """
    for cmd in plan.commands:
        if isinstance(cmd, ObservingCommand) and cmd.spectrum_settings is None:
            cmd.spectrum_settings = settings.spectrum_settings_per_band[cmd.band]


def resolve_integration_counts(plan: "ObservationPlan", driver: "SpectrumDriver") -> None:
    """Turn every Integrate given as total_seconds into a count, in place:
    total_seconds over its driver integration duration, rounded up — asking
    for 60 s of 7 s integrations gets 63 s, never less. total_seconds is
    cleared, so afterwards every Integrate is just a count.

    Runs after resolve_spectrum_settings, whose settings it needs.
    """
    for cmd in plan.commands:
        if isinstance(cmd, Integrate) and cmd.total_seconds is not None:
            assert cmd.spectrum_settings is not None, "resolve_spectrum_settings first"
            per = driver.integration_duration(cmd.spectrum_settings)
            # Rounded before ceil so 0.9 / 0.3 = 3.0000000000000004 is 3, not 4.
            cmd.integrations = math.ceil(round(cmd.total_seconds / per, 9))
            cmd.total_seconds = None


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

    The receiver side is always `Integrate` primitives, each copying this
    command's spectrum_settings — already concrete, since
    resolve_spectrum_settings runs first — and a `role` for its leg. Times
    become counts here: a switch period of `switching_time_seconds` is
    `ceil(switching_time_seconds / driver.integration_duration(settings))`
    integrations.

    Rotor spans must cover the whole time a pointing is depended on,
    including the holds. A gap invites another plan to take the dish.

    Each returned primitive sets `from_command` to this command's uuid.
    """
    raise NotImplementedError


def span_for(
    command: "PrimitiveCommand",
    start: datetime,
    driver: "SpectrumDriver",
    config: "DaemonConfig",
) -> Span:
    """The single span a primitive occupies, starting at `start`.

    Deterministic except slews. Waits state their durations; an `Integrate`
    lasts `integrations * driver.integration_duration(spectrum_settings)`.
    Slew time comes from the LPR params loaded on the controller
    (`RotorState.lpr`, not the possibly-pending `settings.lpr`) — distance
    over `pAzVmax` plus the `pAzAmax` ramp, plus settle. Margin on top is
    open (see design doc).

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

    Keyed on band only, for now — deliberately. An observation that
    overrides spectrum_settings shares its band's calibration, and the
    HotColdTest this triggers uses the band default, even though a Y-factor
    at one RBW or reference level may not transfer to another.
    """
    raise NotImplementedError


# A plan's resources are `{span.resource for span in plan.spans}`. Don't add
# a per-command claims() — a second answer to the same question can disagree
# with the first.
