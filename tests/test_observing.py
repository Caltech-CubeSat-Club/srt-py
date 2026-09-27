"""
The observing layer's submission-time decisions. Most of routines.py is
still stubs; this covers what isn't.
"""

from srt.daemon import command_types as ct
import pytest
from pydantic import ValidationError

from srt.daemon.observing.routines import resolve_integration_counts, resolve_spectrum_settings
from srt.daemon.observing.settings import ObservingSettings
from srt.daemon.telescope_types import SpecanSettings


def _specan(start_hz: float, stop_hz: float) -> SpecanSettings:
    return SpecanSettings(
        start_hz=start_hz, stop_hz=stop_hz,
        resolution_bandwidth_hz=1e6, video_bandwidth_hz=1e5, reference_level_dbm=-30,
    )


L_DEFAULT = _specan(1.0e9, 1.9e9)
HI_NARROW = _specan(1.415e9, 1.425e9)
OBSERVING = ObservingSettings(
    switching_time_seconds=60,
    desired_snr=10,
    spectrum_settings_per_band={"L": L_DEFAULT},
    hot_calibrators=[],
    cold_offset_deg=5.0,
    calibration_integration_seconds=30.0,
    calibration_interval_seconds=360.0,
)


def test_band_default_fills_in_and_overrides_survive():
    """An override is kept; anything without one gets its band's default —
    including an Integrate written by hand."""
    plan = ct.ObservationPlan(commands=[
        ct.ObserveObject(object_id="CassA", band="L", total_time_seconds=600),
        ct.ObserveObject(object_id="CygA", band="L", total_time_seconds=600, spectrum_settings=HI_NARROW),
        ct.Integrate(band="L", integrations=10),
    ])
    resolve_spectrum_settings(plan, OBSERVING)
    assert [c.spectrum_settings for c in plan.commands] == [L_DEFAULT, HI_NARROW, L_DEFAULT]


def test_editing_band_defaults_later_leaves_a_resolved_plan_alone():
    """Resolution copies the settings into the plan, so a queued plan runs
    with what it was admitted with."""
    plan = ct.ObservationPlan(commands=[ct.Integrate(band="L", integrations=10)])
    resolve_spectrum_settings(plan, OBSERVING)

    edited = OBSERVING.model_copy(update={"spectrum_settings_per_band": {"L": HI_NARROW}})
    resolve_spectrum_settings(plan, edited)
    assert plan.commands[0].spectrum_settings == L_DEFAULT


def test_non_observing_commands_are_untouched():
    plan = ct.ObservationPlan(commands=[ct.Stow(), ct.Wait(duration_seconds=5)])
    resolve_spectrum_settings(plan, OBSERVING)
    assert not any(hasattr(c, "spectrum_settings") for c in plan.commands)


class _FixedDurationDriver:
    """Stands in for a SpectrumDriver: resolution only asks it how long one
    integration takes."""

    def __init__(self, seconds: float):
        self.seconds = seconds

    def integration_duration(self, settings) -> float:
        return self.seconds


@pytest.mark.parametrize(
    "per,total,expected",
    [
        (3.0, 60.0, 20),   # divides evenly
        (7.0, 60.0, 9),    # rounds up: 63 s, never less than asked
        (0.3, 0.9, 3),     # 0.9 / 0.3 is 3.0000000000000004 in floats
    ],
)
def test_total_seconds_becomes_a_count(per, total, expected):
    plan = ct.ObservationPlan(commands=[ct.Integrate(band="L", total_seconds=total)])
    resolve_spectrum_settings(plan, OBSERVING)
    resolve_integration_counts(plan, _FixedDurationDriver(per))
    (cmd,) = plan.commands
    assert (cmd.integrations, cmd.total_seconds) == (expected, None)


def test_a_given_count_is_left_alone():
    plan = ct.ObservationPlan(commands=[ct.Integrate(band="L", integrations=4)])
    resolve_spectrum_settings(plan, OBSERVING)
    resolve_integration_counts(plan, _FixedDurationDriver(3.0))
    assert plan.commands[0].integrations == 4


@pytest.mark.parametrize("lengths", [{}, {"integrations": 4, "total_seconds": 60.0}])
def test_integrate_needs_exactly_one_length(lengths):
    with pytest.raises(ValidationError, match="exactly one"):
        ct.Integrate(band="L", **lengths)
