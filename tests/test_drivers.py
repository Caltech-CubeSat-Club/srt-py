"""
Both receiver drivers against the SpectrumDriver contract. Mostly stubs
still, so this checks shape: each is a SpectrumDriver that can be
constructed (an ABC with a missing method fails at construction, which is
the point of it being an ABC), and the Siglent's declared capabilities match
what a single-trace instrument can do.
"""

from pathlib import Path

import pytest

from srt.daemon.radio_control import RfsocDriver, SiglentDriver
from srt.daemon.radio_control.driver import SpectrumDriver
from srt.config_loader import load_default_settings

DEFAULTS_PATH = Path(__file__).resolve().parent.parent / "config" / "settings.defaults.yaml"
DEFAULTS = load_default_settings(DEFAULTS_PATH)


def test_both_drivers_satisfy_the_contract():
    assert isinstance(SiglentDriver("NO-SUCH-SERIAL", DEFAULTS.specan_live_view), SpectrumDriver)
    assert isinstance(RfsocDriver(), SpectrumDriver)


def test_siglent_cannot_claim_stokes():
    """Stokes needs two polarizations; the Siglent has one trace. Plan
    validation rejects impossible formats using exactly this list."""
    caps = SiglentDriver("NO-SUCH-SERIAL", DEFAULTS.specan_live_view).capabilities
    assert caps.driver == "specan"
    assert caps.polarizations == 1
    assert "stokes" not in caps.supported_output_formats


def test_rfsoc_is_honest_about_being_a_stub():
    with pytest.raises(NotImplementedError):
        RfsocDriver().capabilities


def test_siglent_integration_time_covers_exactly_the_averaged_sweeps():
    """A frame's integration_seconds is the summed duration of the sweeps
    averaged into it — the rolling window's, not every sweep since start."""
    import numpy as np

    settings = DEFAULTS.specan_live_view
    driver = SiglentDriver("NO-SUCH-SERIAL", settings)
    trace = np.full(5, -80.0)

    for seconds in (1.0, 2.0, 3.0, 4.0):
        _, count, integrated = driver._apply_averaging(trace, seconds, "average", 3)
    assert (count, integrated) == (3, 9.0)

    _, count, integrated = driver._apply_averaging(trace, 0.5, "clear_write", 3)
    assert (count, integrated) == (1, 0.5)
