"""
Smoke tests for the real config/config.yaml against the DaemonConfig model.

The repo's own config is the one the telescope actually runs on, so a field
added to the model without a matching entry (or vice versa) should fail here
rather than at daemon startup on the roof.
"""

from pathlib import Path

import pytest
from pydantic import ValidationError

from srt.config_loader import load_config

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG = REPO_ROOT / "config" / "config.yaml"


def test_repo_config_loads_and_validates():
    config = load_config(CONFIG)
    assert config.STATION.name


def test_limits_are_ordered():
    config = load_config(CONFIG)
    assert config.AZLIMITS.lower_bound < config.AZLIMITS.upper_bound
    assert config.ELLIMITS.lower_bound < config.ELLIMITS.upper_bound


def test_stow_and_cal_locations_are_within_limits():
    """A stow position outside the mount's own limits would be
    unreachable, and only discovered when something tries to stow."""
    config = load_config(CONFIG)
    for name, loc in [("STOW", config.STOW_LOCATION), ("CAL", config.CAL_LOCATION)]:
        assert config.AZLIMITS.lower_bound <= loc.azimuth <= config.AZLIMITS.upper_bound, name
        assert config.ELLIMITS.lower_bound <= loc.elevation <= config.ELLIMITS.upper_bound, name


def test_invalid_config_raises_with_a_field_path(tmp_path):
    """load_config's contract is a ValidationError naming the bad field,
    not a KeyError three layers into daemon startup."""
    bad = tmp_path / "config.yaml"
    bad.write_text("STATION:\n  latitude: not-a-number\n")
    with pytest.raises(ValidationError):
        load_config(bad)
