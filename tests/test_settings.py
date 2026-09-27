"""
The runtime-editable settings half: settings.yaml round-trips, partial
updates, and the SpecanSettings model that replaced SpectrumConfig.

Nothing here exercises the daemon itself — every one of these settings is
read on a live control path, so a regression here is otherwise found on the
roof.
"""

from pathlib import Path

import pytest
from pydantic import ValidationError

import yaml

from srt.config_loader import load_config, load_default_settings, load_settings, save_settings
from srt.daemon.radio_control.siglent_driver import SiglentDriver
from srt.daemon.settings import merge_settings, settings_overrides
from srt.daemon.telescope_types import LprEncoderParams, LprParams, LprTuning, SpecanSettings

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG = REPO_ROOT / "config" / "config.yaml"
DEFAULTS_PATH = REPO_ROOT / "config" / "settings.defaults.yaml"
DEFAULTS = load_default_settings(DEFAULTS_PATH)


def test_missing_file_means_defaults(tmp_path):
    assert load_settings(tmp_path / "settings.yaml", DEFAULTS_PATH) == DEFAULTS


def test_round_trip_leaves_config_yaml_alone(tmp_path):
    config_before = CONFIG.read_bytes()
    path = tmp_path / "settings.yaml"
    edited = merge_settings(DEFAULTS, {"scans": {"dwell_time_seconds": 12.5}})

    save_settings(edited, path, DEFAULTS_PATH)

    assert load_settings(path, DEFAULTS_PATH) == edited
    assert CONFIG.read_bytes() == config_before
    assert [p.name for p in tmp_path.iterdir()] == ["settings.yaml"], "temp file left behind"
    # Only the override is stored, not a copy of every default.
    assert yaml.safe_load(path.read_text()) == {"scans": {"dwell_time_seconds": 12.5}}


def test_changed_default_reaches_a_daemon_that_has_saved_before(tmp_path):
    """The reason settings.yaml holds only overrides: if it held everything,
    the first save would pin every value and editing the defaults file would
    silently do nothing."""
    defaults = tmp_path / "settings.defaults.yaml"
    defaults.write_text(DEFAULTS_PATH.read_text())
    path = tmp_path / "settings.yaml"
    save_settings(merge_settings(DEFAULTS, {"scans": {"dwell_time_seconds": 12.5}}), path)

    defaults.write_text(DEFAULTS_PATH.read_text().replace("num_beamswitches: 25", "num_beamswitches: 7"))
    loaded = load_settings(path)
    assert loaded.scans.num_beamswitches == 7        # new default arrives
    assert loaded.scans.dwell_time_seconds == 12.5   # override survives


def test_overrides_invert_merge():
    edited = merge_settings(DEFAULTS, {"lpr": {"pAzKvp": 0.6}, "webcam": {"max_width": 640}})
    assert settings_overrides(DEFAULTS, edited) == {"lpr": {"pAzKvp": 0.6}, "webcam": {"max_width": 640}}
    assert merge_settings(DEFAULTS, settings_overrides(DEFAULTS, edited)) == edited


def test_merge_is_deep():
    """A patch touching one field mustn't reset its siblings to defaults."""
    current = merge_settings(DEFAULTS, {"pointing": {"error_stable_cycles": 9}})
    new = merge_settings(current, {"pointing": {"move_timeout_seconds": 60}})
    assert new.pointing.error_stable_cycles == 9
    assert new.pointing.move_timeout_seconds == 60


@pytest.mark.parametrize(
    "patch",
    [
        {"scans": {"dwell_time_secs": 10}},                        # typo
        {"specan_live_view": {"rbw_hz": 1e6}},                        # old SpectrumConfig name
        {"pointing": {"error_stable_cycles": 0}},                  # out of range
        {"specan_live_view": {"start_hz": 2e9}},                      # start above stop
        {"specan_live_view": {"freq_mode": "center_span"}},           # pair not set
        {"radiometry": {"t_reciever_k": 95}},                      # typo in a folded-in model
    ],
)
def test_bad_patch_is_rejected_and_changes_nothing(patch):
    current = DEFAULTS
    snapshot = current.model_copy(deep=True)
    with pytest.raises(ValidationError):
        merge_settings(current, patch)
    assert current == snapshot


def test_moved_field_left_in_config_yaml_fails_loudly(tmp_path):
    """Pydantic ignores unknown keys by default, so without extra="forbid" a
    stale OBSERVATION_DWELL_TIME would load fine and silently do nothing."""
    stale = tmp_path / "config.yaml"
    stale.write_text(CONFIG.read_text() + "OBSERVATION_DWELL_TIME: 10\n")
    with pytest.raises(ValidationError, match="OBSERVATION_DWELL_TIME"):
        load_config(stale)


def test_center_hz_in_both_modes():
    start_stop = SpecanSettings(
        start_hz=1e9, stop_hz=2e9,
        resolution_bandwidth_hz=1e6, video_bandwidth_hz=1e5, reference_level_dbm=-30,
    )
    center_span = SpecanSettings(
        freq_mode="center_span", center_frequency_hz=1.42e9, span_hz=1e7,
        resolution_bandwidth_hz=1e6, video_bandwidth_hz=1e5, reference_level_dbm=-30,
    )
    assert start_stop.center_hz == 1.5e9
    assert center_span.center_hz == 1.42e9


def test_beamwidth_at_hydrogen_line():
    """Roughly 2.5 deg for a 6 m dish at 21 cm."""
    config = load_config(CONFIG)
    assert config.beamwidth_deg(1.4204e9) == pytest.approx(2.46, abs=0.01)


def test_driver_reconfigures_only_on_change():
    """set_settings touches no hardware, so this runs without an analyzer."""
    settings = DEFAULTS.specan_live_view
    driver = SiglentDriver("NO-SUCH-SERIAL", settings)

    driver.set_settings(settings.model_copy())
    assert not driver._reconfigure_flag.is_set()

    driver.set_settings(settings.model_copy(update={"num_averages": 10}))
    assert driver._reconfigure_flag.is_set()
    assert driver.get_settings().num_averages == 10


def test_lpr_halves_cover_lpr_exactly():
    """The runtime and static halves are reassembled into one LPR command;
    a field in neither, or in both, would be dropped or ambiguous."""
    tuning, encoder = set(LprTuning.model_fields), set(LprEncoderParams.model_fields)
    assert not tuning & encoder
    assert tuning | encoder == set(LprParams.param_order())


def test_lpr_tuning_rejects_gaps_typos_and_encoder_fields():
    with pytest.raises(ValidationError, match="pAzKvp"):
        merge_settings(DEFAULTS, {"lpr": {"pAzKvp": None}})
    with pytest.raises(ValidationError):
        merge_settings(DEFAULTS, {"lpr": {"pAzKvpp": 1.0}})  # typo
    with pytest.raises(ValidationError, match="pAzEpo"):
        merge_settings(DEFAULTS, {"lpr": {"pAzEpo": 0.0}})   # static, not runtime


def test_lpr_edit_is_staged_until_calibration():
    """The controller only takes LPR inside SPA/LPR/CLE, so rotor.lpr must
    keep reporting what's loaded until calibrate() runs."""
    from srt.daemon.rotor_control.testing_driver import TestingDriver

    config = load_config(CONFIG)
    loaded = LprParams.from_parts(DEFAULTS.lpr, config.MOTOR_ENCODER_PARAMS)
    assert loaded.is_loaded
    driver = TestingDriver(config.AZLIMITS, config.ELLIMITS, loaded)
    edited = merge_settings(DEFAULTS, {"lpr": {"pAzKvp": 0.6}}).lpr

    driver.set_lpr_params(LprParams.from_parts(edited, config.MOTOR_ENCODER_PARAMS))
    assert driver.get_state().lpr == loaded

    driver.calibrate()
    assert driver.get_state().lpr.pAzKvp == 0.6
