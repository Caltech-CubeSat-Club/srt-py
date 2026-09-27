"""
One-shot migration for a config.yaml from before the static/runtime split.

    python scripts/migrate_settings.py config/config.yaml

Writes the runtime values it finds to settings.yaml next to it, then lists
the keys to delete from config.yaml by hand. It never rewrites config.yaml
itself — that file is hand-maintained, and yaml.safe_dump would eat its
comments. The daemon refuses to start until those keys are gone.
"""

import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from srt.config_loader import DEFAULTS_FILENAME, load_default_settings, save_settings  # noqa: E402
from srt.daemon.settings import merge_settings  # noqa: E402
from srt.daemon.telescope_types import LprEncoderParams  # noqa: E402

# old key -> (group, new field)
MOVED = {
    "OBSERVATION_DWELL_TIME": ("scans", "dwell_time_seconds"),
    "NUM_BEAMSWITCHES": ("scans", "num_beamswitches"),
    "SCAN_SETTLE_TIME": ("pointing", "settle_time_seconds"),
    "ROTOR_MOVE_TIMEOUT": ("pointing", "move_timeout_seconds"),
    "TRACKING_COMMAND_DEADBAND_MDEG": ("pointing", "tracking_command_deadband_mdeg"),
    "POINTING_ERROR_THRESHOLD_MDEG": ("pointing", "error_threshold_mdeg"),
    "POINTING_ERROR_STABLE_CYCLES": ("pointing", "error_stable_cycles"),
    "STOW_ON_OOB": ("pointing", "stow_on_out_of_bounds"),
    "WEBCAM_TARGET_FPS": ("webcam", "target_fps"),
    "WEBCAM_JPEG_QUALITY": ("webcam", "jpeg_quality"),
    "WEBCAM_MAX_WIDTH": ("webcam", "max_width"),
}

# SpectrumConfig name -> SpecanSettings name. The y-axis/x_units display
# preferences have no new home; instrument_serial goes to config.yaml.
SPECTRUM_RENAMES = {
    "freq_mode": "freq_mode",
    "start_hz": "start_hz",
    "stop_hz": "stop_hz",
    "center_hz": "center_frequency_hz",
    "span_hz": "span_hz",
    "rbw_hz": "resolution_bandwidth_hz",
    "vbw_hz": "video_bandwidth_hz",
    "ref_level_dbm": "reference_level_dbm",
    "atten_auto": "attenuation_auto",
    "atten_db": "attenuation_db",
    "preamp_on": "preamp_on",
    "trace_type": "trace_type",
    "num_averages": "num_averages",
}

DROPPED = {
    "BEAMWIDTH", "END_OBSERVATION_ON_OOB", "DASHBOARD_DOWNLOADS", "DASHBOARD_REFRESH_MS",
    "SPECTRUM_ANALYZER_START_HZ", "SPECTRUM_ANALYZER_STOP_HZ", "SPECTRUM_ANALYZER_RBW_HZ",
    "SPECTRUM_ANALYZER_VBW_HZ", "SPECTRUM_ANALYZER_REF_LEVEL_DBM",
}


def main(config_path: Path) -> None:
    raw = yaml.safe_load(config_path.read_text()) or {}
    settings_path = config_path.parent / "settings.yaml"
    if settings_path.exists():
        sys.exit(f"{settings_path} already exists; not overwriting it.")

    patch: dict = {}
    for old, (group, field) in MOVED.items():
        if raw.get(old) is not None:
            patch.setdefault(group, {})[field] = raw[old]
    spectrum = raw.get("SPECTRUM_ANALYZER") or {}
    live = {new: spectrum[old] for old, new in SPECTRUM_RENAMES.items() if old in spectrum}
    if live:
        patch["specan_live_view"] = live
    # LPR splits: encoder geometry stays in config.yaml, tuning moves.
    lpr = raw.get("MOTOR_LPR_PARAMS") or {}
    encoder_keys = set(LprEncoderParams.model_fields)
    if lpr:
        patch["lpr"] = {k: v for k, v in lpr.items() if k not in encoder_keys}

    defaults = load_default_settings(config_path.parent / DEFAULTS_FILENAME)
    save_settings(merge_settings(defaults, patch), settings_path)
    print(f"Wrote {settings_path}")

    stale = [
        k for k in raw
        if k in MOVED or k in DROPPED or k in ("SPECTRUM_ANALYZER", "MOTOR_LPR_PARAMS")
    ]
    print("\nNow edit config.yaml by hand:")
    for key in stale:
        print(f"  delete  {key}")
    if "DISH_DIAMETER_M" not in raw:
        print("  add     DISH_DIAMETER_M: 6.0      (replaces BEAMWIDTH)")
    if lpr and "MOTOR_ENCODER_PARAMS" not in raw:
        print("  add     MOTOR_ENCODER_PARAMS:   (from the old MOTOR_LPR_PARAMS)")
        for k in LprEncoderParams.model_fields:
            print(f"            {k}: {lpr.get(k)}")
    if "SPECTRUM_ANALYZER_SERIAL" not in raw:
        serial = spectrum.get("instrument_serial", "<analyzer serial>")
        print(f'  add     SPECTRUM_ANALYZER_SERIAL: "{serial}"')


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(Path(sys.argv[1]))
