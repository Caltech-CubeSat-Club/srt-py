"""config_loader.py

Loads and validates config.yaml using the DaemonConfig Pydantic model
(see telescope_types.py), replacing yamale + schema.yaml. Also loads and
saves the runtime half, settings.yaml (see daemon/settings.py).

validate_yaml_schema() and load_yaml() are kept below, commented out,
for reference/rollback — they're superseded by load_config(), which
does both jobs (parse + validate) in one typed call and raises a
pydantic.ValidationError with a precise field path on any problem,
rather than yamale's validation-result-object pattern.
"""

import os
import tempfile
from pathlib import Path

import yaml

from .daemon.settings import RuntimeSettings, merge_settings, settings_overrides
from .daemon.telescope_types import DaemonConfig


def load_config(config_path: str | Path) -> DaemonConfig:
    """Parses and validates config.yaml in one step.

    Parameters
    ----------
    config_path : str | Path
        Path to the config.yaml file

    Returns
    -------
    DaemonConfig
        Fully validated, typed configuration object. Every field that
        was previously accessed via config_dict["KEY"] in
        SmallRadioTelescopeDaemon.__init__ is now config.KEY — see
        daemon_init_changes.py for the corresponding __init__ updates.

    Raises
    ------
    pydantic.ValidationError
        If config.yaml is missing a required field, has a value of
        the wrong type, fails the MOTOR_TYPE/freq_mode/etc. enum
        checks, or fails the AZLIMITS/ELLIMITS ordering check. The
        error message names the exact field path, e.g.
        "EMERGENCY_CONTACT.phone_number\\n  Field required".
    """
    config_path = Path(config_path)
    with open(config_path) as file:
        raw = yaml.safe_load(file)
    return DaemonConfig.model_validate(raw)


DEFAULTS_FILENAME = "settings.defaults.yaml"

_SETTINGS_HEADER = f"""\
# Written by the SRT daemon; changes made through the web app land here.
# Only what differs from {DEFAULTS_FILENAME} — delete a key to fall back to
# the default, or the file to reset everything. Safe to edit by hand while
# the daemon is stopped, but comments and key order are not preserved.
"""


def _defaults_path_for(settings_path: Path) -> Path:
    return settings_path.with_name(DEFAULTS_FILENAME)


def load_default_settings(defaults_path: str | Path) -> RuntimeSettings:
    """settings.defaults.yaml on its own. It has to be complete — it's the
    only place defaults live."""
    with open(defaults_path) as file:
        return RuntimeSettings.model_validate(yaml.safe_load(file))


def load_settings(settings_path: str | Path, defaults_path: str | Path | None = None) -> RuntimeSettings:
    """settings.yaml layered over settings.defaults.yaml — by default, the
    one beside it. A missing settings.yaml just means the defaults.

    Raises pydantic.ValidationError on a bad or unknown field, same as
    load_config.
    """
    settings_path = Path(settings_path)
    defaults = load_default_settings(defaults_path or _defaults_path_for(settings_path))
    if not settings_path.exists():
        return defaults
    with open(settings_path) as file:
        overrides = yaml.safe_load(file) or {}
    return merge_settings(defaults, overrides)


def save_settings(
    settings: RuntimeSettings,
    settings_path: str | Path,
    defaults_path: str | Path | None = None,
) -> None:
    """Writes settings.yaml — only the overrides — atomically: a crash or
    full disk mid-write leaves the old file intact rather than a truncated
    one the daemon can't boot from. The temp file shares the target's
    directory so os.replace is a rename, not a cross-device copy.
    """
    settings_path = Path(settings_path)
    defaults = load_default_settings(defaults_path or _defaults_path_for(settings_path))
    overrides = settings_overrides(defaults, settings)
    fd, tmp = tempfile.mkstemp(
        dir=settings_path.parent, prefix=f".{settings_path.name}.", suffix=".tmp"
    )
    try:
        with os.fdopen(fd, "w") as file:
            file.write(_SETTINGS_HEADER)
            yaml.safe_dump(overrides, file, sort_keys=False)
            file.flush()
            os.fsync(file.fileno())
        os.replace(tmp, settings_path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# --- Superseded by load_config() above — kept for reference ---------------
#
# import yamale
#
# def validate_yaml_schema(config_path, schema_path):
#     schema = yamale.make_schema(schema_path)
#     data = yamale.make_data(config_path)
#     return yamale.validate(schema, data)
#
# def load_yaml(config_path):
#     with open(config_path) as file:
#         config = yaml.load(file, Loader=yaml.FullLoader)
#         return config
#
# schema.yaml itself can be deleted once every caller of load_yaml() /
# validate_yaml_schema() has been migrated to load_config().