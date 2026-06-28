"""config_loader.py

Loads and validates config.yaml using the DaemonConfig Pydantic model
(see telescope_types.py), replacing yamale + schema.yaml.

validate_yaml_schema() and load_yaml() are kept below, commented out,
for reference/rollback — they're superseded by load_config(), which
does both jobs (parse + validate) in one typed call and raises a
pydantic.ValidationError with a precise field path on any problem,
rather than yamale's validation-result-object pattern.
"""

import yaml
from pathlib import Path

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