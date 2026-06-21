"""rotor_control/__init__.py

Public API for the rotor control package.

Driver factory
--------------
Use make_driver() to get the right driver for the configured MOTOR_TYPE.
This is the only place that knows which driver class maps to which type string.

    from srt.daemon.rotor_control import make_driver
    driver = make_driver(
        motor_type="MOORE6M",
        port="COM3",
        baudrate=115200,
        az_limits=(-89, 449),
        el_limits=(15, 81),
        lpr_params=LprParams.from_dict(config_dict["MOTOR_LPR_PARAMS"]),
        safe_mode=False,
    )
    state: RotorState = driver.get_state()

"""

from ..types import LprParams
from .moore6m_driver import Moore6mDriver
from .testing_driver import TestingDriver


def make_driver(
    motor_type: str,
    port: str,
    baudrate: int,
    az_limits,
    el_limits,
    lpr_params: LprParams,
    safe_mode: bool = False,
):
    """Instantiate the correct driver for *motor_type*.

    Parameters
    ----------
    motor_type : str
        One of "MOORE6M", "CALTECH6M" (both map to Moore6mDriver),
        or "NONE" (maps to TestingDriver).
    port : str
        Serial port identifier, e.g. "COM3" or "/dev/ttyUSB0".
        Ignored for TestingDriver.
    baudrate : int
        Serial baudrate. Ignored for TestingDriver.
    az_limits : (float, float)
        Lower and upper azimuth limits in degrees.
    el_limits : (float, float)
        Lower and upper elevation limits in degrees.
    lpr_params : LprParams
        Servo loop parameters. Must be fully loaded (lpr_params.is_loaded).
        Pass LprParams.from_dict(config_dict["MOTOR_LPR_PARAMS"]).
    safe_mode : bool
        If True, motion commands are blocked on startup.

    Returns
    -------
    Moore6mDriver | TestingDriver
    """
    t = str(motor_type).upper()
    if t in ("MOORE6M", "CALTECH6M"):
        return Moore6mDriver(
            port=port,
            baudrate=baudrate,
            az_limits=az_limits,
            el_limits=el_limits,
            lpr_params=lpr_params,
            safe_mode=safe_mode,
        )
    if t == "NONE":
        return TestingDriver(
            az_limits=az_limits,
            el_limits=el_limits,
            lpr_params=lpr_params,
        )
    raise ValueError(
        f"Unsupported MOTOR_TYPE: {motor_type!r}. "
        "Expected one of: MOORE6M, CALTECH6M, NONE."
    )


__all__ = [
    "make_driver",
    "Moore6mDriver",
    "TestingDriver",
]