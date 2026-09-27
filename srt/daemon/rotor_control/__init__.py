"""rotor_control/__init__.py

Public API for the rotor control package.

Driver factory
--------------
Use make_driver() to get the right driver for the configured MOTOR_TYPE.
This is the only place that knows which driver class maps to which type string.

    from srt.daemon.rotor_control import make_driver
    driver = make_driver(
        motor_type=config.MOTOR_TYPE,
        port=config.MOTOR_PORT,
        baudrate=config.MOTOR_BAUDRATE,
        az_limits=config.AZLIMITS,      # Limit objects, not tuples
        el_limits=config.ELLIMITS,
        lpr_params=settings.lpr,        # runtime settings, not config
        safe_mode=False,
    )
    state: RotorState = driver.get_state()

"""

from ..telescope_types import Limit, LprParams
from .moore6m_driver import Moore6mDriver
from .testing_driver import TestingDriver


def make_driver(
    motor_type: str,
    port: str,
    baudrate: int,
    az_limits: Limit,
    el_limits: Limit,
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
    az_limits : Limit
        Lower and upper azimuth limits in degrees. Both drivers read
        .lower_bound/.upper_bound, so not a tuple.
    el_limits : Limit
        Lower and upper elevation limits in degrees.
    lpr_params : LprParams
        Servo loop parameters. Must be fully loaded (lpr_params.is_loaded).
        Pass RuntimeSettings.lpr.
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