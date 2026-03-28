"""rotors.py

Module for Managing Different Motor Objects

"""

from .motors import Caltech6m, H180Motor, NoMotor, PushRodMotor, Rot2Motor


def angle_within_range(angle, limits):
    lower_limit, upper_limit = limits
    if lower_limit <= upper_limit:
        return lower_limit <= angle <= upper_limit
    else:
        return not lower_limit < angle < upper_limit

class Rotor:
    """
    Class for Controlling Any Rotor Motor Through a Common Interface

    See Also
    --------
    motors.py
    """

    def __init__(self, motor_type, port, baudrate, az_limits, el_limits, motor_kwargs=None):
        """Initializes the Rotor with its Motor Object

        Parameters
        ----------
        motor_type : str
            Name of motor backend to use
        port : str
            Serial Port Identifier String for Communicating with the Motor
        az_limits : (float, float)
            Tuple of Lower and Upper Azimuth Limits
        el_limits : (float, float)
            Tuple of Lower and Upper Elevation Limits
        """
        motor_type_upper = str(motor_type).upper()
        motor_types = {
            "NONE": NoMotor,
            "ALFASPID": Rot2Motor,
            "H180MOUNT": H180Motor,
            "PUSHROD": PushRodMotor,
            "CALTECH6M": Caltech6m,
        }
        try:
            motor_class = motor_types[motor_type_upper]
        except KeyError as exc:
            raise ValueError(f"Unsupported MOTOR_TYPE: {motor_type}") from exc

        motor_kwargs = motor_kwargs or {}
        self.motor = motor_class(port, baudrate, az_limits, el_limits, **motor_kwargs)
        self.az_limits = az_limits
        self.el_limits = el_limits

    def is_motion_allowed(self):
        return not bool(getattr(self.motor, "safe_mode", False))

    def get_azimuth_elevation(self):
        """Latest Known Azimuth and Elevation

        Returns
        -------
        (float, float)
            Azimuth and Elevation Coordinate as a Tuple of Floats
        """
        return self.motor.status()

    def set_azimuth_elevation(self, az, el):
        """Sets the Azimuth and Elevation of the Motor

        Parameters
        ----------
        az : float
            Azimuth Coordinate to Point At
        el : float
            Elevation Coordinate to Point At

        Returns
        -------
        None
        """
        if self.angles_within_bounds(az, el):
            self.motor.point(az, el)
        else:
            raise ValueError("Angle Not Within Bounds")

    def angles_within_bounds(self, az, el):
        return angle_within_range(az, self.az_limits) and angle_within_range(
            el, self.el_limits
        )

    def get_recent_serial_communications(self, limit=12):
        if hasattr(self.motor, "get_recent_serial_communications"):
            return self.motor.get_recent_serial_communications(limit=limit)
        return []

    def get_recent_command_history(self, limit=12):
        if hasattr(self.motor, "get_recent_command_history"):
            return self.motor.get_recent_command_history(limit=limit)
        return []

    def get_fsm_status(self):
        if hasattr(self.motor, "get_fsm_status"):
            return self.motor.get_fsm_status()
        return {}

    def get_pointing_error(self):
        """Returns pointing error (azerr, elerr) in millidegrees when available."""
        if hasattr(self.motor, "azerr") and hasattr(self.motor, "elerr"):
            try:
                return float(self.motor.azerr), float(self.motor.elerr)
            except (TypeError, ValueError):
                return None
        return None

    def get_diagnostics(self):
        """Returns motor backend diagnostics when available."""
        keys = [
            "mode",
            "CalSts",
            "AzBrkOn",
            "ElBrkOn",
            "EmStopOn",
            "azerr",
            "elerr",
            "amp_currents",
        ]
        diagnostics = {}
        for key in keys:
            if hasattr(self.motor, key):
                diagnostics[key] = getattr(self.motor, key)
        fsm = self.get_fsm_status()
        if fsm:
            diagnostics["fsm_status"] = fsm
        return diagnostics
