"""
Radiometer-equation and Y-factor math.

None of this is new work. `scripts/combined_data_collection.py` already has
the Y-factor solution, the T_sky SkyView lookup and the ADC overload
rejection, as a standalone script that talks to the Siglent directly. Port
it here and have the script import from this module, or the two will drift
and disagree about T_sys.

STATUS: stubs.
"""

from __future__ import annotations

from typing import Optional

from ..telescope_types import SpectrumFrame
from .settings import Radiometry


def system_temperature_k(
    hot_frame: SpectrumFrame,
    cold_frame: SpectrumFrame,
    t_sky_hot_k: float,
    t_sky_cold_k: float,
    constants: Radiometry,
) -> float:
    """Y-factor solution for receiver temperature.

    Existing implementation is compute_yfactor_temperature_flux() in
    scripts/combined_data_collection.py:
        T_source = Y*T_sky2 + (Y-1)*T_receiver - T_sky1
    """
    raise NotImplementedError


def sky_temperature_k(
    azimuth_deg: float, elevation_deg: float, frequency_hz: float
) -> float:
    """Brightness temperature of blank sky at this pointing.

    The script uses a SkyView lookup against the 1420MHz (Bonn) survey.
    That's a network call, so it needs caching before it goes anywhere near
    a per-integration code path.
    """
    raise NotImplementedError


def integration_time_for_snr(
    target_snr: float,
    source_flux_jy: float,
    bandwidth_hz: float,
    t_sys_k: float,
    constants: Radiometry,
) -> float:
    """Radiometer equation, solved for time.

    This is get_pointing_time_for_SNR from the meeting notes, with the
    inputs made explicit: the caller resolves an object name to a flux and
    a T_sys before calling, rather than this reaching into the catalog.
    """
    raise NotImplementedError


def effective_area_m2(constants: Radiometry) -> float:
    """Aperture efficiency times geometric area."""
    raise NotImplementedError


def parallactic_angle_deg(
    azimuth_deg: float, elevation_deg: float, latitude_deg: float
) -> float:
    """Rotation of the linearly polarized beam on the source.

    An az-el mount rotates the feed relative to the sky as it tracks, so
    over a long integration a fixed source rotates through the beam's
    polarization axes. Saren's point 6: this has to be corrected in
    software, and until both polarizations are absolutely calibrated the
    resulting gain drift is indistinguishable from the source's flux
    actually changing.

    Needs to be evaluated and stored per integration, not per observation.
    SpectrumFrame currently has nowhere to put it.
    """
    raise NotImplementedError


def accumulate(
    frames: list[SpectrumFrame], weights: Optional[list[float]] = None
) -> SpectrumFrame:
    """Combine frames into one. Reject ADC-overloaded traces as the script
    does (Siglent error 606, MAX_BAD_TRACES_PER_SCAN)."""
    raise NotImplementedError
