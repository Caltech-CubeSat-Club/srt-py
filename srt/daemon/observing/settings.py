"""
Observing policy: what to calibrate against, how often, what to produce.

Part of the runtime-editable half — folded into daemon/settings.py's
RuntimeSettings, alongside the knobs that moved out of DaemonConfig.

Instrument settings live in telescope_types (SpecanSettings), not here.
"""

from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, ConfigDict, Field

from ..common import Band, OutputFormat
from ..telescope_types import SpectrumSettings


class _Strict(BaseModel):
    # These are edited live through settings patches; a typo'd key should be
    # rejected, not silently dropped.
    model_config = ConfigDict(extra="forbid")


class DataProcessingSettings(_Strict):
    """What the pipeline should produce from the accumulated spectra.

    Not independently valid: "stokes" needs two polarizations, which the
    Siglent cannot provide (one trace per sweep). Validate against the
    driver's declared capabilities, not in isolation — see
    radio_control.driver.SpectrumDriver.capabilities.
    """

    output_format: OutputFormat = "raw_spectra"
    time_resolution_seconds: Optional[float] = None


class HotCalibrator(_Strict):
    """One entry in the preference-ordered calibrator list.

    Tried in order; the first genuinely observable source wins. "Observable"
    means clear of the terrain profile, not merely above ELLIMITS — a dish
    pointed at a mountain sees ~300 K and silently ruins T_receiver.
    """

    object_id: str
    min_elevation_deg: Optional[float] = None  # extra margin above terrain


class ObservingSettings(_Strict):
    """Runtime-editable observing configuration. Values, and the shape to
    fill in, are in config/settings.defaults.yaml."""

    switching_time_seconds: float = Field(
        gt=0, description="How long to dwell before switching source/reference."
    )
    desired_snr: float = Field(gt=0)

    spectrum_settings_per_band: dict[Band, SpectrumSettings]

    hot_calibrators: list[HotCalibrator]

    cold_offset_deg: float = Field(
        gt=0,
        description="Angular offset from source for the reference pointing.",
    )

    calibration_integration_seconds: float = Field(
        gt=0,
        description="Standard dwell for one calibration measurement.",
    )

    calibration_interval_seconds: float = Field(
        gt=0,
        description="""This interval should be a set fraction of the timescale on which receiver noise drifts. 
        Actual value pending the drift measurement that justifies it by Saren/Ruby.
        """,
    )


class Radiometry(_Strict):
    """Constants feeding the radiometer equation and Y-factor solution.

    Taken from scripts/combined_data_collection.py, minus dish diameter and
    beamwidth: the diameter is hardware (DaemonConfig.DISH_DIAMETER_M) and
    the beamwidth depends on frequency (DaemonConfig.beamwidth_deg()).
    """

    t_receiver_k: float
    aperture_efficiency: float
    polarization_correction_factor: float
