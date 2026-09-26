"""
Observing policy: what to calibrate against, how often, what to produce.

The runtime-editable half — what an operator changes between observations,
as opposed to hardware facts fixed at startup. The runtime-ish fields still
in DaemonConfig belong here too; see CLAUDE.md.

Instrument settings live in radio_control/driver.py, not here.
"""

from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field

from ..common import Band, OutputFormat
from ..radio_control.driver import SpectrumSettings


class DataProcessingSettings(BaseModel):
    """What the pipeline should produce from the accumulated spectra.

    Not independently valid: "stokes" needs two polarizations, which the
    Siglent cannot provide (one trace per sweep). Validate against the
    driver's declared capabilities, not in isolation — see
    radio_control.driver.SpectrumDriver.capabilities.
    """

    output_format: OutputFormat = "raw_spectra"
    time_resolution_seconds: Optional[float] = None


class HotCalibrator(BaseModel):
    """One entry in the preference-ordered calibrator list.

    Tried in order; the first genuinely observable source wins. "Observable"
    means clear of the terrain profile, not merely above ELLIMITS — a dish
    pointed at a mountain sees ~300 K and silently ruins T_receiver.
    """

    object_id: str
    min_elevation_deg: Optional[float] = None  # extra margin above terrain


class ObservingSettings(BaseModel):
    """Runtime-editable observing configuration."""

    switching_time_seconds: float = Field(
        gt=0, description="How long to dwell before switching source/reference."
    )
    desired_snr: float = Field(gt=0)

    spectrum_settings_per_band: dict[Band, SpectrumSettings] = Field(default_factory=dict)

    hot_calibrators: list[HotCalibrator] = Field(default_factory=list)

    cold_offset_deg: float = Field(
        default=5.0,
        gt=0,
        description="Angular offset from source for the reference pointing.",
    )

    calibration_integration_seconds: float = Field(
        default=30.0,
        gt=0,
        description="Standard dwell for one calibration measurement.",
    )

    calibration_interval_seconds: float = Field(
        default=360.0, 
        gt=0, 
        description="""This interval should be a set fraction of the timescale on which receiver noise drifts. 
        Actual value pending the drift measurement that justifies it by Saren/Ruby.
        """,
    )


class Radiometry(BaseModel):
    """Constants feeding the radiometer equation and Y-factor solution.

    Taken from scripts/combined_data_collection.py.
    """

    t_receiver_k: float = 80.0
    dish_diameter_m: float = 6.0
    aperture_efficiency: float = 0.7
    polarization_correction_factor: float = 2.0
    beam_width_deg: float = 3.0
