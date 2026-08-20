from __future__ import annotations

from datetime import datetime, timezone
from typing import Annotated, Literal, Union

from pydantic import BaseModel, Field, field_validator

class PointAtObject(BaseModel):
    """Point telescope at named astronomical object."""

    command: Literal["point_at_object"] = "point_at_object"
    object_id: str

class FindObjectLocation(BaseModel):
    """Find and point to location of named astronomical object."""

    command: Literal["find_object_location"] = "find_object_location"
    object_id: str

class PointAtAzEl(BaseModel):
    """Point telescope at absolute azimuth/elevation position."""

    command: Literal["point_at_azel"] = "point_at_azel"

    azimuth: float = Field(
        description="Target azimuth in degrees"
    )

    elevation: float = Field(
        description="Target elevation in degrees"
    )

class PointAtOffset(BaseModel):
    """Move to offset from current position."""

    command: Literal["point_at_offset"] = "point_at_offset"

    azimuth_offset: float = Field(
        description="Azimuth offset in degrees"
    )

    elevation_offset: float = Field(
        description="Elevation offset in degrees"
    )

class SpectrumConfig(BaseModel):
    """Configure spectrum analyzer settings."""

    command: Literal["spectrum_config"] = "spectrum_config"
    
    start_hz: float | None = Field(default=None, gt=0)
    stop_hz: float | None = Field(default=None, gt=0)

    center_hz: float | None = Field(default=None, gt=0)
    span_hz: float | None = Field(default=None, gt=0)

    rbw_hz: float | None = Field(default=None, gt=0)
    vbw_hz: float | None = Field(default=None, gt=0)

    ref_level_dbm: float | None = None

    num_averages: int | None = Field(default=None, ge=1)

class Wait(BaseModel):
    """Wait for a specified duration."""

    command: Literal["wait"] = "wait"
    
    duration_seconds: float = Field(
        gt=0,
        description="Duration in seconds"
    )

class WaitUntil(BaseModel):
    """Wait until a specific absolute time."""
    command: Literal["wait_until"] = "wait_until"

    time: datetime = Field(
        description="Absolute date/time to wait until. Naive values are read as UTC."
    )

    @field_validator("time")
    @classmethod
    def _to_utc(cls, v: datetime) -> datetime:
        return v.replace(tzinfo=timezone.utc) if v.tzinfo is None else v.astimezone(timezone.utc)

class Stow(BaseModel):
    """Move telescope to stow position."""

    command: Literal["stow"] = "stow"

class CalibrateEncoders(BaseModel):
    """Run telescope encoder calibration."""

    command: Literal["calibrate_encoders"] = "calibrate_encoders"

class SpectrumStart(BaseModel):
    """Start spectrum analyzer."""

    command: Literal["spectrum_start"] = "spectrum_start"

class SpectrumStop(BaseModel):
    """Stop spectrum analyzer."""

    command: Literal["spectrum_stop"] = "spectrum_stop"

class EmergencyStop(BaseModel):
    """Emergency stop telescope."""

    command: Literal["emergency_stop"] = "emergency_stop"

# Union of all telescope commands

TelescopeCommand = Annotated[
    Union[
        PointAtObject,
        FindObjectLocation,
        PointAtAzEl,
        PointAtOffset,
        SpectrumConfig,
        Wait,
        WaitUntil,
        Stow,
        CalibrateEncoders,
        SpectrumStart,
        SpectrumStop,
        EmergencyStop,
    ],
    Field(discriminator="command")
]

# Observation plan
class ObservationPlan(BaseModel):
    """Observation plan for telescope."""

    name: str | None = None

    commands: list[TelescopeCommand] = Field(
        min_length=1,
        description="List of telescope commands to execute"
    )