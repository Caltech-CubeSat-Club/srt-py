"""
The runtime-editable half of the daemon's configuration: knobs an operator
changes mid-session without restarting.

The static half is telescope_types.DaemonConfig (config/config.yaml, edited
by hand). This half is config/settings.yaml, written by the daemon — a
program rewriting a file people maintain eventually eats their comments.

Kept a separate object from DaemonConfig rather than nested in it: the
config is pickled into the FastAPI process at startup, so a mutable half
inside it would go stale there without anyone noticing. The daemon owns
this; everyone else sees it through DaemonStatus.

No defaults here: they live in config/settings.defaults.yaml, which
settings.yaml is layered over (config_loader.load_settings). One place, so
they can't drift.

Sits between telescope_types and status in the dependency graph.
"""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, ConfigDict, Field

from .observing.settings import ObservingSettings, Radiometry
from .telescope_types import LprTuning, SpecanSettings


class _Strict(BaseModel):
    # A typo'd key in an update should be rejected, not silently dropped.
    model_config = ConfigDict(extra="forbid")


class PointingSettings(_Strict):
    """Read by the pointing loop every cycle, so edits apply immediately."""

    tracking_command_deadband_mdeg: float = Field(ge=0)
    error_threshold_mdeg: float = Field(ge=0)
    error_stable_cycles: int = Field(ge=1)
    move_timeout_seconds: float = Field(gt=0)
    # Nothing reads this yet.
    settle_time_seconds: float = Field(ge=0)
    stow_on_out_of_bounds: bool


class ScanSettings(_Strict):
    """The legacy n-point and beam-switch scans. Read per step, so an edit
    lands mid-scan."""

    dwell_time_seconds: float = Field(ge=0)
    num_beamswitches: int = Field(ge=1)


class WebcamSettings(_Strict):
    target_fps: float = Field(gt=0)
    jpeg_quality: int = Field(ge=1, le=100)
    max_width: int = Field(gt=0)


class RuntimeSettings(_Strict):
    pointing: PointingSettings
    scans: ScanSettings

    # The Siglent's free-running live view, outside any observation.
    # Siglent-only: it isn't part of the SpectrumDriver contract, and the
    # RFSoC may have no equivalent. Observations never read it — each
    # Integrate carries its own settings.
    specan_live_view: SpecanSettings

    webcam: WebcamSettings

    # Null in settings.defaults.yaml until someone configures observing — it
    # has fields with no sensible default, and plans can't be validated
    # without it.
    #
    # Plans should snapshot this at admission rather than reading it live:
    # their spans were computed from it.
    observing: Optional[ObservingSettings]
    radiometry: Radiometry

    # Servo gains and limits — LPR minus the encoder geometry, which stays in
    # config.yaml. The controller only takes LPR as part of the SPA/LPR/CLE
    # calibration sequence, so an edit is pending until the next encoder
    # calibration. rotor.lpr in DaemonStatus is what's loaded.
    lpr: LprTuning


def _deep_merge(base: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
    out = dict(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def _diff(base: dict[str, Any], new: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for key, value in new.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            nested = _diff(base[key], value)
            if nested:
                out[key] = nested
        elif base.get(key) != value:
            out[key] = value
    return out


def settings_overrides(defaults: RuntimeSettings, settings: RuntimeSettings) -> dict[str, Any]:
    """What `settings` changes relative to `defaults` — the inverse of
    merge_settings, so merge_settings(defaults, this) == settings.

    settings.yaml stores only this. Storing everything would pin every value
    the first time the daemon saved, and a later change to the defaults file
    would then silently do nothing.
    """
    return _diff(defaults.model_dump(mode="json"), settings.model_dump(mode="json"))


def merge_settings(current: RuntimeSettings, patch: dict[str, Any]) -> RuntimeSettings:
    """Apply a partial update, e.g. {"scans": {"dwell_time_seconds": 10}}.

    Nested dicts merge; anything else (lists included) replaces. Raises
    pydantic.ValidationError, leaving `current` untouched, if the result
    isn't valid.
    """
    merged = _deep_merge(current.model_dump(mode="json"), patch)
    return RuntimeSettings.model_validate(merged)
