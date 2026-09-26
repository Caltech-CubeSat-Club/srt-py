"""
The contract every receiver driver implements, and each one's settings.

Lives here rather than in `observing/` so the dependency runs one way:
observing depends on drivers, not the reverse. An ABC rather than a
Protocol because both implementations are ours — a missing method should
fail at instantiation, not three hours into an observation.

NOTE: SiglentDriver doesn't implement this yet. It has
start/stop/get_latest/get_history — a free-running loop you sample
asynchronously — and no way to say "integrate for N seconds, give me one
frame". That's real driver work, not a wrapper.

STATUS: stubs.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Annotated, Literal, Optional, Union

from pydantic import BaseModel, Field

from ..common import DriverKind, OutputFormat
from ..telescope_types import FrameMetadata, SpectrumFrame


# --- per-driver settings -----------------------------------------------


class SpecanSettings(BaseModel):
    """Siglent spectrum analyzer.

    Replaces `telescope_types.SpectrumConfig`, but not as a rename — that
    model's 19 fields were three different things. Instrument settings are
    below; the y-axis and x_units fields were plot display preferences that
    never reach the analyzer and belong in UI state; `instrument_serial` is
    hardware identity, so static config.

    `SiglentDriver` reads the old model in 16 places and
    `SpectrumFrame.config` embeds it, so retiring it is a real pass.
    """

    driver: Literal["specan"] = "specan"

    # Frequency can be expressed either way; the daemon currently runs in
    # start_stop mode, so both pairs have to survive the migration.
    freq_mode: Literal["start_stop", "center_span"] = "start_stop"
    start_hz: Optional[float] = None
    stop_hz: Optional[float] = None
    center_frequency_hz: Optional[float] = None
    span_hz: Optional[float] = None

    resolution_bandwidth_hz: float
    video_bandwidth_hz: float
    reference_level_dbm: float
    num_averages: int = 300
    trace_type: Literal["clear_write", "average"] = "clear_write"

    attenuation_db: Optional[float] = None
    attenuation_auto: bool = True
    preamp_on: Optional[bool] = True

    sweep_time_seconds: Optional[float] = None
    number_of_points: Optional[int] = None


class RfsocSettings(BaseModel):
    """RFSoC. Extend when the board arrives.

    switch1/switch2 from the original sketch are band-select RF switches on
    the board's GPIOs, so they're not fields here — `band` is what a person
    chooses and the mapping belongs in the driver. Two ways to say one thing
    is two ways to disagree.

    The consequence that does reach the observing layer: throwing those
    switches changes the RF path and invalidates calibration.
    """

    driver: Literal["rfsoc"] = "rfsoc"

    attenuation_db: float
    # TODO(shaurya/danica): real field list once the interface is known.


SpectrumSettings = Annotated[
    Union[SpecanSettings, RfsocSettings],
    Field(discriminator="driver"),
]


# --- the contract --------------------------------------------------------


class DriverCapabilities(BaseModel):
    """What a driver can produce, so an impossible output format is rejected
    at validation rather than three hours in.

    A model and not part of the ABC because it has to be *sent* — a plan
    editor greys out "stokes" on a single-polarization receiver, and an ABC
    has nothing to serialize.

    `driver` overlaps the settings discriminator but answers a different
    question: settings say which receiver they're for, this says which is
    attached. The frame's config only exists after a sweep completes.
    """

    driver: DriverKind

    polarizations: int = Field(
        ge=1, description="1 for the Siglent's single trace; stokes needs 2."
    )
    supported_output_formats: list[OutputFormat]


class SpectrumDriver(ABC):
    """Everything an observation routine is allowed to ask of a receiver."""

    @property
    @abstractmethod
    def capabilities(self) -> DriverCapabilities: ...

    @abstractmethod
    def integration_duration(self, settings: SpectrumSettings) -> float:
        """How long one integration will take with these settings, in seconds.

        A computation, NOT a hardware query — integration time is chosen,
        not discovered. For the Siglent it's
        `num_averages * sweep_time_seconds`, both `SpecanSettings` fields.
        Implementations must not touch the instrument.

        A method only because the formula is per-backend; the RFSoC will use
        accumulation length, not sweeps.

        Sets the granularity above it: a switch period is
        `ceil(switching_time / this)` integrations.

        NEEDS A DRIVER FIX: `_configure_instrument` sends `:SWE:TIME:AUTO
        ON`, handing sweep time to the instrument and making duration
        unknowable. Set it from `sweep_time_seconds`. The analyzer enforces
        a floor per span/RBW/VBW and flags UNCAL below it — check that when
        settings are chosen, not per integration.
        """

    @abstractmethod
    def do_one_integration(
        self,
        duration_seconds: float,
        settings: SpectrumSettings,
        metadata: FrameMetadata,
    ) -> SpectrumFrame:
        """Integrate on the current pointing; return one frame with
        `metadata` attached.

        Blocking, by design — it runs in the resource's worker thread, not
        in the daemon's command handler.

        The caller supplies metadata because the driver knows none of it:
        pointing, whether that's on source or off, band, parallactic angle
        are facts about what the routine did. Attach it, don't invent it.
        """
