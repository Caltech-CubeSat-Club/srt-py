"""
The contract every receiver driver implements.

Lives here rather than in `observing/` so the dependency runs one way:
observing depends on drivers, not the reverse. The per-driver settings
models live in telescope_types, because SpectrumFrame embeds them and this
package imports that module; they're re-exported here.

An ABC rather than a Protocol because both implementations are ours — a missing method should
fail at instantiation, not three hours into an observation.

Implementations: SiglentDriver (siglent_driver.py) and RfsocDriver
(rfsoc_driver.py). Both subclass SpectrumDriver so the contract is enforced,
but only SiglentDriver.capabilities is real; the rest raise
NotImplementedError. The Siglent's existing start/stop/get_latest loop is
a free-running live view, not this contract — "integrate for N seconds,
give me one frame" is real driver work, not a wrapper.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from pydantic import BaseModel, Field

from ..common import DriverKind, OutputFormat
from ..telescope_types import (  # noqa: F401  (re-export)
    FrameMetadata,
    RfsocSettings,
    SpecanSettings,
    SpectrumFrame,
    SpectrumSettings,
)


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


SettingsT = TypeVar("SettingsT", SpecanSettings, RfsocSettings)


class SpectrumDriver(ABC, Generic[SettingsT]):
    """Everything an observation routine is allowed to ask of a receiver.

    Generic over the driver's own settings model, so SiglentDriver can take
    SpecanSettings without an isinstance check or an override that narrows
    the parameter type. Callers holding a driver of unknown kind pass the
    settings for its `capabilities.driver`.
    """

    @property
    @abstractmethod
    def capabilities(self) -> DriverCapabilities: ...

    @abstractmethod
    def integration_duration(self, settings: SettingsT) -> float:
        """How long one integration will take with these settings, in seconds.

        A computation, NOT a hardware query — integration time is chosen,
        not discovered. For the Siglent it's
        `num_averages * sweep_time_seconds`, both `SpecanSettings` fields.
        Implementations must not touch the instrument.

        A method only because the formula is per-backend; the RFSoC will use
        accumulation length, not sweeps.

        The unit everything above counts in: an `Integrate` of n
        integrations occupies n times this, and a switch period is
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
        settings: SettingsT,
        metadata: FrameMetadata,
    ) -> SpectrumFrame:
        """Do exactly one integration on the current pointing — as long as
        integration_duration(settings) says — and return its frame with
        `metadata` attached and integration_seconds measured.

        No duration argument: the settings fix it. An `Integrate` asking for
        n frames calls this n times.

        Blocking, by design — it runs in the resource's worker thread, not
        in the daemon's command handler.

        The caller supplies metadata because the driver knows none of it:
        pointing, whether that's on source or off, band, parallactic angle
        are facts about what the routine did. Attach it, don't invent it.
        """
