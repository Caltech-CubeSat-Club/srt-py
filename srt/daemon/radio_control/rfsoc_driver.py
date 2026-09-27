"""rfsoc_driver.py

RFSoC receiver driver. A stub until the board arrives: it exists so the
SpectrumDriver contract has its second implementation to be checked against,
and so there's an obvious place to start.

TODO(shaurya/danica): everything below, once the interface is known. Fill
in RfsocSettings (telescope_types.py) alongside it.
"""

from __future__ import annotations

from ..telescope_types import FrameMetadata, RfsocSettings, SpectrumFrame
from .driver import DriverCapabilities, SpectrumDriver


class RfsocDriver(SpectrumDriver[RfsocSettings]):
    """Things to settle while writing this, from the observing design:

    - Two polarizations, integrating at the same time. The scheduler gives
      POL_X and POL_Y a worker thread each, so do_one_integration must be
      safe to call from both concurrently — or say it isn't and the
      scheduler serializes them.
    - Band select is GPIO RF switches on the board, chosen from `band` in the
      frame metadata, not a settings field. Switching changes the RF path and
      invalidates calibration; the observing layer needs to hear about it.
    - Integration duration comes from accumulation length, not sweeps.
    """

    @property
    def capabilities(self) -> DriverCapabilities:
        # Presumably polarizations=2 with every output format including
        # stokes, but that's a claim about hardware not yet seen.
        raise NotImplementedError

    def integration_duration(self, settings: RfsocSettings) -> float:
        """Computed from accumulation length and sample rate; must not touch
        the board."""
        raise NotImplementedError

    def do_one_integration(
        self,
        settings: RfsocSettings,
        metadata: FrameMetadata,
    ) -> SpectrumFrame:
        raise NotImplementedError
