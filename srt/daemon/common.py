"""
Shared vocabulary. The bottom of the dependency graph — this module imports
nothing from the rest of the package, so everything else can import it
without anything pointing back.
"""

from enum import Enum
from typing import Literal

# Which receiver path an observation uses.
Band = Literal["L", "S", "C"]

# Which kind of receiver is attached.
DriverKind = Literal["specan", "rfsoc"]

# What a frame's pointing meant: which leg of a switching cycle. Y-factor
# reduction is impossible without it.
FrameRole = Literal["source", "reference", "calibration"]

class Resource(Enum):
    """Independently schedulable hardware — one timeline row each.

    Vocabulary, not inventory: which exist depends on what's attached, and
    `DriverCapabilities.polarizations` reports how many are real.
    """

    ROTOR = "rotor"
    POL_X = "pol_x"
    POL_Y = "pol_y"

# What data an observation should produce.
OutputFormat = Literal[
    "raw_spectra",
    "power_spectral_density",
    "flux_density",
    "brightness_temperature",
    "stokes",
]

class CommandState(Enum):
    """Execution status of a queued command. Lives on the command itself (command_types.CommandBase)."""

    PENDING = "pending"
    RUNNING = "running"
    DONE = "done"
    ABORTED = "aborted"
    FAILED = "failed"
