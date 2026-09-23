"""
Covers the browser-command -> daemon-text translation in zmq_bridge.bridge.

This layer is worth testing because a mistake here is silent: the daemon
logs "Command Not Identified" into a status field and carries on, so a
button in the UI just quietly does nothing. The whitespace-in-object-id
case below was a real bug found this way.
"""

from datetime import datetime, timedelta, timezone
from typing import get_args

import pytest

from srt.daemon import command_types as ct
from srt.daemon.telescope_types import DaemonStatus
from srt.fastapi_backend.zmq_bridge import bridge
from srt.fastapi_backend.zmq_bridge.bridge import (
    CommandRejected,
    command_listener,
    encode_command,
)

# (command instance, exact text the daemon should receive).
# Verified against daemon.py's srt_daemon_main dispatch chain.
CASES = [
    (ct.PointAtObject(object_id="CassA"), "CassA"),
    (ct.FindObjectLocation(object_id="M17"), "object M17"),
    (ct.PointAtAzEl(azimuth=120.5, elevation=45.0), "azel 120.500000 45.000000"),
    (ct.PointAtOffset(azimuth_offset=-0.25, elevation_offset=0.5), "offset -0.250000 0.500000"),
    (ct.Wait(duration_seconds=5), "wait 5.000000"),
    (ct.WaitUntil(time=datetime(2026, 9, 22, 14, 30, tzinfo=timezone.utc)), "2026:265:14:30:00"),
    (ct.Stow(), "stow"),
    (ct.CalibrateEncoders(), "calibrate_encoders"),
    (ct.SpectrumStart(), "spectrum_start"),
    (ct.SpectrumStop(), "spectrum_stop"),
    (
        ct.SpectrumConfigCommand(start_hz=1.4e9, num_averages=100),
        "spectrum_config num_averages=100 start_hz=1400000000.0",
    ),
]


@pytest.mark.parametrize("cmd,expected", CASES, ids=lambda v: type(v).__name__ if hasattr(v, "command") else None)
def test_encoding(cmd, expected):
    assert encode_command(cmd) == expected


@pytest.mark.parametrize("cmd,_expected", CASES, ids=lambda v: None)
def test_encoding_survives_the_daemon_tokenizer(cmd, _expected):
    """The daemon does command.split(" ") and indexes the result, so a
    trailing space or embedded newline shifts every argument."""
    line = encode_command(cmd)
    assert line == line.strip()
    assert "\n" not in line
    assert "  " not in line


def test_every_command_type_is_covered():
    """Fails when someone adds a command to the union without adding it
    here — which is also the moment they'd forget encode_command."""
    union = get_args(ct.TelescopeCommand)[0]  # Annotated[Union[...], Field]
    members = set(get_args(union))
    covered = {type(cmd) for cmd, _ in CASES} | {ct.EmergencyStop}
    assert covered == members, f"uncovered: {members - covered}"


def test_emergency_stop_is_never_encoded():
    """It goes to the controller's separate e-stop socket. Encoding it
    would queue it behind whatever the daemon is currently blocked on."""
    with pytest.raises(CommandRejected):
        encode_command(ct.EmergencyStop())


def test_floats_are_never_exponential():
    """float() parses 1e-07 fine, but the daemon echoes the raw string
    into operator-facing logs."""
    line = encode_command(ct.PointAtOffset(azimuth_offset=1e-7, elevation_offset=-1e-7))
    for token in line.split()[1:]:
        assert "e" not in token.lower(), line
        float(token)  # must survive what the daemon does to it


def test_spectrum_config_omits_unset_fields():
    """Sending an unset field would clobber the driver's current value."""
    assert encode_command(ct.SpectrumConfigCommand(rbw_hz=1e6)) == "spectrum_config rbw_hz=1000000.0"


def test_spectrum_config_with_no_fields_is_rejected():
    with pytest.raises(CommandRejected):
        encode_command(ct.SpectrumConfigCommand())


@pytest.mark.parametrize(
    "when",
    [
        datetime(2026, 9, 22, 14, 30),                                       # naive -> read as UTC
        datetime(2026, 9, 22, 7, 30, tzinfo=timezone(timedelta(hours=-7))),  # aware -> converted
    ],
)
def test_wait_until_normalizes_to_utc(when):
    """The daemon compares against datetime.utcfromtimestamp()."""
    assert encode_command(ct.WaitUntil(time=when)) == "2026:265:14:30:00"


# --- pre-flight, which needs a DaemonStatus to check against -------------


@pytest.fixture
def status(monkeypatch):
    st = DaemonStatus(
        object_locs={"CassA": (10.0, 20.0), "M17": (30.0, 40.0)},
        el_limits=(15.0, 81.0),
        az_limits=(-89.0, 449.0),
    )
    monkeypatch.setattr(bridge.status_broadcaster, "_latest", st)
    return st


def test_object_id_with_a_space_is_rejected(status):
    """The daemon only matches a single whitespace token against its
    ephemeris table, so 'Cas A' can never resolve."""
    with pytest.raises(CommandRejected, match="whitespace"):
        command_listener._preflight(ct.PointAtObject(object_id="Cas A"))


def test_unknown_object_is_rejected(status):
    with pytest.raises(CommandRejected, match="unknown object"):
        command_listener._preflight(ct.PointAtObject(object_id="Andromeda"))


def test_known_object_passes(status):
    command_listener._preflight(ct.PointAtObject(object_id="CassA"))


@pytest.mark.parametrize("az,el", [(120.0, 5.0), (120.0, 89.0), (500.0, 45.0)])
def test_out_of_bounds_azel_is_rejected(status, az, el):
    with pytest.raises(CommandRejected, match="outside mount limits"):
        command_listener._preflight(ct.PointAtAzEl(azimuth=az, elevation=el))


def test_in_bounds_azel_passes(status):
    command_listener._preflight(ct.PointAtAzEl(azimuth=120.0, elevation=45.0))


def test_offset_is_not_bounds_checked(status):
    """Relative to wherever the dish lands when the command is dequeued,
    which isn't knowable here."""
    command_listener._preflight(ct.PointAtOffset(azimuth_offset=999.0, elevation_offset=999.0))


def test_preflight_is_skipped_before_the_first_status(monkeypatch):
    """Cold start: no status yet, so don't block the operator on a guess."""
    monkeypatch.setattr(bridge.status_broadcaster, "_latest", None)
    command_listener._preflight(ct.PointAtObject(object_id="whatever"))
