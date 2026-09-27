"""
Covers the browser-request -> daemon-line translation and pre-flight checks
in zmq_bridge.commands.

This layer is worth testing because a mistake here is silent: the daemon
logs "Command Not Identified" into a status field and carries on, so a
button in the UI just quietly does nothing. The whitespace-in-object-id
case below was a real bug found this way.
"""

import asyncio
from datetime import datetime, timedelta, timezone
from typing import get_args

import pytest
from pydantic import ValidationError

from pathlib import Path

from srt.config_loader import load_default_settings
from srt.daemon import command_types as ct
from srt.daemon.status import DaemonStatus
from srt.fastapi_backend.zmq_bridge import status as status_module
from srt.fastapi_backend.zmq_bridge.commands import (
    CommandRejected,
    check,
    check_command,
    command_listener,
    encode,
    encode_command,
)

REPO_ROOT = Path(__file__).resolve().parent.parent

# (command instance, exact text the daemon should receive).
# Verified against daemon.py's srt_daemon_main dispatch chain.
CASES = [
    (ct.PointAtObject(object_id="CassA"), "CassA"),
    (ct.PointAtAzEl(azimuth=120.5, elevation=45.0), "azel 120.500000 45.000000"),
    (ct.PointAtOffset(azimuth_offset=-0.25, elevation_offset=0.5), "offset -0.250000 0.500000"),
    (ct.Wait(duration_seconds=5), "wait 5.000000"),
    (ct.WaitUntil(time=datetime(2026, 9, 22, 14, 30, tzinfo=timezone.utc)), "2026:265:14:30:00"),
    (ct.Stow(), "stow"),
    (ct.CalibrateEncoders(), "calibrate_encoders"),
    (ct.SpectrumStart(), "spectrum_start"),
    (ct.SpectrumStop(), "spectrum_stop"),
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


# Observation commands, which the daemon can't run until it has a plan
# scheduler — they're aggregates to be expanded, not lines to execute.
NOT_YET_SENDABLE = [
    ct.Integrate(band="L", integrations=10),
    ct.ObserveObject(object_id="CassA", band="L", total_time_seconds=60),
    ct.ParkedScan(azimuth=120.0, elevation=45.0, band="L", total_time_seconds=60),
    ct.GridScan(
        center_object_id="CassA",
        ra_span_deg=1.0,
        dec_span_deg=1.0,
        resolution_deg=0.5,
        band="L",
    ),
    ct.HotColdTest(band="L"),
]


def test_every_command_type_is_covered():
    """Fails when someone adds a command to the union without deciding how it
    ships — which is also the moment they'd forget encode_command."""
    union = get_args(ct.TelescopeCommand)[0]  # Annotated[Union[...], Field]
    members = set(get_args(union))
    covered = (
        {type(cmd) for cmd, _ in CASES}
        | {ct.EmergencyStop}
        | {type(cmd) for cmd in NOT_YET_SENDABLE}
    )
    assert covered == members, f"uncovered: {members - covered}"


@pytest.mark.parametrize("cmd", NOT_YET_SENDABLE, ids=lambda c: type(c).__name__)
def test_observation_commands_are_refused_until_the_scheduler_exists(cmd):
    """Must reject rather than silently emit something the daemon would
    parse as a different command."""
    with pytest.raises(CommandRejected, match="scheduler"):
        encode_command(cmd)


def test_observation_commands_need_exactly_one_stop_condition():
    
    with pytest.raises(ValidationError):
        ct.ObserveObject(object_id="CassA", band="L")  # neither
    with pytest.raises(ValidationError):
        ct.ObserveObject(
            object_id="CassA", band="L", total_time_seconds=60, desired_snr=5
        )  # both


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
def status():
    return DaemonStatus(
        object_locs={"CassA": (10.0, 20.0), "M17": (30.0, 40.0)},
        el_limits=(15.0, 81.0),
        az_limits=(-89.0, 449.0),
        settings=load_default_settings(REPO_ROOT / "config" / "settings.defaults.yaml"),
    )


def test_object_id_with_a_space_is_rejected(status):
    """The daemon only matches a single whitespace token against its
    ephemeris table, so 'Cas A' can never resolve."""
    with pytest.raises(CommandRejected, match="whitespace"):
        check_command(ct.PointAtObject(object_id="Cas A"), status)


def test_unknown_object_is_rejected(status):
    with pytest.raises(CommandRejected, match="unknown object"):
        check_command(ct.PointAtObject(object_id="Andromeda"), status)


def test_known_object_passes(status):
    check_command(ct.PointAtObject(object_id="CassA"), status)


@pytest.mark.parametrize("az,el", [(120.0, 5.0), (120.0, 89.0), (500.0, 45.0)])
def test_out_of_bounds_azel_is_rejected(status, az, el):
    with pytest.raises(CommandRejected, match="outside mount limits"):
        check_command(ct.PointAtAzEl(azimuth=az, elevation=el), status)


def test_in_bounds_azel_passes(status):
    check_command(ct.PointAtAzEl(azimuth=120.0, elevation=45.0), status)


def test_offset_is_not_bounds_checked(status):
    """Relative to wherever the dish lands when the command is dequeued,
    which isn't knowable here."""
    check_command(ct.PointAtOffset(azimuth_offset=999.0, elevation_offset=999.0), status)


def test_check_is_skipped_before_the_first_status():
    """Cold start: no status yet, so don't block the operator on a guess."""
    check(ct.CommandRequest(command=ct.PointAtObject(object_id="whatever")), None)


# --- the request envelope ------------------------------------------------


def _plan(*commands):
    return ct.ObservationPlan(commands=list(commands))


def test_settings_patch_is_one_json_line():
    """One whitespace-free token after the verb, so the daemon's split(" ")
    can't cut it."""
    patch = ct.SettingsPatch(patch={"specan_live_view": {"start_hz": 1.4e9, "num_averages": 100}})
    assert encode(patch) == [
        'update_settings {"specan_live_view":{"start_hz":1400000000.0,"num_averages":100}}'
    ]


def test_settings_patch_that_wont_validate_is_rejected_up_front(status):
    """Checked against the current settings, so the browser hears about it
    instead of the daemon logging it after replying ok. Default stop_hz is
    1.9 GHz."""
    patch = ct.SettingsPatch(patch={"specan_live_view": {"start_hz": 2.5e9}})
    with pytest.raises(ValidationError, match="start_hz must be less than stop_hz"):
        check(patch, status)


def test_submitted_plan_is_its_commands_in_order():
    submit = ct.SubmitPlan(plan=_plan(ct.Stow(), ct.Wait(duration_seconds=5)))
    assert encode(submit) == ["stow", "wait 5.000000"]


@pytest.mark.parametrize("timing", [
    {"start_time": datetime(2026, 9, 27, 3, 0, tzinfo=timezone.utc)},
    {"max_duration_seconds": 3600},
])
def test_plan_timing_is_refused_rather_than_ignored(timing):
    """Nothing can honor it yet, so a plan for 03:00 would otherwise run
    immediately and uncapped."""
    plan = ct.ObservationPlan(commands=[ct.Stow()], **timing)
    with pytest.raises(CommandRejected, match="scheduler"):
        encode(ct.SubmitPlan(plan=plan))


def test_plan_operations_beyond_submit_are_refused_with_a_reason():
    """They have no daemon to go to yet; say so rather than dropping them."""
    with pytest.raises(CommandRejected, match="scheduler"):
        encode(ct.CancelPlan(plan_uuid=_plan(ct.Stow()).uuid))


def test_nothing_is_sent_if_any_plan_step_fails_the_check(status, monkeypatch):
    """A bad step 4 mustn't let steps 1-3 run. With no socket, reaching the
    send would raise DaemonUnreachable instead."""
    monkeypatch.setattr(status_module.status_broadcaster, "_latest", status)
    bad = _plan(ct.Stow(), ct.PointAtAzEl(azimuth=120.0, elevation=5.0))
    with pytest.raises(CommandRejected, match="outside mount limits"):
        asyncio.run(command_listener.submit_request(ct.SubmitPlan(plan=bad)))
