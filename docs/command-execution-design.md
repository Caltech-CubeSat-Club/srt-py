# Command execution design

This is all about how a command gets from a user (the web app) to the
telescope, how the daemon holds and schedules it, and how progress and
results get reported back.

Living document — update it as decisions change. See the
[README](../README.md) for how this fits into the rest of the software.
Mirrored to Notion (Telescope Control Software → Command execution design);
this file is the source of truth, so edit here and copy across.

---

# Status

## The baseline this replaces

`daemon.py` holds `self.command_queue`, a `Queue` of **strings**, and
`current_queue_item`, one string. `DaemonStatus` reports `queued_item: str`
and `queue_size: int`. Two threads: `update_command_queue` puts,
`srt_daemon_main` gets. That's thread-safe because it's a dumb FIFO of
immutable strings — the new structure below is touched by two threads too,
so it needs its own safe-access story.

None of the design below is wired into that loop yet. `srt_daemon_main`
still runs the string path exactly as it always has.

## Where things actually stand (as of 26 Sep 2026)

More of the type layer exists than you'd guess from a doc titled "design" —
most of it just isn't connected to anything yet.

| File | Status | Notes |
|---|---|---|
| `common.py` | Done | `Band`, `DriverKind`, `Resource`, `OutputFormat`, `CommandState`, `FrameRole` — all real. |
| `command_types.py` | Done | `CommandBase` and all fifteen concrete commands, the `TelescopeCommand` union, `ObservationPlan` — all real fields and validators, covered by `tests/test_command_encoding.py`. |
| `status.py` | Done | `DaemonStatus` (moved out of `telescope_types.py`), now carrying the runtime `settings`. `TICK_EXCLUDE` defined. |
| `settings.py` | Done | `RuntimeSettings` — the runtime-editable half of the config. Defaults in `config/settings.defaults.yaml` (checked in); the daemon's overrides in `config/settings.yaml`. Folds in `ObservingSettings` and `Radiometry`. See the README's "Configuration: static vs runtime". |
| `scheduling.py` | Partial | `PlanState`, `RotorMode`, `Span` are real. `Timeline.conflicts()`, `Timeline.row()`, and `Span.overlaps()` raise `NotImplementedError`. |
| `radio_control/driver.py` | Partial | `DriverCapabilities` and the `SpectrumDriver` contract are real, and it re-exports `SpecanSettings` / `RfsocSettings` (which live in `telescope_types.py` — see below). `SiglentDriver` and a new `RfsocDriver` both subclass the contract, but only `SiglentDriver.capabilities` is implemented; everything else raises `NotImplementedError`. |
| `observing/settings.py` | Done | `DataProcessingSettings`, `HotCalibrator`, `ObservingSettings`, `Radiometry` — all real models. |
| `observing/routines.py` | Stub | `resolve_spectrum_settings()` and `resolve_integration_counts()` are real. `expand()`, `span_for()`, `advance()`, `needs_calibration()` raise `NotImplementedError`, but the signatures and docstrings are settled. |
| `observing/validation.py` | Stub | All four functions raise `NotImplementedError` — see the open lead below. |
| `observing/radiometry.py` | Stub | Every function raises `NotImplementedError`. This is meant to **port** `scripts/combined_data_collection.py`'s existing Y-factor and T_sky math, not invent new math — see that script before writing any of these bodies. |
| `PlanOperation` | Partial | The six operation models exist in `command_types.py`, inside the `ClientRequest` envelope `/ws/command` accepts. The backend handles `submit` by queueing the plan's primitives as it always has; the other five are refused with a reason until the daemon has a scheduler to send them to. |
| `daemon.py` handler loop | Not started | `srt_daemon_main` is still the original string queue. None of the above is wired in. |

TypeScript generation already covers everything in the "Done" and "Partial"
rows (`Span`, `Timeline`, `SpectrumSettings`, `DataProcessingSettings`,
`ObservingSettings`, `Radiometry`, `DriverCapabilities`, `RuntimeSettings`
are all in `generate_ts_types.py`'s model list) — so the frontend types are
sitting there ready before the python bodies that fill them in exist.

One sign of how far ahead of the daemon this already is:
`zmq_bridge.commands.encode_command` already refuses to silently mis-encode
an observation command — it raises `CommandRejected` naming the command type
and saying why. It has nowhere to send it yet, because the daemon can't run
plans.

## The request envelope

Everything a browser sends on `/ws/command` is one Pydantic union,
`command_types.ClientRequest`, discriminated by `kind`:

| `kind` | Model | What it does |
|---|---|---|
| `command` | `CommandRequest` | Run one `TelescopeCommand` now, outside any plan |
| `settings` | `SettingsPatch` | A partial `RuntimeSettings`; nested dicts merge |
| `plan` | `PlanOperation` | `submit`, `cancel`, `insert`, `remove`, `replace`, `reorder` |

**Settings are state; commands are actions.** A settings patch changes
what the daemon persists (`RuntimeSettings`) and is never a plan step.
Changing the Siglent's live view — its free-running idle mode, Siglent-only and outside the driver contract — is just `{"specan_live_view": {...}}`.
Commands don't change settings as a side effect — each `Integrate` carries
the radio settings it uses, never whatever the last command left behind.
With plans sharing a timeline, "set the radio, then observe" would let
another plan's integration run in between. Where those settings come from
is under `command_types.py` below.

**The daemon's protocol is lines**: a verb, then arguments. Simple commands
take plain arguments (`azel 120.500000 45.000000`, `stow`); structured
payloads take one JSON argument (`update_settings {...}`, and `plan {...}`
once there's a scheduler to receive plan operations). Deliberately kept
typeable by hand.

`zmq_bridge/commands.py` gets a request there in three steps, in the order
they appear in the file:

| Step | Function | |
|---|---|---|
| check | `check(request, status)` | against the last `DaemonStatus`: mount limits, known objects, settings patches merged onto the current settings. Raises, so the browser gets the reason instead of `ok`. |
| encode | `encode(request)` | the daemon lines it becomes. A submitted plan is its commands in order, starting now, until the daemon can run plans — so a plan setting `start_time` or `max_duration_seconds` is refused rather than having them silently ignored. |
| send | `CommandListener.submit_request` | checks and encodes everything first, so a bad step 4 doesn't run steps 1–3, then pushes the lines over ZMQ 5556. |

The daemon re-validates what it receives. `EmergencyStop` skips all of this
and goes to the controller's own e-stop socket.

## Open lead: terrain occlusion and the non-reproducible calibration

> `observing/validation.py`'s docstring surfaces something worth checking
> before this gets built further. `HORIZON_POINTS` — the terrain obstruction
> profile already published to the frontend — is checked in zero code paths
> right now. CasA and CygA can sit above `ELLIMITS` and still be behind a
> ridge, which would make a "hot" calibration load actually a roughly 300 K
> mountain and make `T_receiver` come out wildly low. This lines up with the
> hot/cold calibration reproducibility problem in the README's "What we're
> building toward" (1.2.2) — worth checking against the existing bad
> calibration data before hunting for a subtler cause.

---

# Worked example

"Observe CassA for an hour at L band, starting 03:00."

The browser sends one command:

```
plan  start_time 03:00, max_duration 1h
      commands: [ ObserveObject(CassA, band=L, total_time=1h) ]
```

The daemon expands that into primitives and lays them on the timeline. Each
resource gets its own timeline — a **row** — and a primitive occupies one
stretch of one row:

| Row | Time | What |
|---|---|---|
| rotor | 03:00:00 – 03:00:38 | slew to CassA |
| rotor | 03:00:38 – 03:01:38 | track CassA — on source |
| pol_x | 03:00:38 – 03:01:38 | integrate |
| rotor | 03:01:38 – 03:01:45 | slew +5° off |
| rotor | 03:01:45 – 03:02:45 | track off-source — reference |
| pol_x | 03:01:45 – 03:02:45 | integrate |
| … | … | × 30 switches |

Both rows are busy throughout — position switching moves the dish, so the
rotor is claimed for the whole hour, not just the opening slew.

`advance()` then runs those primitives one at a time, ticking progress onto
each. Nothing runs the `ObserveObject` itself; it only existed to say what
the rows should contain.

---

# Reference: file by file

Everything below describes the target design. Cross-check against the
Status table above for what's actually real today.

## `common.py` — shared vocabulary

No internal imports, so everything else can use it freely. Six names, no
behavior:

- **`Band`** — `"L" | "S" | "C"`. Which receiver path an observation uses.
- **`DriverKind`** — `"specan" | "rfsoc"`. Which receiver is attached.
- **`Resource`** — `ROTOR | POL_X | POL_Y`. Independently schedulable
  hardware; each gets its own row in the timeline. Vocabulary rather than
  inventory — which exist depends on what's plugged in, and
  `DriverCapabilities.polarizations` says how many are real. Separate
  polarizations are what would let a survey ride along on the free one
  during someone else's observation — it claims no rotor row, so it doesn't
  conflict with the plan that does.
- **`OutputFormat`** — `raw_spectra`, `power_spectral_density`,
  `flux_density`, `brightness_temperature`, `stokes`. What the pipeline
  should produce.
- **`CommandState`** — `PENDING | RUNNING | DONE | ABORTED | FAILED`. One
  command's progress through its life; lives on the command itself, not a
  separate tracker.
- **`FrameRole`** — `"source" | "reference" | "calibration"`. What a frame's
  pointing meant; set on each `Integrate` and recorded in its frame's
  metadata. Y-factor reduction is impossible without it.

## `scheduling.py` — the timeline

Sits below `command_types.py` so a `Span` can be a plan's field without a
cycle — a span names its command by uuid, not by object.

- **`PlanState`** — `QUEUED | RUNNING | DONE | FAILED | CANCELLED`. No
  suspended state: a running plan keeps its resources until it finishes or
  hits `max_duration`, so nothing is ever paused half-way.
- **`RotorMode`** — `SLEW | TRACK | HOLD`, what the dish is doing during a
  rotor span. All three are claims; an occupied rotor row is never idleness.
  TRACK sets `ephemeris_cmd_location` and lets the ephemeris thread drive;
  HOLD sets `rotor_cmd_location` and clears it. `Wait` is a HOLD; a drift
  scan is slew-then-hold with the sky moving through a stationary beam.
- **`Span`** — one resource, one time range, one primitive, plus
  `rotor_mode` if applicable. Derived, never authored. Times are absolute so
  a span means the same thing after the plan is edited or reloaded.
- **`Timeline`** — every span currently claimed, across all plans.
  `conflicts()` returns what a candidate would double-book; `row()` returns
  one resource's spans in time order, which is what the UI draws.

`Timeline.conflicts()` is what makes submission safe. A plan occupying time
already claimed on a resource it needs is refused with a reason; a
rotor-only and a radio-only plan at the same instant is not a conflict.
Because the timeline is conflict-free by construction, a plan that reaches
its start time and can't acquire its resources means the previous occupant
overran — so it fails and is cancelled rather than waiting.

## `command_types.py` — commands and plans

**`CommandBase`** carries `uuid`, `state`, `detail`, `progress`. Everything
addresses commands by uuid: edits, aborts, progress, spans.

Three subclasses sit between `CommandBase` and the actual commands:

- **`PrimitiveCommand`** — occupies one span on one row. What `advance()`
  actually runs. Operators may author these directly — an observing file
  with manual pointings is a primitive sequence. Adds `from_command`, the
  uuid of the aggregate it was expanded from (unset if hand-written).
- **`AggregateCommand`** — a preset pattern that expands into primitives
  across several rows. `advance()` never sees one, only its expansion — so a
  new observing mode is a new pattern, not a new execution path. A pure
  marker: no fields, no methods.
- **`ObservingCommand`** — the commands that point at a band and produce
  spectra. Adds `band`, `frames`, and an optional `spectrum_settings`.
  Separate from `CommandBase` so `Stow` and `Wait` don't carry an empty
  frames list forever, including in the generated TypeScript.

**Radio settings for an observation.** `spectrum_settings` left unset means
the band's default (`ObservingSettings.spectrum_settings_per_band`); set, it
overrides that for this command — e.g. a narrow span on the HI line for one
source. Either way it's resolved at submission: `resolve_spectrum_settings`
fills in every unset one, and `expand()` copies the result onto each
`Integrate` it emits. So a queued plan keeps its settings if the band
defaults are edited, and its frames record exactly what was used.

**`Integrate`** is the one primitive that uses the receiver: whole driver
integrations wherever the dish points, one frame each, with a `role`
(source / reference / calibration) saying what that pointing meant. It runs
in whole integrations because that's all the driver can do — each lasts
`integration_duration(settings)`, for the Siglent `num_averages` sweeps.

Give either `integrations` (exactly that many frames) or `total_seconds` (a
minimum). Seconds become a count at submission, rounded up —
`resolve_integration_counts` — so afterwards every `Integrate` carries a
concrete count, and its duration, `integrations × integration_duration`, is
exact. How long *one* integration lasts is a settings choice, not a command
one: `num_averages × sweep_time_seconds` on the Siglent, accumulation length
on the RFSoC. Aggregates expand into `Integrate`s interleaved with pointing
commands, converting their times to counts the same way; operators can also
write them by hand between pointing steps.

Each frame records its measured `integration_seconds` — what the radiometer
equation needs, which may differ from what was asked for.

Both `PrimitiveCommand` and `AggregateCommand` are markers — the code that
acts on them is in `observing/routines.py`, kept out of this module so the
models don't drag in driver and observing imports.

The fifteen concrete commands then pick their bases:

| Base | Commands |
|---|---|
| `PrimitiveCommand` | `PointAtObject`, `PointAtAzEl`, `PointAtOffset`, `Stow`, `CalibrateEncoders`, `Wait`, `WaitUntil`, `EmergencyStop`, `SpectrumStart`, `SpectrumStop` |
| `PrimitiveCommand` + `ObservingCommand` | `Integrate` |
| `AggregateCommand` + `ObservingCommand` | `ObserveObject`, `ParkedScan`, `GridScan`, `HotColdTest` |

`TelescopeCommand` is the discriminated union of all fifteen. One union, not
one per subsystem — the interesting commands use both the rotor and the
radio, so splitting by subsystem would leave them homeless.

`ObservationPlan` holds three lists, plus `start_time` and
`max_duration_seconds`:

| List | Holds | Notes |
|---|---|---|
| `commands` | what was asked for | may include aggregates; the only one authored |
| `steps` | what it turned into | always primitives — what actually runs |
| `spans` | when each step runs | one per step, one resource each |

Only `commands` is authored; the rest are derived at submission and
re-derived on edit. Each step's `from_command` names the line somebody
wrote, so the UI can collapse two hundred integrations into "observe CassA,
1h."

`max_duration_seconds` is a hard cap, not a guess — the plan is aborted at
it. Without one, an overrun cascades: the plan it kills frees nothing, so it
keeps running and kills the next.

Plans survive a daemon restart with their state and cursor. A command that
was RUNNING when the daemon died comes back FAILED — dish position and
analyzer state are both unknown afterwards.

## `radio_control/driver.py` — the receiver contract

Lives outside `observing/` so the dependency runs one way: observing depends
on drivers, not the reverse.

- **`SpecanSettings`** / **`RfsocSettings`** — per-driver instrument
  settings, discriminated by `driver`. Defined in `telescope_types.py`
  rather than here, because `SpectrumFrame` embeds them and this package
  imports `telescope_types`; `driver.py` re-exports them. `SpecanSettings`
  replaced the old `SpectrumConfig`, which mixed instrument settings with
  plot-display preferences (dropped — they never reached the analyzer) and
  the instrument serial (now `DaemonConfig.SPECTRUM_ANALYZER_SERIAL`).
- **`DriverCapabilities`** — what a driver can actually produce, so an
  impossible output format (e.g. `stokes` on a single-polarization receiver)
  is rejected at validation rather than three hours into an observation.
- **`SpectrumDriver`** (ABC, generic over the driver's settings model) —
  `capabilities`, `integration_duration()`, `do_one_integration()`.
  `do_one_integration` does exactly one integration — no duration argument,
  the settings fix it — and is blocking by design: it runs in the resource's
  worker thread, not the daemon's command handler.
- **`SiglentDriver`** subclasses it, but only `capabilities` is real (one
  polarization, everything but `stokes`). Its working API is still
  `start`/`stop`/`get_latest`/`get_history`, a free-running live view you
  sample asynchronously, with no "do one integration and hand me the frame"
  call — that's real driver work, not a wrapper, because the
  acquisition thread owns the VISA session. `integration_duration` is
  blocked on `siglent_driver` sending `:SWE:TIME:AUTO ON`, which hands
  sweep time to the instrument and makes duration unknowable — it needs to
  be set from `sweep_time_seconds` instead.
- **`RfsocDriver`** (`rfsoc_driver.py`) — every member raises
  `NotImplementedError` until the board's interface is known. Its docstring
  lists what the design already expects of it: two polarizations
  integrating concurrently, band select via GPIO switches, duration from
  accumulation length.

**One receiver at a time.** For now that's the Siglent, which has to both
free-run the live view and take observation integrations — it can't do
both at once, so an integration takes over its acquisition loop. The plan
is for both polarizations to end up permanently on the RFSoC, at which
point the Siglent, and with it the live view, becomes optional.

## `observing/settings.py` — observing policy

What an operator changes between observations, as opposed to hardware facts
fixed at startup. Both `ObservingSettings` and `Radiometry` are fields of
`settings.RuntimeSettings`, so they persist and are editable at runtime like
the rest of the runtime config.

- **`DataProcessingSettings`** — what the pipeline should produce. Not
  independently valid: `stokes` needs two polarizations, which the Siglent
  can't provide — validate against the driver's declared capabilities, not
  in isolation.
- **`HotCalibrator`** — one entry in the preference-ordered calibrator list.
  Tried in order; the first genuinely observable source wins. "Observable"
  means clear of the terrain profile, not merely above `ELLIMITS` (see the
  open lead above).
- **`ObservingSettings`** — switching time, desired SNR, per-band spectrum
  settings, the calibrator list, cold-offset angle, calibration cadence.
  Plans should snapshot it at admission rather than reading it live, since
  their spans were computed from it.
- **`Radiometry`** — constants feeding the radiometer equation and Y-factor
  solution, taken from `scripts/combined_data_collection.py`. Dish diameter
  and beamwidth aren't in it: the diameter is hardware
  (`DaemonConfig.DISH_DIAMETER_M`) and the beamwidth depends on frequency
  (`DaemonConfig.beamwidth_deg()`).

## `observing/routines.py` — expand, lay out, run

Six functions, in the order they get called. Functions rather than methods
on the command models, so `command_types.py` stays free of driver and
observing imports.

**`resolve_spectrum_settings(plan, ...)`** gives every observing command
without its own `spectrum_settings` its band's default. Implemented.

**`resolve_integration_counts(plan, driver)`** turns every `Integrate` given
as `total_seconds` into a count, rounding up. Implemented.

**`expand(aggregate, ...) -> list[PrimitiveCommand]`** turns one aggregate
into the primitives it stands for. `ObserveObject` becomes alternating
track-on-source / `Integrate` / slew-off / track-off / `Integrate`;
`HotColdTest` is the same alternation against a calibrator; `ParkedScan` and
`GridScan` are slew-then-hold.

**`span_for(primitive, start, ...) -> Span`** places one primitive on its
row. Everything is deterministic except slew time: waits state their
durations, and an `Integrate` lasts `integrations × integration_duration`
(which is why it takes the driver).

**`advance(primitive, ...)`** starts or polls one primitive — dispatches
work to that resource's worker, or checks whether dispatched work finished.
Never blocks.

**`needs_calibration(plan, band, ...)`** decides whether a `HotColdTest` has
to go in ahead of an observation, either because the interval lapsed or
because the last calibration was in a different band. Keyed on band only,
for now: an observation that overrides `spectrum_settings` shares its band's
calibration, and the `HotColdTest` it triggers uses the band default — even
though a Y-factor at one RBW or reference level may not transfer.

Submission — the path a `submit` operation takes through `PlanOperation`
handling, before the plan is admitted:

| Step | Outcome |
|---|---|
| `validate_observation_plan(plan, ...)` | problems? reject |
| `resolve_spectrum_settings(plan, ...)` | every observing command's settings concrete |
| `resolve_integration_counts(plan, driver)` | every `Integrate` a count |
| `expand(cmd, ...)` for each aggregate | → `plan.steps` |
| `span_for(step, ...)` for each step | → `plan.spans` |
| `timeline.conflicts(plan.spans)` | double-booked? reject |
| `plan.state = QUEUED` | admitted; slot is held |

Expansion happens at submission, not run time, because the timeline needs
concrete spans before it can tell whether a plan double-books anything.

Execution — what the handler loop does each pass:

| Step | Outcome |
|---|---|
| acquire the plan's resources atomically | failed? cancel it |
| `advance(step, ...)` for each running step | dispatch or poll — never blocks |
| drain the worker result queue | append frames, update state |
| `now > plan.end_time` | abort |
| all steps DONE and frames on disk | drop the plan |

**Every row has a worker; the handler only sets intent and polls.** Rotor
needs no new threads — `update_ephemeris_location` and `update_rotor_status`
already work this way. Radio gets one new thread per polarization, so POL_X
and POL_Y can integrate simultaneously, which the resource model requires
and a blocking handler makes impossible. The worker owns
`do_one_integration` and writing the frame to `SAVE_DIRECTORY`, so neither
the instrument nor the disk stalls scheduling. Completed frames come back
through a queue rather than being appended by the worker directly, so the
plan set keeps its single-writer property.

## `observing/validation.py` — can this plan run

Answers "can this plan run" before committing the dish to it.

- **`is_clear_of_terrain(...)`** — above the interpolated `HORIZON_POINTS`
  profile at a given azimuth. Has to interpolate between sparse samples and
  wrap correctly across 0/360°. See the open lead above for why this matters
  more than it sounds like it should.
- **`will_source_be_in_sky_for_whole_observation(...)`** — samples a
  source's track over the observation and checks every sample clears terrain
  and mount limits. Use `EphemerisTracker.get_azimuth_elevation()` directly,
  not `get_all_azel_time()`'s cached grid — that grid has no resolution in
  the range that matters here. `astroplan` is a declared dependency that
  nothing currently imports and would give rise/set times directly; worth
  using rather than hand-rolling.
- **`select_hot_calibrator(...)`** — first candidate in preference order
  that stays observable for the whole calibration.
- **`validate_observation_plan(plan, ...)`** — returns a list of problems
  rather than a bool, because "no" alone is useless to an operator at 2 a.m.
  Checks: every tracked source stays observable for its step's duration;
  every commanded az/el is within mount limits and clear of terrain; the
  requested output format is producible by the chosen driver; every
  observing command has spectrum settings (its own, or its band's default); the plan's spans
  don't conflict with the timeline. It deliberately does **not** check that
  a plan looks like a sensible observation — a radio integration with no
  rotor span under it is allowed, since the operator may not care where the
  dish is pointing.

## `observing/radiometry.py` — the math

None of this is new science — `scripts/combined_data_collection.py` already
has the Y-factor solution, the T_sky SkyView lookup, and ADC-overload
rejection, as a standalone script that talks to the Siglent directly. This
module is meant to be that math **ported**, with the script then importing
from here, so the two implementations can't drift and disagree about
`T_sys`.

Six functions: `system_temperature_k` (the Y-factor solution),
`sky_temperature_k` (SkyView lookup — a network call, so it needs caching
before it's anywhere near a per-integration path), `integration_time_for_snr`
(radiometer equation solved for time), `effective_area_m2`,
`parallactic_angle_deg` (the beam's rotation relative to the sky over a long
az-el tracked integration — Saren's point 6 in the README's wishlist; has to
be evaluated and stored per integration, and `SpectrumFrame` currently has
nowhere to put it), and `accumulate` (combine frames, rejecting
ADC-overloaded traces the way the script does).

## `daemon.py::srt_daemon_main` — the handler loop

The target: scheduling handled by the existing command-handler thread,
keeping the plan set single-writer and lock-free.

1. drain the operations queue, applying edits
2. ask the timeline what spans are due
3. advance each — dispatch to its worker, or poll it
4. repeat

**The handler never blocks.** Two synchronization requirements, both
one-way: workers hand completed frames back through a queue that the handler
drains and appends (keeping the plan set single-writer), and the status
thread reads an immutable snapshot the handler publishes, rather than the
plan set itself.

## `PlanOperation` — plan edits from the web app

The models exist (`command_types.py`, one per operation, inside the
`ClientRequest` envelope above); the daemon side doesn't. The receiving
thread would enqueue operations; the handler applies them between polls,
keeping the single-writer, no-lock property that makes today's `Queue`
safe.

The protocol is *operations*, not "here is a plan":
`submit | cancel | insert | remove | replace | reorder`. Both whole-plan
submit and incremental edits exist — submit creates, edits modify.

**Editing and cancelling are different.** Only PENDING commands can be
edited; a RUNNING one is immutable until it reaches a boundary, because its
spans are already on the timeline and other plans booked around them. A plan
or command can be **cancelled** at any point, including mid-integration —
cancelling releases its claims rather than rewriting them. Edits targeting a
non-PENDING uuid are rejected with a reason, which is also how stale-copy
conflicts between two browser tabs get handled. Operations double as an
audit log and an undo stack.

Commands are addressed by uuid, never by index: `InsertCommand` takes the
uuid to insert `after` (none for the start), and `ReorderCommands` takes the
plan's PENDING uuids in their new order.

## `status.py` — what gets sent back to the browser

`DaemonStatus.plans` carries queued, running and finished plans. Since
commands hold their own state and results, this is both progress and
history — no separate log to keep in sync, and "what did the telescope do
last night" is answered by the same objects that described the work.

A plan is dropped once its frames reach `SAVE_DIRECTORY`. Not a count or an
age — "still here" means "not yet safely on disk."

`TICK_EXCLUDE` keeps frames off the 2 Hz broadcast (they live on the
commands that produced them; the web app fetches them when it wants to
plot). It's defined but not yet applied — `update_status` still calls a bare
`model_dump_json()`.

The old `ObservationEvent` model and everything that recorded it is gone.

---

# Special cases

**Maintenance** takes the whole instrument, not just the dish. It moves to
the maintenance position, holds the rotor, and stops the radio workers until
an operator explicitly releases it. Everything anchored during the lease
fails on arrival, so the operator taking it should be told what they're
about to cancel.

**Ad-hoc commands** are ordinary daemon commands sent to the existing queue,
not plan operations — pointing the dish, stowing, starting the analyzer, the
things an operator does by hand between observations. The consequence to
watch: they bypass the timeline, so an ad-hoc `stow` during a running plan
fights that plan for the rotor. Probably correct as an operator override,
but it means the daemon can be in a state the timeline doesn't describe.

**Surveys** are manually scheduled, like anything else. Automating "observe
whenever the instrument is free" needs preemption, which this model doesn't
have.

---

# Open questions

**Slew margin.** Slew time is the only non-deterministic term in an
expansion. Use a flat **60 s** for now, rather than computing it from
`pAzVmax` / `pAzAmax` — the dish's real settling behavior isn't measured
yet, and a fixed, obviously-conservative number beats an estimate that's
precisely wrong. When it is computed, use the LPR values actually loaded on
the controller (`RotorState.lpr`), not the possibly-pending `settings.lpr`.
Revisit with data.

**Where validation stops.** `validate_observation_plan` deliberately does
not check that a plan looks like a sensible observation, only that it's a
legal one. Whether that line is in the right place is still open. It also
checks a plan in isolation, but runnability depends on the timeline too.

---

# Still to build

Quick checklist — see the Status table above for detail on each:

- [ ] `Timeline.conflicts()`, `Timeline.row()`, `Span.overlaps()`
- [ ] `expand()`, `span_for()`, `advance()`, `needs_calibration()` in `observing/routines.py`
- [ ] everything in `observing/validation.py` and `observing/radiometry.py`
- [ ] `SiglentDriver.integration_duration()` and `do_one_integration()`, including setting sweep time from `sweep_time_seconds` instead of `:SWE:TIME:AUTO ON`
- [ ] `RfsocDriver`, once the board's interface is known
- [ ] the radio workers (one thread per polarization) and the result queue
- [ ] `PlanOperation` handling in the daemon — only `submit` works, by queueing primitives the old way
- [ ] the daemon handler loop rewrite — currently still executes text strings, nothing above is wired in
- [ ] the immutable snapshot the status thread reads instead of shared state
- [ ] wire `TICK_EXCLUDE` into `update_status`
- [ ] plans snapshot `settings.observing` at admission
