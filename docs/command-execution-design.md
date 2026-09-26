# Command execution design

This is all about how a command gets from a user (the web app) to the telescope, how the daemon holds and
schedules it, and how progress and results get reported back.

**Status:** design drafted, nothing implemented — every body raises
`NotImplementedError`. Living document; update it as decisions change.

---

# What exists today

`daemon.py` holds `self.command_queue`, a `Queue` of **strings**, and
`current_queue_item`, one string. `DaemonStatus` reports `queued_item: str`
and `queue_size: int`. Two threads: `update_command_queue` puts,
`srt_daemon_main` gets.

That's thread-safe because it's a dumb FIFO of immutable strings. The new structure will be touched by two threads, so we have to write safe access around it.

Below is an overview of the new structure I'm proposing. None of it has actually been implemented yet.

---

# `common.py` — shared types

No internal imports, so everything else can use it freely. Five aliases and
enums, no behaviour:

- **`Band`** — `"L" | "S" | "C"`. Which receiver path an observation uses.
- **`OutputFormat`** — `raw_spectra`, `power_spectral_density`,
  `flux_density`, `brightness_temperature`, `stokes`. What the pipeline
  should produce.
- **`CommandState`** — `PENDING | RUNNING | DONE | ABORTED | FAILED`. One
  command's progress through its life.
- **`DriverKind`** — `"specan" | "rfsoc"`. Which receiver is attached.
- **`Resource`** — `ROTOR | POL_X | POL_Y`. Independently schedulable
  hardware; each gets its own **row** in the **timeline**. Vocabulary rather
  than inventory: which exist depends on what's plugged in, and
  `DriverCapabilities.polarizations` says how many are real.

Separate polarizations are what would let a survey ride along on the free
one during someone else's observation — it claims no rotor row, so it
doesn't conflict with the plan that does.

---

# `scheduling.py` — maintains the timeline of what hardware gets used when

- **`PlanState`** — `QUEUED | RUNNING | DONE | FAILED | CANCELLED`. No
  suspended state: a running plan keeps its resources until it finishes or
  hits `max_duration`, so nothing is ever paused half-way.
- **`RotorMode`** — `SLEW | TRACK | HOLD`, what the dish is doing during a
  rotor span. All three are claims; an occupied rotor row is never idleness.
  They're separate because they're separate daemon state — TRACK sets
  `ephemeris_cmd_location` and lets the ephemeris thread drive, HOLD sets
  `rotor_cmd_location` and clears it. `Wait` is a HOLD; a drift scan is
  slew-then-hold with the sky moving through a stationary beam.
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
its start time and *can't* acquire its resources means the previous occupant
overran — so it fails and is cancelled rather than waiting.

---

# `command_types.py` — commands and plans

**`CommandBase`** — a base class for all commands I added underneath
Danica's framework. Carries `uuid`, `state`, `detail`, `progress`.
Everything addresses commands by uuid: edits, aborts, progress, spans.

I'm thinking status should live on the command because the web app has to
display it, as opposed to some separate object the daemon keeps to itself.

Three subclasses sit between `CommandBase` and the actual commands:

- **`PrimitiveCommand`** — occupies one span on one row. What `advance()`
  actually runs. Operators may author these directly; Saren's "observing
  file with manual pointings" is a primitive sequence. Adds `from_command`,
  the uuid of the aggregate it was expanded from (unset if hand-written).
- **`AggregateCommand`** — a preset pattern that expands into primitives
  across several rows. `advance()` never sees one, only its expansion, so a
  new observing mode is a new pattern rather than a new execution path.
- **`ObservingCommand`** — the ones that return observation data -- spectra. Adds `band` and `frames`.

Neither has any methods. `AggregateCommand` is entirely empty;
`PrimitiveCommand` adds only `from_command`. They exist to tag which
treatment a command gets — the code that acts on them is in
`observing/routines.py`, kept out of this file so the models don't drag in
driver and observing imports.

The sixteen concrete commands then pick their bases:

```
PrimitiveCommand   PointAtObject, FindObjectLocation, PointAtAzEl,
                   PointAtOffset, Stow, CalibrateEncoders, Wait, WaitUntil,
                   EmergencyStop, SpectrumConfigCommand, SpectrumStart,
                   SpectrumStop
Aggregate +        ObserveObject, ParkedScan, GridScan, HotColdTest
Observing
```

**`TelescopeCommand`** is the discriminated union of all sixteen. One union,
not one per subsystem — the interesting commands use both the rotor and the
radio, so splitting by subsystem would leave them homeless.

**`ObservationPlan`** holds three lists (plus `start_time` and
`max_duration`):

```
commands   what was asked for     may include aggregates
steps      what it turned into    always primitives; what runs
spans      when each step runs    one per step
```

Only `commands` is authored; the rest are derived at submission and
re-derived on edit. Each step's `from_command` names the line somebody
wrote, so the UI can collapse two hundred integrations into "observe CassA,
1h".

`max_duration` is a hard cap and the plan is aborted at it. Without one an
overrun cascades — the plan it kills frees nothing, so it keeps running and
kills the next.

Plans survive a daemon restart with their state and cursor. A command that
was RUNNING when the daemon died comes back FAILED: dish position and
analyzer state are both unknown afterwards.

---

# `observing/routines.py` — helper functions to expand, lay out, and run observation plans

Four functions, in the order they get called.

**`expand(aggregate, ...) -> list[PrimitiveCommand]`** turns one aggregate
into the primitives it stands for. `ObserveObject` becomes alternating
track-on-source / integrate / slew-off / track-off / integrate;
`HotColdTest` is the same alternation against a calibrator; `ParkedScan` and
`GridScan` are slew-then-hold.

**`span_for(primitive, start, ...) -> Span`** places one primitive on its
row. Everything is deterministic except slew time: `integration_duration` is
`num_averages * sweep_time_seconds`, both chosen, and waits are stated.

**`advance(primitive, ...)`** runs one primitive — dispatches work to that
resource's worker, or polls work already dispatched. Never blocks.

**`needs_calibration(plan, band, ...)`** decides whether a `HotColdTest`
has to go in ahead of an observation, either because the interval lapsed or
because the last calibration was in a different band.

**Submission** — this is the path a `submit` operation takes through
`PlanOperation` handling in the daemon, before the plan is admitted:

```
validate_observation_plan(plan, ...)   -> problems? reject
expand(cmd, ...) for each aggregate    -> plan.steps
span_for(step, ...) for each step      -> plan.spans
timeline.conflicts(plan.spans)         -> double-booked? reject
plan.state = QUEUED                       admitted; slot is held
```

Expansion has to happen at submission rather than at run time, because the
timeline needs concrete spans before it can tell whether the plan
double-books anything.

**Execution** — what the handler loop does each pass:

```
acquire the plan's resources atomically  -> failed? cancel it
advance(step, ...) for each running step -> dispatch or poll
drain the worker result queue            -> append frames, update state
now > plan.end_time                      -> abort
all steps DONE and frames on disk        -> drop the plan
```

---

# `daemon.py::srt_daemon_main` — the handler loop

We'll have scheduling handled by the existing command-handler thread, which
keeps the plan set single-writer and lock-free:

1. drain the operations queue, applying edits
2. ask the timeline what spans are due
3. advance each — dispatch to its worker, or poll it
4. repeat

**The handler never blocks.** Every resource row has its own worker.

- **Rotor.** No new threads: `update_ephemeris_location` and
  `update_rotor_status` already work this way, and `advance()` just sets
  `ephemeris_cmd_location` or `rotor_cmd_location`.
- **Radio.** One new thread per polarization — so two on a dual-pol
  receiver, one today with the Siglent. That's what lets POL_X and POL_Y
  integrate simultaneously, which the resource model requires and a blocking
  handler makes impossible. The worker owns `do_one_integration` *and*
  writing the frame to `SAVE_DIRECTORY`, so neither the instrument nor the
  disk stalls scheduling.

Two synchronisation requirements, both one-way:

- workers hand completed frames back through a queue; the handler drains it
  and appends, so the plan set stays single-writer
- the status thread reads an immutable snapshot the handler publishes, not
  the plan set itself

---

# `PlanOperation` — new daemon commands to tell it how/what to do with observation plans coming from the web app

Not yet written. The receiving thread enqueues operations; the handler
applies them between polls. Single writer, no lock — the property that makes
today's `Queue` safe, kept.

So the protocol is *operations*, not "here is a plan":
`submit | cancel | insert | remove | replace | reorder`. Both whole-plan
submit and incremental edits exist — submit creates, edits modify.

**Editing and cancelling are different.** Only PENDING commands can be
*edited*; a RUNNING one is immutable until it reaches a boundary, because
its spans are already on the timeline and other plans booked around them.
But a plan or command can be **cancelled at any point**, including
mid-integration — cancelling releases its claims rather than rewriting them.

Edits targeting a non-PENDING uuid are rejected with a reason, which is also
how stale-copy conflicts between two browser tabs get handled.

Operations double as an audit log and an undo stack.

---

# `status.py` — what gets sent back to the browser

**`DaemonStatus.plans`** carries queued, running and finished plans. Since
commands hold their own state and results, this is both progress and
history — there's no separate log to keep in sync, and "what did the
telescope do last night" is answered by the same objects that described the
work.

A plan is dropped once its frames reach `SAVE_DIRECTORY`. Not a count or an
age: "still here" means "not yet safely on disk".

**`TICK_EXCLUDE`** keeps frames off the 2 Hz broadcast. They live on the
commands that produced them, which is right everywhere except a tick that
fires twice a second; the web app fetches them when it wants to plot.

`ObservationEvent` is superseded by this and should be deleted along with
`_record_observation_event` and friends once plans execute.

---

# Special cases

**Maintenance** takes the whole instrument, not just the dish. It moves to
the maintenance position, holds the rotor, and stops the radio workers,
until an operator explicitly releases it. Everything anchored during the
lease fails on arrival, so the operator taking it should be told what
they're about to cancel.

**Ad-hoc commands** are ordinary daemon commands sent to the existing queue,
not plan operations. Point the dish, stow, start the analyzer — the things
an operator does by hand between observations.

The consequence to watch: they bypass the timeline, so an ad-hoc `stow`
during a running plan will fight that plan for the rotor. That's probably
correct as an operator override, but it means the daemon can be in a state
the timeline doesn't describe.

**Surveys** are manually scheduled, like anything else. Automating "observe
whenever the instrument is free" needs preemption, which this model doesn't
have.

---

# A worked example

"Observe CassA for an hour at L band, starting 03:00."

The browser sends one command:

```
plan  start_time 03:00, max_duration 1h
      commands: [ ObserveObject(CassA, band=L, total_time=1h) ]
```

The daemon expands that into primitives and lays them on the timeline. Each
resource gets its own timeline — a **row** — and a primitive occupies one
stretch of one row:

```
rotor  03:00:00 – 03:00:38   slew to CassA
rotor  03:00:38 – 03:01:38   track CassA          on source
pol_x  03:00:38 – 03:01:38   integrate
rotor  03:01:38 – 03:01:45   slew +5° off
rotor  03:01:45 – 03:02:45   track off-source     reference
pol_x  03:01:45 – 03:02:45   integrate
         ... × 30 switches
```

Both rows are busy throughout — position switching moves the dish, so the
rotor is claimed for the whole hour, not just the opening slew.

`advance()` then runs those primitives one at a time, ticking progress onto
each. Nothing runs the `ObserveObject` itself; it only existed to say what
the rows should contain.

---

# Open questions

**Slew margin.** Slew time is the only non-deterministic term in an
expansion. Use a flat **60 s** for now, rather than computing it from
`pAzVmax` / `pAzAmax` — the dish's real settling behaviour isn't measured
yet, and a fixed number that's obviously conservative beats an estimate
that's precisely wrong. Revisit with data.

**Where validation stops.** `validate_observation_plan` checks:

- every tracked source stays above the terrain profile for its span
- every commanded az/el is inside `AZLIMITS` / `ELLIMITS`
- the requested `output_format` is in `DriverCapabilities.supported_output_formats`
- per-band spectrum settings exist for every band the plan uses
- the plan's spans don't conflict with the timeline

It deliberately does *not* check that a plan looks like a sensible
observation. A radio integration with no rotor span under it is allowed —
the operator may not care where the dish is pointing.

---

# Still to build

- **The scheduler** — admission, deciding what's due, the handler loop.
  `Timeline.conflicts()` and `row()` are stubs.
- **`PlanOperation`** — doesn't exist; the websocket accepts whole plans.
- **The radio workers** (one thread per polarization) and the result queue.
- **The snapshot** the status thread should read instead of shared state.
- **`TICK_EXCLUDE` is unused** — `update_status` calls a bare
  `model_dump_json()`.
- **`validate_observation_plan`** checks a plan in isolation; runnability
  also depends on the timeline.
- **`siglent_driver` sends `:SWE:TIME:AUTO ON`**, the only reason
  integration duration isn't already knowable. Set it from
  `sweep_time_seconds`.
- **The daemon still executes text strings.** Nothing calls any of the
  above.
