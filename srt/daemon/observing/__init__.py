"""
Observation mode: planning, calibration, and data collection.

Nothing here is implemented — every body raises NotImplementedError. Design
is in docs/command-execution-design.md; science requirements are in the
README's "What we're building toward".

**The model.** A plan holds three lists, derived left to right:

    commands   what was asked for     may include aggregates
    steps      what it turned into    always primitives; what runs
    spans      when each step runs    one per step, one resource each

Commands are Pydantic models in `command_types.py` carrying their own uuid,
state, progress and results — so publishing plan progress is just publishing
the plan.

Each resource has its own timeline — a **row** — and **primitives** occupy
one stretch of one row.
**Aggregates** are patterns that expand into primitives: `ObserveObject`
alternates track-on-source and integrate with slew-off-source and integrate,
occupying both the rotor and a polarization row throughout. Only primitives
are ever run, so a new observing mode is a new pattern, not a new execution
path.

**Files.**

    settings.py    observing policy — what to calibrate against, how often
    routines.py    expand(), span_for(), advance(), needs_calibration()
    validation.py  can this plan run (terrain, limits, driver capability)
    radiometry.py  the math, ported from scripts/combined_data_collection.py

The receiver contract lives outside this package, in
`radio_control/driver.py`, so drivers don't import the observing layer to
implement it.

**Why the handler never blocks.** It runs on one thread, and anything it
waits on is time it can't spend draining the operations queue or servicing
another resource — the bug that crashed the daemon on Saren's n-point scan,
stuck in `time.sleep` so it never saw 'stop'.

So every resource row has its own worker and `advance()` only sets intent or
polls. The rotor threads already work this way; radio gets one worker per
polarization, which is also what lets POL_X and POL_Y integrate at once.

**Results live on the command that produced them.** Keeping the 2 Hz tick
small is a transport concern: `status.TICK_EXCLUDE` drops frames from the
broadcast, and the web app fetches them when it wants to plot them.
"""
