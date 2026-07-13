# Telescope Pointing-First Restructure Plan

Date: 2026-03-28
Branch intent: repurpose this branch toward telescope pointing/control workflows (Caltech 6m focused), de-emphasize GNU Radio/SDR, and support robust offline simulation.

## Executive Summary

The current codebase is still architected as an SRT+radio system:
- The daemon initializes both rotor and radio tasks together.
- Dashboard status rendering depends on radio fields.
- The current SOFTWARE config flag only toggles a few monitor-page layout variants, not a full product mode.

Recommendation:
1. Do not do a destructive rewrite immediately.
2. Introduce a "pointing-first" product mode behind configuration, then peel radio out of the critical path in phases.
3. Build a deterministic serial state machine for Caltech 6m control and a practical simulation backend that implements the same interface for offline development.

## What Exists Today (Relevant Findings)

### 1) SOFTWARE is already a UI switch, but shallow
- SOFTWARE is defined in schema/config.
- Dashboard currently passes SOFTWARE through to layout functions.
- Practical effect today: mostly differences in monitor page controls/layout for "Very Small Radio Telescope" vs default.

Implication: SOFTWARE can be used as the top-level mode selector, but we need to extend it beyond layout tweaks to drive command set, status schema, and backend adapter selection.

### 2) Control backend is tightly coupled to radio
- Daemon startup creates radio process task and starts radio threads/queues.
- Status payload includes many radio-centric fields expected by callbacks.
- Command handlers include recording/calibration/radio parameters by default.

Implication: for a control-centric fork, define a reduced status contract and optional radio adapter so telescope control is stable with radio absent.

### 3) Simulation support exists but is not complete for Caltech 6m workflow
- A NoMotor class exists for fixed/simulated az/el.
- Config/docs mention MOTOR_TYPE: NONE for simulation/stationary mode.
- Current rotor wrapper in this branch appears hardwired to Caltech6m motor class.

Implication: formalize simulation as a first-class motor backend with realistic behavior (slew rate, limits, faults, delayed telemetry), not just static position.

### 4) Caltech6m serial handling is procedural and ad hoc
- Startup sends fixed command sequence and checks for expected patterns.
- Command/retry logic exists, but no explicit lifecycle FSM.
- Error handling is mostly logging + retries; state transitions are implicit.

Implication: move to an explicit serial state machine with clear transitions, timeouts, recovery strategy, and observability.

## Proposed Target Architecture

### A) Product modes (config-driven)
Add a mode layer on top of SOFTWARE, for example:
- SOFTWARE: Caltech 6m Control
- CONTROL_PROFILE: POINTING_ONLY | FULL_SRT
- HARDWARE_PROFILE: CALTECH6M | SIMULATED

Behavior:
- POINTING_ONLY hides/disables radio controls and never starts radio processes.
- FULL_SRT preserves existing behavior.
- HARDWARE_PROFILE selects concrete motor backend (real serial vs simulated).

### B) Backend split by interfaces
Define explicit interfaces:
1. PointingController
- set_target_azel
- set_offset
- stow
- stop
- get_state

2. TrackingProgrammer
- track_object(name, cadence_s, horizon_deg=15)
- track_radec(ra_deg, dec_deg, cadence_s)
- track_altaz_sequence(points, dwell_s)
- run_sequence(sequence_spec)
- stop_tracking()

Notes:
- This interface should be astropy-native for coordinate transforms and target resolution.
- It should be backend-agnostic, so the same tracking program runs on Caltech6m and simulation.

3. TelemetryProvider
- get_status_snapshot (single canonical schema)

4. Optional RadioAdapter
- no-op implementation in POINTING_ONLY mode
- real GNU Radio adapter in FULL_SRT

Implementation guidance:
- Reuse proven command semantics from Caltech6m code paths already in use.
- Pull shared command/parse logic into reusable helpers, then place FSM around them rather than rewriting behavior from scratch.

### C) Serial state machine (Caltech6m)
Implement a formal FSM with at least:
- DISCONNECTED
- CONNECTING
- STARTUP_SYNC
- READY
- SLEWING
- TRACKING
- FAULT
- RECOVERING
- SHUTDOWN

Transition triggers:
- connect success/failure
- startup command sequence verified
- command accepted/completed/timeout
- parse/CRC/protocol errors
- limit switches/emergency stop flags
- explicit stop/shutdown

Reliability rules:
1. Single writer thread for serial TX/RX.
2. Command queue with per-command timeout and retry policy.
3. Read/parse framing centralized in one parser.
4. Idempotent recovery routine from FAULT -> RECOVERING -> READY.
5. Structured state + event logs suitable for dashboard display.

### D) Simulation mode requirements
Simulation backend should emulate:
- az/el limits and clamping
- finite slew rates and settle time
- queued command timing
- startup/calibration state and status flags

This is intentionally a sensible offline dev solution, not a high-fidelity physics simulator.

## UI Strategy: Customize vs Rewrite

Decision: prefer heavy customization first, rewrite later only if necessary.

Rationale:
- SOFTWARE-based branching already exists.
- We can hide radio controls and graphs quickly while preserving command transport and status wiring.
- Faster path to a working Caltech6m control panel with lower risk.

Concrete UI plan:
1. Add a dedicated SOFTWARE value for Caltech control mode.
2. Build a "Pointing" monitor layout variant:
- target/object controls
- current az/el, destination, offsets
- command queue + last errors
- safety/limit/brake/emergency indicators
- optional trajectory/slew timeline
3. Remove/disable radio plots and recording controls in pointing-only mode.
4. Keep system page but retarget sections toward motor/serial diagnostics.

## Implementation Plan (Phased)

### Phase 0 - Stabilization and contracts
1. Freeze current status keys and commands used by UI.
2. Define new pointing-only status schema and compatibility layer.
3. Add feature flags in config/schema for profile and simulation behavior.

Deliverable:
- Daemon can run with radio disabled and still serve healthy status.

### Phase 1 - Decouple radio from control core
1. Extract radio lifecycle into optional adapter.
2. Ensure command parser works without radio commands.
3. Make dashboard callbacks tolerant to missing radio fields in pointing-only mode.

Deliverable:
- End-to-end pointing workflow independent of GNU Radio.

### Phase 2 - Serial FSM for Caltech6m
1. Introduce explicit state enum and event-driven loop.
2. Move startup sequence and command handling under FSM transitions.
3. Implement retry/backoff + fault recovery.
4. Expose current_state, last_transition, retry_count, and last_error in status.

Deliverable:
- Predictable behavior through disconnects/timeouts and clean observability.

### Phase 3 - Practical simulation backend
1. Implement SimCaltech6m backend with motion model.
2. Add simple timing and state behavior for startup, slew, and stop.
3. Keep implementation small and deterministic for daily dev use.

Deliverable:
- Hardware-free development path that mirrors operator workflows.

### Phase 4 - UI customization for pointing-first operations
1. Add Caltech-specific monitor mode driven by SOFTWARE.
2. Replace radio cards with pointing/safety/health cards.
3. Add simulation status badge and controls (optional in dev builds).

Deliverable:
- Operator-focused control UI without radio clutter.

## Testing and Validation Plan

Constraint for initial implementation:
- Skip formal unit test buildout in this phase.
- Focus on integration smoke checks and operator workflow validation first.

### Integration tests
- daemon + dashboard in POINTING_ONLY + SIMULATED
- daemon + real Caltech6m serial adapter smoke tests
- command dispatch and status publication contract tests

### Acceptance criteria
1. App starts and remains healthy with no SDR/GNU Radio installed.
2. Point/stow/offset/track workflows work in simulation and real hardware mode.
3. Serial disruptions recover without process restart in supported scenarios.
4. UI clearly communicates state, faults, and command progress.

## Risks and Mitigations

1. Risk: Hidden coupling to radio fields in callbacks.
Mitigation: add schema validation and mode-gated callback branches early.

2. Risk: Serial quirks not captured by FSM assumptions.
Mitigation: include raw transcript logging and state/event replay fixtures.

3. Risk: Divergence from upstream SRT functionality.
Mitigation: keep FULL_SRT profile available while POINTING_ONLY matures.

## Suggested Immediate Next Steps

1. Add profile config keys and no-radio daemon boot path.
2. Implement an initial FSM shell around existing Caltech6m send/get functions.
3. Build a minimal Caltech control dashboard variant using SOFTWARE-based branching.
4. Add a baseline SimCaltech6m backend to unblock hardware-free development.
