# Servo Automation Workbench Reverse Engineering Memo

Date: 2026-03-28  
Source artifact: 6mv105c.saw (plaintext string extraction from binary container)

## Scope and confidence

This memo reconstructs behavior from recoverable plaintext embedded in the SAW project file. The reconstruction is high-confidence for:

- command names and command routing,
- task names and high-level state machine,
- most control-loop variables and equations,
- firmware/autostart and persistent storage logic.

It is medium-confidence for exact branching details where strings are truncated.

## Executive summary

The project is a telescope servo controller with four major runtime domains:

1. Serial command processor task: parses comma-delimited ASCII commands and dispatches control/status handlers.
2. Position loop task: cascaded position-to-velocity-to-torque controller for azimuth/elevation.
3. Encoder calibration task: drives axes, detects index deltas, computes angular offsets and reconstructed positions.
4. Flash and persistence manager: firmware updates, autostart image staging, typed persistent parameter storage.

## Reconstructed architecture

### Main runtime tasks

- TaskMonitorCommPort: command parser and dispatcher.
- TaskPositionLoop: active tracking/control loop.
- TaskCalEncoders: calibration routine.
- TaskUpdateDisplay: telemetry and debug panel updates.
- TaskSafetyMonitor: present in project strings; appears conditionally started (or commented in some paths).

### Startup behavior

Recovered startup strings indicate this sequence:

1. Reset watchdog and initialize flags/counters.
2. Zero command states:
   - CmdAz, CmdEl,
   - CmdVelAz, CmdVelEl,
   - CmdTorqueAz, CmdTorqueEl,
   - position/velocity errors and integrators,
   - offsets and index/calibration state.
3. Initialize loop gains and constraints.
4. Zero commanded torques on all DAC-driven axes (X, Y, Z).
5. Start display update scheduling.
6. Start comm-port monitor task.

## Control-loop model (TaskPositionLoop)

The structure is cascaded PI with feedforward and saturation at multiple stages.

### Signal definitions (from recovered identifiers)

- Position commands: CmdAz, CmdEl
- Position measurements: Paz, Pel (and/or CppAz/CppEl intermediate reference forms)
- Position errors: AzPosErr, ElPosErr
- Position integral errors: AzPosIntErr, ElPosIntErr
- Velocity commands: CmdVelAz, CmdVelEl
- Velocity measurements: $v^{meas}_{az}$, $v^{meas}_{el}$ (raw fields: Vaz, Vel)
- Velocity errors: AzVelErr, ElVelErr
- Velocity integral errors: AzVelIntErr, ElVelIntErr
- Torque commands: CmdTorqueAz, CmdTorqueEl

### Position error

$$
\mathrm{AzPosErr} = \mathrm{CmdAzRef} - \mathrm{AzMeas}
$$
$$
\mathrm{ElPosErr} = \mathrm{CmdElRef} - \mathrm{ElMeas}
$$

Where recovered strings suggest forms equivalent to:

$$
\mathrm{AzPosErr} = \mathrm{CppAz} - \mathrm{Paz},\quad
\mathrm{ElPosErr} = \mathrm{CppEl} - \mathrm{Pel}
$$

### Position integral update with anti-windup

Using loop step size visible as 0.005:

$$
\mathrm{AzPosIntErr}_{k+1} = \mathrm{clip}\left(\mathrm{AzPosIntErr}_k + 0.005\,\mathrm{AzPosErr}_k,\ -pAzImax,\ pAzImax\right)
$$
$$
\mathrm{ElPosIntErr}_{k+1} = \mathrm{clip}\left(\mathrm{ElPosIntErr}_k + 0.005\,\mathrm{ElPosErr}_k,\ -pElImax,\ pElImax\right)
$$

### Position PI plus feedforward to velocity command

$$
\mathrm{CmdVelAz}^{(axis)} = pAzKpp\,\mathrm{AzPosErr} + pAzKpi\,\mathrm{AzPosIntErr} + pAzKff\,Azu
$$
$$
\mathrm{CmdVelEl}^{(axis)} = pElKpp\,\mathrm{ElPosErr} + pElKpi\,\mathrm{ElPosIntErr} + pElKff\,Elu
$$

Then converted to motor-domain scaling:

$$
\mathrm{CmdVelAz}^{(motor)} = pAzAr\,\mathrm{CmdVelAz}^{(axis)},\quad
\mathrm{CmdVelEl}^{(motor)} = pElAr\,\mathrm{CmdVelEl}^{(axis)}
$$

### Acceleration limiting (rate limiting)

With sample interval 0.005 s:

$$
\mathrm{CmdVelAz}_{k+1} = \mathrm{clip}\left(
\mathrm{CmdVelAz}_{raw},
\mathrm{CmdVelAz}_k - 0.005\,pAzAmax\,pAzAr,
\mathrm{CmdVelAz}_k + 0.005\,pAzAmax\,pAzAr
\right)
$$

$$
\mathrm{CmdVelEl}_{k+1} = \mathrm{clip}\left(
\mathrm{CmdVelEl}_{raw},
\mathrm{CmdVelEl}_k - 0.005\,pElAmax\,pElAr,
\mathrm{CmdVelEl}_k + 0.005\,pElAmax\,pElAr
\right)
$$

### Velocity limiting

$$
\mathrm{CmdVelAz} \in [-pAzVmax\,pAzAr,\ pAzVmax\,pAzAr]
$$
$$
\mathrm{CmdVelEl} \in [-pElVmax\,pElAr,\ pElVmax\,pElAr]
$$

### Velocity PI to torque

$$
\mathrm{AzVelErr} = \mathrm{CmdVelAz} - v^{meas}_{az},\quad
\mathrm{ElVelErr} = \mathrm{CmdVelEl} - v^{meas}_{el}
$$

$$
\mathrm{AzVelIntErr}_{k+1} = \mathrm{AzVelIntErr}_k + 0.005\,\mathrm{AzVelErr}_k
$$
$$
\mathrm{ElVelIntErr}_{k+1} = \mathrm{ElVelIntErr}_k + 0.005\,\mathrm{ElVelErr}_k
$$

$$
\mathrm{CmdTorqueAz} = pAzKvp\,\mathrm{AzVelErr} + pAzKvi\,\mathrm{AzVelIntErr}
$$
$$
\mathrm{CmdTorqueEl} = pElKvp\,\mathrm{ElVelErr} + pElKvi\,\mathrm{ElVelIntErr}
$$

### Torque saturation and Y/Z az split mapping

Clamp torques:

$$
\mathrm{CmdTorqueAz} \in [pAzTmin,\ pAzTmax],\quad
\mathrm{CmdTorqueEl} \in [pElTmin,\ pElTmax]
$$

Recovered mapping to DAC outputs:

$$
\mathrm{CmdTorqueX} = \mathrm{round}(\mathrm{CmdTorqueEl})
$$
$$
\mathrm{CmdTorqueY} = \mathrm{round}(\mathrm{CmdTorqueAz} + pAzTbias - 0.3\,\Delta Vaz)
$$
$$
\mathrm{CmdTorqueZ} = \mathrm{round}(\mathrm{CmdTorqueAz} - pAzTbias + 0.3\,\Delta Vaz)
$$

Then X/Y/Z axis commanded torques are written to DAC outputs.

## Calibration model (TaskCalEncoders)

Recovered behavior suggests:

1. Set CalComplete to in-progress.
2. Drive axis motion patterns and observe index transitions.
3. Compute ElTheta and AzTheta from index delta rules (piecewise formulas detected).
4. Compute reconstructed encoder offsets RecEl/RecAz using scale factor 400 and offset terms pElEpo/pAzEpo.
5. Stop torques, mark calibration complete.

Representative recovered forms:

$$
\mathrm{RecEl} = \mathrm{round}\left(400\,(\mathrm{ElTheta} - pElEpo)\right) + \mathrm{Waxis.ActualPosition} - \mathrm{ElIndexPos}
$$

$$
\mathrm{RecAz} = \mathrm{round}\left(400\,(\mathrm{AzTheta} - pAzEpo)\right) + \mathrm{Vaxis.ActualPosition} - \mathrm{AzIndexPos}
$$

Calibration state encoding (from status outputs):

- 0: not calibrated
- 1: calibration in progress
- 2: calibration complete

## Serial command protocol (recovered)

### Command routing in TaskMonitorCommPort

Detected command tokens and comments:

- AZEL: command Az/El/time
- AZL: command Az/El
- GAE: request Az/El
- GET: download buffer data
- ZPN: zero buffer pointer
- TON: start position loop
- LPR: load parameter set
- CLE: calibrate encoders
- SPA: stop all tasks except comm
- CLO: clear offsets
- POF: set position offsets
- VOF: set velocity offsets
- STS: status request
- STX: extended status request
- ERR: position error request
- SEN: set encoder values
- TMD: set test mode
- EBN: elevation brake on
- EBF: elevation brake off
- ABN: azimuth brake on
- ABF: azimuth brake off
- AZV: set az DAC drive
- ELV: set el DAC drive
- VER: return version
- ENC: read motor encoders
- NER: number of position-loop errors

### Reply behavior

- Set-type commands often call SendAck(token).
- Query-type commands often send comma-prefixed payload strings via comm.SendString(...).
- Parser failures trigger a scanning error escape path.

## Firmware, autostart, and flash behavior

Recovered logic includes:

1. Device identify and flash path selection (AMD vs SST flow variants).
2. ProgramByte with timeout and readback verification.
3. Sector erase with timeout-based escape handling.
4. Firmware image update from OCT text lines.
5. Updated-firmware signature checks in segment E000 bytes 0..3:
   - expected pattern includes 55, AA, AA, 55.
6. Autostart header and routine pointer storage in segment 8000:
   - stores Main.Setup address bytes.
7. Invalidate/disable autostart by zeroing 8000:0..3 in some flows.

## Persistent storage subsystem

The persistent store is sophisticated and wear-aware.

### Key mechanism

- Uses 256-byte sectors per logical index region.
- Header bytes encode type and validity; jump-table advances over variable record sizes.
- New writes append records; older header is invalidated (top-bit clearing pattern observed).

Recovered jump table initialization:

- JumpTable[1] = 2 (boolean payload)
- JumpTable[2] = 3 (integer payload)
- JumpTable[3] = 5 (longint payload)
- JumpTable[4] = 5 (single payload)

### Typed API surface

- SetPersistentBoolean / PersistentBoolean
- SetPersistentInteger / PersistentInteger
- SetPersistentLongint / PersistentLongint
- SetPersistentSingle / PersistentSingle
- SavePersistentDataToFile / LoadPersistentDataFromFile

### Error paths

Detected escape codes for:

- program-byte failure,
- erase failure,
- persistent sector erase failure,
- persistent datatype mismatch,
- scanner/parser failures,
- comm serial transmit/receive failures.

## Quirks and likely bugs

1. ACK mismatch in DAC command path:
   - AVS branch appears to SendAck("AZS") (possible typo/compat artifact).
2. Safety task startup ambiguity:
   - TaskSafetyMonitor appears in code strings but one startup line appears commented in extracted text.
3. Brake polarity depends on hardware IO semantics:
   - EBN/EBF and ABN/ABF map to output bit on/off; physical brake polarity must be verified on hardware.
4. Dual gain namespaces:
   - Legacy gains (AzPosPGain, etc.) and active pAz*/pEl* gains both appear.
5. Direct DAC write commands (AZV/ELV) can bypass normal trajectory intent and should be treated as hazardous in operation.

## Operator-safe command workflow (recommended)

1. Query status:
   - STS (or STX), verify calibration and mode.
2. Ensure brakes are in intended state:
   - ABF/EBF for motion, ABN/EBN for hold.
3. Load/confirm parameters:
   - LPR then STS.
4. Optional offsets:
   - POF and/or VOF, validate with STS/ERR.
5. Start tracking loop:
   - TON.
6. Send trajectory commands:
   - AZEL or AZL.
7. Monitor:
   - STS, ERR, NER, ENC.
8. Stop safely:
   - SPA, then apply brakes if needed.

## Practical implications for srt-py integration

This protocol maps well to a command-queue driver model where each outbound command has:

- strict token formatting,
- optional ack expectation,
- timeout and retry semantics,
- parser/escape failure interpretation.

The control loop itself is on-controller; host software should emphasize:

- parameter sanity bounds,
- safe state transitions,
- telemetry validation,
- explicit stop/fault handling.

## Appendix: concise pseudocode

### Startup

1. Init state, gains, offsets, errors, counters.
2. Set all commanded torques to zero.
3. Start TaskUpdateDisplay.
4. Start TaskMonitorCommPort.

### Command processor

1. Read line.
2. Tokenize to uppercase command and arguments.
3. Dispatch by command string.
4. For setters: parse numeric args, set values, SendAck.
5. For getters: stream payload via comm.SendString.
6. On malformed input: scanning escape.

### Position loop tick

1. Build/refine command trajectory (including offsets).
2. Position PI + FF to velocity command.
3. Acceleration and velocity limits.
4. Velocity PI to torque command.
5. Torque limits and axis-mapping.
6. Write DAC torques.
7. Update debug/status channels and optional log buffers.

## Appendix B: quick-reference command sheet

### Motion and task control

| Command | Purpose | Typical effect |
|---|---|---|
| AZEL | Set az, el, and time | Updates commanded pointing trajectory |
| AZL | Set az and el only | Immediate target update |
| TON | Start tracking loop | Begins TaskPositionLoop |
| SPA | Stop active tasks | Stops motion tasks, leaves comm task alive |
| CLE | Calibrate encoders | Runs TaskCalEncoders, updates calibration state |

### Offsets and loop configuration

| Command | Purpose | Typical effect |
|---|---|---|
| LPR | Load parameter set | Writes loop gains/limits and related config |
| VOF | Set velocity offsets | Adjusts velocity bias terms |
| POF | Set position offsets | Adjusts position bias terms |
| CLO | Clear offsets | Zeros VOF/POF-related state |
| SEN | Set encoder values | Writes encoder-related state |
| TMD | Set test mode | Activates internal test injections |

### Brake and direct output commands

| Command | Purpose | Caution |
|---|---|---|
| EBN / EBF | Elevation brake on/off | Verify IO polarity on hardware |
| ABN / ABF | Azimuth brake on/off | Verify IO polarity on hardware |
| AZV | Direct azimuth DAC set | Can bypass normal trajectory control |
| ELV | Direct elevation DAC set | Can bypass normal trajectory control |
| AVS | Alternate az path in parser | Recovered ACK appears as AZS (quirk) |

### Status and telemetry

| Command | Purpose | Typical response |
|---|---|---|
| STS | Status packet | Task state, booleans, calibration code |
| STX | Extended status | STS-like, extended fields |
| ERR | Position error report | Az/el error values |
| GAE | Get az/el | Current az/el telemetry |
| ENC | Encoder report | Motor/encoder values |
| NER | Number of errors | Position loop error counter |
| GET | Buffer download | Logged channel data |
| VER | Version | Firmware/version text |

## Appendix C: parameter glossary and clearer formula mapping

This section defines the pAz*/pEl* parameters explicitly and rewrites the controller equations with grouped terms.

### LPR loading scope (what `LPR` appears to set)

Based on recovered parser strings, this is the best-supported split:

#### Loaded directly by `LPR`

Confirmed as parsed and assigned in `CmdLPR` (from `DoubleInString(...)` assignments):

- pAzKo, pElKo, pAzKv, pElKv, pAze, pEle
- pAzAmax, pElAmax, pAzVmax, pElVmax, pAzImax, pElImax
- pAzKpp, pElKpp, pAzKpi, pElKpi, pAzKvp, pElKvp, pAzKvi, pElKvi, pAzKff, pElKff
- pAzTmax, pElTmax, pAzTmin, pElTmin, pAzTsgn, pElTsgn, pAzTbias
- pAzEcr, pElEcr, pAzMEcr, pElMEcr, pAzEpo, pElEpo
- pAzAr, pElAr

Also seen in related parsing paths (likely command-parameter paths adjacent to `LPR`):

- pAzMrd, pElMrd, pAzMcr, pElMcr

#### Not loaded by `LPR` (loaded by other commands)

- VelOffAz, VelOffEl: loaded by `VOF`
- PosOffAz, PosOffEl: loaded by `POF`
- Offset reset behavior: `CLO` clears offset-related states
- TestMode: loaded by `TMD`

This section is intentionally scoped to what the command parser shows about `LPR` assignment behavior.

### Controller equations (grouped)

Position loop:

$$
\begin{aligned}
e_{p,az} &= \mathrm{AzPosErr}, \\
e_{p,el} &= \mathrm{ElPosErr}, \\
I_{p,az}[k+1] &= \mathrm{clip}\left(I_{p,az}[k] + T_s e_{p,az}[k],\ -I^{max}_{p,az},\ I^{max}_{p,az}\right), \\
I_{p,el}[k+1] &= \mathrm{clip}\left(I_{p,el}[k] + T_s e_{p,el}[k],\ -I^{max}_{p,el},\ I^{max}_{p,el}\right), \\
v_{az}^{axis} &= K^{pos}_{P,az}\,e_{p,az} + K^{pos}_{I,az}\,I_{p,az} + K^{ff}_{az}\,u_{az}, \\
v_{el}^{axis} &= K^{pos}_{P,el}\,e_{p,el} + K^{pos}_{I,el}\,I_{p,el} + K^{ff}_{el}\,u_{el}.
\end{aligned}
$$

Axis-to-motor scaling and limits:

$$
v_{az}^{motor} = r_{az}\,v_{az}^{axis},\qquad
v_{el}^{motor} = r_{el}\,v_{el}^{axis}
$$

$$
v_{az}^{motor} \in [-v^{max}_{az}\,r_{az},\ v^{max}_{az}\,r_{az}],\qquad
v_{el}^{motor} \in [-v^{max}_{el}\,r_{el},\ v^{max}_{el}\,r_{el}]
$$

Velocity loop:

$$
\begin{aligned}
e_{v,az} &= v_{az}^{motor} - v^{meas}_{az}, \\
e_{v,el} &= v_{el}^{motor} - v^{meas}_{el}, \\
I_{v,az}[k+1] &= I_{v,az}[k] + T_s e_{v,az}[k], \\
I_{v,el}[k+1] &= I_{v,el}[k] + T_s e_{v,el}[k], \\
τ_{az} &= K^{vel}_{P,az}\,e_{v,az} + K^{vel}_{I,az}\,I_{v,az}, \\
τ_{el} &= K^{vel}_{P,el}\,e_{v,el} + K^{vel}_{I,el}\,I_{v,el}.
\end{aligned}
$$

Torque clamp and DAC mapping:

$$
τ_{az} \in [τ^{min}_{az},\ τ^{max}_{az}],\qquad
τ_{el} \in [τ^{min}_{el},\ τ^{max}_{el}]
$$

$$
\begin{aligned}
\mathrm{CmdTorqueX} &= \mathrm{round}(τ_{el}), \\
\mathrm{CmdTorqueY} &= \mathrm{round}(τ_{az} + b_{τ,az} - 0.3\,\Delta v_{az}), \\
\mathrm{CmdTorqueZ} &= \mathrm{round}(τ_{az} - b_{τ,az} + 0.3\,\Delta v_{az}).
\end{aligned}
$$

Where the recovered sample period is:

$$
T_s \approx 0.005\ \mathrm{s}
$$

Interpretation of $T_s$:

- $T_s$ is the controller update interval in the discrete-time loop.
- With $T_s \approx 0.005\ \mathrm{s}$, the loop runs at about $1/T_s \approx 200\ \mathrm{Hz}$.
- Integrators use $T_s$ as the accumulation scale (larger $T_s$ means faster integral buildup per tick).
- Acceleration limiting terms also use $T_s$, converting acceleration limits into per-cycle velocity-step limits.
- If runtime timing deviates from 0.005 s, effective gains and slew limits deviate proportionally.

Recovered variable-name mapping for velocity measurements:

$$
v^{meas}_{az} \leftrightarrow \mathrm{Vaz},\qquad
v^{meas}_{el} \leftrightarrow \mathrm{Vel}
$$

### Parameter glossary table

| Raw field name | Rendered symbol | Meaning | LPR load |
|---|---|---|---|
| pAzKpp | $K^{pos}_{P,az}$ | Az position P gain | Yes |
| pAzKpi | $K^{pos}_{I,az}$ | Az position I gain | Yes |
| pAzKff | $K^{ff}_{az}$ | Az feedforward gain | Yes |
| pElKpp | $K^{pos}_{P,el}$ | El position P gain | Yes |
| pElKpi | $K^{pos}_{I,el}$ | El position I gain | Yes |
| pElKff | $K^{ff}_{el}$ | El feedforward gain | Yes |
| pAzKvp | $K^{vel}_{P,az}$ | Az velocity P gain | Yes |
| pAzKvi | $K^{vel}_{I,az}$ | Az velocity I gain | Yes |
| pElKvp | $K^{vel}_{P,el}$ | El velocity P gain | Yes |
| pElKvi | $K^{vel}_{I,el}$ | El velocity I gain | Yes |
| pAzImax | $I^{max}_{p,az}$ | Az position-integrator clamp | Yes |
| pElImax | $I^{max}_{p,el}$ | El position-integrator clamp | Yes |
| pAzAmax | $a^{max}_{az}$ | Az accel limit | Yes |
| pElAmax | $a^{max}_{el}$ | El accel limit | Yes |
| pAzVmax | $v^{max}_{az}$ | Az max velocity | Yes |
| pElVmax | $v^{max}_{el}$ | El max velocity | Yes |
| pAzTmax | $\tau^{max}_{az}$ | Az max torque command | Yes |
| pAzTmin | $\tau^{min}_{az}$ | Az min torque command | Yes |
| pElTmax | $\tau^{max}_{el}$ | El max torque command | Yes |
| pElTmin | $\tau^{min}_{el}$ | El min torque command | Yes |
| pAzTbias | $b_{\tau,az}$ | Az split bias for Y/Z mapping | Yes |
| pAzAr | $r_{az}$ | Az axis-to-motor ratio | Yes |
| pElAr | $r_{el}$ | El axis-to-motor ratio | Yes |
| pAzEpo | $\phi^{off}_{enc,az}$ | Az encoder phase/offset | Yes |
| pElEpo | $\phi^{off}_{enc,el}$ | El encoder phase/offset | Yes |
| pAzEcr | $c^{err}_{az}$ | Az calibration/error coefficient | Yes |
| pElEcr | $c^{err}_{el}$ | El calibration/error coefficient | Yes |
| pAzMEcr | $c^{err2}_{az}$ | Az secondary calibration coefficient | Yes |
| pElMEcr | $c^{err2}_{el}$ | El secondary calibration coefficient | Yes |
| pAzTsgn | $s_{\tau,az}$ | Az torque sign/polarity term | Yes |
| pElTsgn | $s_{\tau,el}$ | El torque sign/polarity term | Yes |
| pAzKo | $k^{model}_{0,az}$ | Legacy model/constraint coefficient | Yes |
| pElKo | $k^{model}_{0,el}$ | Legacy model/constraint coefficient | Yes |
| pAzKv | $k^{model}_{v,az}$ | Legacy model/constraint coefficient | Yes |
| pElKv | $k^{model}_{v,el}$ | Legacy model/constraint coefficient | Yes |
| pAze | $k^{model}_{e,az}$ | Legacy model/constraint coefficient | Yes |
| pEle | $k^{model}_{e,el}$ | Legacy model/constraint coefficient | Yes |
| pAzMrd | $m^{rd}_{az}$ | Additional model term seen in parser | Adjacent path |
| pElMrd | $m^{rd}_{el}$ | Additional model term seen in parser | Adjacent path |
| pAzMcr | $m^{cr}_{az}$ | Additional model term seen in parser | Adjacent path |
| pElMcr | $m^{cr}_{el}$ | Additional model term seen in parser | Adjacent path |

Notes:

- LPR load status is from recovered `CmdLPR` assignment strings.

### Naming convention decoder

The repeated symbol structure can be read as:

- p: parameter
- Az / El: azimuth or elevation axis
- K: gain
- pp: position-proportional
- pi: position-integral
- vp: velocity-proportional
- vi: velocity-integral
- ff: feedforward
- Amax / Vmax / Imax: max accel, velocity, integrator
- Tmax / Tmin: torque limits
- Tbias: azimuth split-output bias
- Ar: axis-to-motor scaling ratio
- Epo: encoder phase offset

