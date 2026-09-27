# srt-py

Motor and radio control for the Caltech 6m dish on the Moore roof.

This is a fork[^1] of MIT Haystack Observatory's motor+radio control program, which they developed for their 'Small Radio Telescope' (SRT) DIY educational radio telescope kit. It's written in python (hence `srt-py`).

**This describes the `new_frontend` branch, as of September 2026.** `master` is still the old `dash` version and does not have any of the Svelte app, the FastAPI backend, or the Pydantic type system described below.

---

# Quick start for local development

## Setup

1. Install [conda forge](https://conda-forge.org/download/) and [pnpm](https://pnpm.io/installation) if you haven't already.

2. Create the conda environment and install dependencies:

```bash
mamba env create -f environment.yml
```

3. Install the frontend dependencies:

```bash
cd srt/svelte-frontend
pnpm install
```

4. Tell pnpm where to find the conda environment by creating a `.env` file in the `srt/svelte-frontend` directory with the following content:

```bash
SRT_DEV_PYTHON_PATH="/path/to/your/conda/env/bin/python"
```

The easiest way is to copy the `.env.example` file and modify it accordingly. You can get the path to your conda environment by running:

```bash
conda run -n srt-dev python -c "import sys;print(sys.executable)"
```

## Running the application

1. Activate the environment:

```bash
mamba activate srt-dev
```

2. Run the development server:

```bash
python scripts/run_dev_server.py
```

3. Open your browser and go to [http://localhost:5173/monitor](http://localhost:5173/monitor) to see the application running.

4. Changes you make to the frontend code will be automatically reflected in the browser. Changes to the backend code will require restarting the development server (Ctrl+C to stop, then run the command again).

---

# How it runs on the roof

**Entry point:** Desktop / Taskbar shortcut on the Moore Roof Desktop — **Moore 6m Controller**.

That runs a Windows PowerShell script, `scripts/run_controller.ps1`. This is a bit of a hack to make the python script conveniently launchable with one click via the windows shortcut instead of the command line — all it does is run `bin/Moore6mController.py` with the correct environment and arguments. It also makes sure only one instance is running at a time and exits immediately otherwise, just to prevent unexpected behavior from spam clicking the shortcut.

**Python script:** `bin/Moore6mController.py`, running in the `srt-dev` conda environment (defined in `environment.yml`).

Note that the various threads and processes in this script can only communicate by passing messages through ZMQ sockets[^2]. Sockets work like queues — messages are processed in the order received.

Stuff managed by this script:

- **`Moore6mDriver`** : a "singleton" class owned directly in this process and shared with whoever else needs to send commands to the telescope servo motor drivers. Basically just {my original `telescope_control.py` from 4 years ago} with a built-in position and status polling loop. It is the only code that reads and writes directly to the telescope serial port — all public methods either read a copy of the latest received information, or add commands to the send queue (except e-stop, which bypasses the queue).

- **`ZMQ PULL` socket** (port 5567) : reserved for telescope motor e-stop requests from the web app, processed immediately. The loop discards the message body and fires on anything that arrives, so the payload doesn't matter.

- **`SRT daemon`** : main bit of the original `srt-py` code, runs all of the science logic — calculating and updating azimuth/elevation positions of sky objects, reading from the spectrum analyzer, planning recording and logging observations, etc. There's already some existing observation planning capability with basic beamswitching and n-point scan functionality, but it's kind of broken, **so this is the next area for active development.** Of python type `threading.Thread`, so it can use the same `Moore6mDriver` instance. Stopped via `quit()`. More info below.

- **`FastAPI backend`** : the web server for the new web app, replacing the old dashboard. The script launches `srt/fastapi_backend/main.py`. Sends e-stop signals to the dedicated `ZMQ PULL` socket; all other info and commands go to/from the daemon over separate ZMQ sockets. Of python type `multiprocessing.Process` so we can kill it instantly when the main script exits, or restart it quickly. More info below.

- **`Tkinter` GUI** : the "Moore 6m Dish Controller" desktop window. Barebones right now but can definitely be expanded out with more buttons. Runs on the main thread.

---

# Typing and schemas

Main files: `srt/daemon/` — `common.py`, `telescope_types.py`, `scheduling.py`, `command_types.py`, `settings.py`, `status.py`

There's a lot of information flowing between a lot of different functions, programs, and interfaces. To avoid mystery errors and unexpected behavior when integrating new bits of code, it's essential to have a schema defining the exact shape and type of all data being passed around.

We define all our types in python using the **Pydantic** library. Then all our python code can import those types directly, and your IDE's language server will scream at you if you try to pass a list into a function which expects a dictionary or something. You'll also get more helpful autocomplete suggestions.

The models are split by dependency depth, so each file only imports from the ones above it:

```
common.py           shared vocabulary — Band, OutputFormat, CommandState, Resource
telescope_types.py  hardware and domain models, SpecanSettings, DaemonConfig
scheduling.py       the timeline — Span, Timeline, PlanState
command_types.py    commands and observation plans
settings.py         RuntimeSettings, the runtime-editable half of the config
status.py           DaemonStatus, which aggregates all of the above
```

`DaemonStatus` sits at the top because it aggregates everything else, so it has to import from all of them.

These type definitions also get exported into TypeScript for the web app frontend, so all our type definitions automatically stay in sync between python and typescript. This is pretty cool because now all communications sent between the server and client are validated against the ultimate source of truth for how it's supposed to work — which we've defined in the model files above. See [Development](#development) below for how that generation works.

Here's a good article explaining this a bit more, for a slightly different application — [Episode 8: JSON Schema Generation in Pydantic](https://medium.com/@kishanbabariya101/episode-8-json-schema-generation-in-pydantic-9a4c4fee02c8).

---

# SRT daemon

Main file: `srt/daemon/daemon.py`

The daemon has five threads. Each one runs independently but they all have access to the same `SmallRadioTelescopeDaemon` object (commonly abbreviated to `self` in this file), so they can read and write the same variables.

## Ephemeris Tracker thread

Loop function: `update_ephemeris_location`

This is where all of the astrology *[sic]* calculations and coordinate conversions happen. If we're currently observing an object (i.e. if `self.ephemeris_cmd_location` is set) then it constantly (1000ms) updates an internal variable (`self.rotor_cmd_location`) with the current az/el of the object.

Catalog objects come from `config/sky_coords.csv`.

## Telescope Pointing thread

Loop function: `update_rotor_status`

Every loop, checks `self.rotor_cmd_location` vs. the actual position of the telescope. If commanded is more than X away from actual, it sends the commanded location to the telescope.

Also on every loop it grabs the latest `RotorState` object from `Moore6mDriver` and updates the `self._rotor_state` variable, which is then read by the…

## Status Broadcasting thread

Loop function: `update_status`

Twice a second, constructs a `DaemonStatus` object, runs Pydantic's `model_dump_json()` on it, and sends the resulting JSON string over the ZMQ socket (port 5555). From there the web app's backend receives it and forwards it over WebSocket to any user's browser that's listening.

For future development we may want to split up this `DaemonStatus` object into one broadcast of the config settings which never change and the web app only needs to receive once (telescope metadata like limits and horizon points), and one with the status updates that change every tick (position, calculated sky coordinates, etc).

## Command Queueing thread

Loop function: `update_command_queue`

Listens for new messages on the ZMQ command socket (port 5556) and puts them into `self.command_queue`.

## Command Handler thread

Loop function: `srt_daemon_main`

Takes command strings out of `self.command_queue` and runs the corresponding code.

**This is the entry point for new development, where we can add new commands and change behavior of old ones.** Two known problems with it today:

- I have not touched the n-point scan and beamswitch commands almost at all since I created this fork, and they almost definitely do not behave correctly anymore. In particular neither one ever reads from the spectrum analyzer — `pwr_list` is initialized empty, never appended to, and published as an empty list. There's no "integrate for N seconds and give me one frame" API anywhere yet.

- Commands block the thread from processing further commands in the queue. The reason Saren experienced the daemon crashing after sending the n-point scan command is that the command handler thread got stuck in a long `time.sleep` inside the n-point scan loop, so it never looked at any further commands in the queue (like 'stop n-point scan').

A replacement design for both problems — typed commands with their own progress, a timeline with one row per resource, plans that expand into spans — already exists at the type level in `common.py`/`command_types.py`/`status.py`, and is tested. None of it is wired into this loop yet. Current build-vs-stub status, a worked example, and what's still open are all in [docs/command-execution-design.md](docs/command-execution-design.md) — that file is the source of truth for where this effort actually stands, more so than this paragraph.

---

# Web app

There are two components: the **backend** (server, runs on the computer physically connected to the telescope and spectrum analyzer on Moore roof) and the **frontend** (client, runs in the user's browser).

## Backend

Main file: `srt/fastapi_backend/main.py`

Implemented with the python **FastAPI** library, which makes it very easy to set up a web server. Endpoints as of September 2026:

`/auth/token` : backend asks for a login and password. If valid, it sends a JSON web token (JWT) back. The user must provide this token for all other telescope control/status requests, otherwise the backend will reject. The token expires after 8 hours and the user must refresh their page to get a new one.

`/ws/status` : the websocket over which the backend broadcasts status updates from the daemon every ~0.5 seconds. Browsers have to provide their authenticated token in order to connect.

`/ws/command` : the reverse direction. Everything browsers send is one envelope, `ClientRequest`, whose `kind` is `command`, `settings` or `plan`. The backend validates it, checks it against the latest daemon status, replies ok or with the reason, and forwards it to the daemon. See [docs/command-execution-design.md](docs/command-execution-design.md#the-request-envelope).

`/` : for any other request path, the backend looks through the `svelte-frontend/build` folder for a matching page. Right now all that exists is `/index.html` and `/monitor`. Anything else returns 404.

### ZMQ bridge

Main files: `srt/fastapi_backend/zmq_bridge/status.py` and `commands.py`

Two classes sit between the websockets and the daemon's ZMQ sockets.

- `StatusBroadcaster` (`status.py`) subscribes once to the daemon's status PUB socket, validates each tick into a `DaemonStatus`, and fans the JSON out to every connected websocket. Doing it once avoids N redundant subscriptions and N redundant `model_validate()` calls for N tabs.

- `CommandListener` (`commands.py`) PUSHes in the other direction. Each request goes through three steps, in the order they appear in the file: `check` it against the latest status, `encode` it into daemon lines, and send. Note the asymmetry: status is JSON end-to-end, but the daemon's command socket takes lines, a verb followed by arguments, with structured payloads as a single JSON argument (`update_settings {...}`). It's kept that way on purpose, so it stays typeable by hand. `EmergencyStop` is the exception — it goes to the controller's dedicated e-stop socket on 5567 instead, so it can't end up queued behind whatever the daemon is currently blocked on.

## Frontend

Main file: `srt/svelte-frontend/src/routes/monitor/+page.svelte`

Written in **Svelte 5** with **Threlte** (a Svelte wrapper around Three.js) for the 3D sky view, replacing the original `dash` + `plotly` dashboard — which functioned, but was really painful to add new functionality to.

The sky map does not use a normal perspective camera. A `THREE.PerspectiveCamera` diverges badly as its field of view approaches 180°, so the whole scene is drawn with a **stereographic projection** implemented as custom vertex shaders. That decision has consequences that will bite you if you don't know about it — anything relying on Three's built-in camera math (frustum culling, raycasting) silently misbehaves, because that machinery has no idea our shaders overrode the projection.

The math lives in `src/lib/shaders/chunks/stereographic.glsl`, which every custom-shader layer `#include`s. There are two projection functions in there and picking the wrong one causes real, specific bugs, not just style inconsistency — the comments in that file say which is which and why. `src/lib/stores/projection.svelte.ts` holds the JS-side mirrors, needed because `Raycaster` and `camera.projectionMatrixInverse` don't know about the custom projection either.

Before changing anything in `src/lib/`, read the header comments in whichever file you're touching. Most of them document a bug that has already happened.

---

# Development

## Regenerating TypeScript types

Main file: `srt/tools/generate_ts_types.py`

Pydantic models → JSON Schema → `json-schema-to-typescript` → `srt/svelte-frontend/src/lib/generated/types.ts`. Never hand-write a TS interface for anything that crosses the python/browser boundary.

This runs automatically on dev server start (a Vite plugin in `vite.config.ts` calls it from `buildStart`), which makes "the TS types are stale" structurally impossible to forget. Set `SKIP_TYPE_GEN=1` to bypass it.

## Tests

```bash
pytest
```

`tests/test_command_encoding.py` covers the command translation layer, which is worth testing because mistakes there are silent — the daemon logs "Command Not Identified" into a status field and carries on, so a button in the UI just quietly does nothing.

`tests/test_config.py` validates the real `config/config.yaml` against the `DaemonConfig` model, so a field added to the model without a matching config entry fails here rather than at daemon startup on the roof.

`tests/test_settings.py` covers the runtime settings: layering over the defaults file, round-trips, partial updates, and rejection of typos and stale keys.

## Configuration: static vs runtime

The files in `config/`, split by who writes them:

- **`config.yaml`** — hardware and site facts, edited by hand, read once at startup (`DaemonConfig`). Mount limits, stow and horizon stay here deliberately: they're safety values, and a web UI shouldn't be able to widen them.
- **`settings.yaml`** — knobs an operator changes mid-session: pointing deadbands, scan dwell, the Siglent's live-view settings, observing policy, motor servo gains and limits (`RuntimeSettings`). The LPR encoder counts and phase offsets stay in `config.yaml` as calibration results. LPR edits are staged and reach the controller at the next encoder calibration, since it only accepts them as part of that sequence. Defaults live in the checked-in `config/settings.defaults.yaml`, hand-edited; `settings.yaml` holds only the daemon's overrides of them, written when the web app sends an edit, and is gitignored and optional. Because it holds only overrides, changing a default reaches every daemon that hasn't overridden it. `observing` is empty in the defaults until someone fills in the per-band spectrum settings; the file shows the shape.

`config.yaml` rejects unknown keys, so a setting left behind after it moved fails at startup rather than silently doing nothing. To convert an old-style `config.yaml`, run `python scripts/migrate_settings.py config/config.yaml`.

## Dependencies

```bash
python scripts/check_env_deps.py
```

Checks that every third-party module imported anywhere is actually declared in `environment.yml`. This exists because an import can work fine locally purely because some other package happened to pull it in transitively, and then fails on a fresh install — `pyyaml` was missing from `environment.yml` for months this way.

`environment.yml` is the only dependency list. Don't add a second one.

## CI

`.github/workflows/ci.yml` runs the dependency check and the tests, then regenerates `types.ts` and fails if the result differs from what's committed.

---

# What we're building toward

Reproduced verbatim from [Saren's software wishlist](https://docs.google.com/document/d/1FgWfWULLtUcSPFVzDWByMROSDIdOM89Jn5ZimKV1khU/preview?tab=t.0) — this is the science side of what the observation-planning work is ultimately for. Nothing below is implemented yet.

Routines:

1. Calibration
   1. Should be pretty standard across all observing modes
   2. Perform Y-factor (hot-cold) measurements to get absolute power calibration of the receiver.
      1. Hot sources would be stuff like CasA, CygA, etc. Basically we should have a list of hot calibrators in order of preference and the code chooses which sources are visible and uses the most preferred visible source. The one thing to be careful of is that "above the horizon" (esp for sources like CasA and CygA) might still mean that its near or behind the mountains, and if we're accidentally pointing at a mountain, we'll be looking at a 300K source (which basically means that our Treceiver value that we measure would be wildly lower than what it actually is).
      2. Cold source would basically be any patch of empty sky, and usually pointing a handful of degrees away from the source has proven to be sufficient. Various sky surveys exist that can be used to determine the precise brightness temperature of the sky at that location for as accurate of a calibration as possible. I'm a little concerned that our hot-cold measurements varied so strongly based on where in the sky we pointed, so we need to figure out why our calibrations are so non-reproducible at the moment.
      3. Calibration cadence should be determined by setting a threshold on receiver noise variation and calibrate at a cadence some factor below that.
         1. e.g. receiver noise drifts by 1K every hour and we want to be a factor of 10 faster than that, so we would calibrate every 6 minutes (this is probably too aggressive)
2. Targeted observation
   1. User required to provide: source name (or ra/dec, or something else depending on what the user is trying to observe), observing band (can only choose one of "L", "S", "C" for now), observing schedule (when/how long), and data format (raw spectra, flux density, brightness temperature, stokes parameters, etc. along with desired data time resolution)
   2. User not required to but can provide: observing file with manual pointings for calibration and observation
   3. (will add more stuff here, currently can't think of much more though)
3. Observing modes
   1. Static point: park dish at az, el setpoint manually determined by user (e.g. geostationary satellite, terrestrial source, etc.)
   2. Drift scan: also parks dish at az, el setpoint
4. Surveys
   1. Basically whenever the instrument is not being used for targeted observation or under maintenance/upgrades, it should be running an all-sky survey. We'll start this at L band and move our way up over time to S and C bands.
   2. It may be interesting to supplement these wideband surveys with spectral line surveys (HI, methanol maser (if possible), etc.), but this would just be a narrowband version of what the continuum surveys do.
5. Instrumental efforts
   1. After building the receiver, certain measurements we can do to better understand and characterize the telescope are:
      1. Drift scans on the sun (or CasA, etc.) to map our antenna beam
      2. Cross-correlations of X-Y polarizations to get crosstalk measurements
      3. (will think of more measurements to do as we go along)
6. Important considerations for observing:
   1. Our linearly polarized beam rotates on our source as we track with an az-el mount (just a consequence of not using a polar axis mount), so we need to make sure we account for this in software for long integrations/tracking.
   2. Because of above effects, both polarizations need to be absolutely calibrated to within some threshold (otherwise what might be gain drifts due to mixed polarization states may look like the source flux density changing)
7. Upgrade paths
   1. 2-3 dish interferometer: once I get my dishes set up at home (likely starting with a C band receiver, but I haven't finalized stuff yet), we can start "imaging" on the sky (I'm assuming data rates will be slow enough where we can send raw voltages from my house to the Moore roof and cross-correlate there, this may have to be done asynchronously). Ideally we could add some E-W baselines in the future as well to be able to get some nicer 2D uv coverage.
   2. Wideband receiver: the current plan is to have L, S, and C band receivers (Qorvo's LNA chip we want to use caps out at 6 GHz, so we're only really getting half of C band), but the feed works from L-X band, so we could potentially add an X-band receiver if we can find a wideband LNA (ideally would not switch LNAs but would have one wideband LNA feeding into multiple switched receiver paths)

---

# Footnotes

[^1]: The original `srt-py` code uses a software defined radio (SDR), specifically the RTL-SDR, along with *GNU Radio*, a python-based signal processing library, to collect the science data and pipe it to the web app for display / to a recording file on disk. Since we're using somewhat more sophisticated receiving equipment, I've torn all of that code out, in favor of the pyVISA USB interface for our spectrum analyzer. If SDRs become useful for us again, we can add GNU Radio back into the pipeline.

[^2]: ZMQ = zero-broker messaging queue, an asynchronous messaging library. Convenient method for inter-thread communication without any extra overhead (like a separate 'broker' program running to convey the messages). GNU Radio uses it extensively, so the original code was naturally built around it. For us, it ain't broke.
