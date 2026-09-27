"""siglent_driver.py

Threaded Siglent spectrum analyzer driver for the daemon process.

Two faces: the free-running live view (start/stop/get_latest/set_settings)
the daemon uses today, and the SpectrumDriver contract observation routines
will use. Only the first works yet.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from threading import Event, RLock, Thread
from typing import List, Optional

import numpy as np
import pyvisa

from ..telescope_types import FrameMetadata, SpecanSettings, SpectrumFrame
from .driver import DriverCapabilities, SpectrumDriver


class SiglentDriver(SpectrumDriver[SpecanSettings]):
    """Owns VISA connection and acquisition loop for a Siglent analyzer."""

    def __init__(self, serial: str, settings: SpecanSettings, history_limit: int = 200):
        self._serial = serial
        self._settings = settings
        self._settings_lock = RLock()

        self._latest: Optional[SpectrumFrame] = None
        self._frame_lock = RLock()
        self._history: deque[SpectrumFrame] = deque(maxlen=history_limit)

        # (power in mW, how long that sweep took) for each sweep in the average.
        self._average_buffer: List[tuple[np.ndarray, float]] = []

        self._reconfigure_flag = Event()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None

        self._connected = False
        self._connection_lock = RLock()
        self._sweep_index = 0

    # ------------------------------------------------------------------
    # Live view
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._run, name="siglent-driver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5)
        self._set_connected(False)

    def set_settings(self, settings: SpecanSettings) -> None:
        """Swap in new instrument settings; the acquisition loop reconfigures
        before its next sweep. Merging and validating partial updates is the
        daemon's job — see settings.merge_settings."""
        with self._settings_lock:
            if settings != self._settings:
                self._settings = settings
                self._reconfigure_flag.set()

    def get_settings(self) -> SpecanSettings:
        # Settings are replaced, never mutated, so the reference is safe to hand out.
        with self._settings_lock:
            return self._settings

    def get_latest(self) -> Optional[SpectrumFrame]:
        with self._frame_lock:
            return self._latest

    def get_history(self, limit: int = 100) -> List[SpectrumFrame]:
        with self._frame_lock:
            if limit <= 0:
                return []
            return list(self._history)[-limit:]

    @property
    def connected(self) -> bool:
        with self._connection_lock:
            return bool(self._connected)

    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    # ------------------------------------------------------------------
    # SpectrumDriver contract
    # ------------------------------------------------------------------

    @property
    def capabilities(self) -> DriverCapabilities:
        # One trace per sweep, so one polarization; everything but stokes.
        return DriverCapabilities(
            driver="specan",
            polarizations=1,
            supported_output_formats=[
                "raw_spectra",
                "power_spectral_density",
                "flux_density",
                "brightness_temperature",
            ],
        )

    def integration_duration(self, settings: SpecanSettings) -> float:
        """`num_averages * sweep_time_seconds`. Not implemented because the
        answer would be wrong: _configure_instrument sends :SWE:TIME:AUTO ON,
        so the instrument picks sweep time and ignores sweep_time_seconds.
        Fix that first, and require sweep_time_seconds to be set.
        """
        raise NotImplementedError

    def do_one_integration(
        self,
        settings: SpecanSettings,
        metadata: FrameMetadata,
    ) -> SpectrumFrame:
        """Not implemented. The acquisition thread owns the VISA session, so
        this can't open its own — it has to take over that loop: apply
        `settings`, clear the averaging buffer, average `num_averages`
        sweeps (whatever `trace_type` says — that's a live-view display
        choice), and return the frame with `metadata` attached. Meanwhile
        the live view either pauses or shows these frames.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_connected(self, value: bool) -> None:
        with self._connection_lock:
            self._connected = bool(value)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            inst = None
            rm = None
            try:
                self._reconfigure_flag.clear()
                settings = self.get_settings()
                resource_name = self._find_instrument_by_serial(self._serial)

                rm = pyvisa.ResourceManager()
                inst = rm.open_resource(resource_name)
                inst.timeout = 60000
                inst.write_termination = "\n" # pyright: ignore[reportAttributeAccessIssue]
                inst.read_termination = "\n" # pyright: ignore[reportAttributeAccessIssue]

                logging.info("SiglentDriver connected to %s", resource_name)
                try:
                    logging.info("Siglent ID: %s", inst.query("*IDN?").strip()) # pyright: ignore[reportAttributeAccessIssue]
                except Exception:
                    pass

                self._configure_instrument(inst, settings)
                self._average_buffer = []
                self._sweep_index = 0
                self._set_connected(True)

                while not self._stop_event.is_set():
                    if self._reconfigure_flag.is_set():
                        # Clear before reading, so an update landing mid-configure re-arms it.
                        self._reconfigure_flag.clear()
                        settings = self.get_settings()
                        self._configure_instrument(inst, settings)
                        self._average_buffer = []
                        self._sweep_index = 0

                    freq_hz, raw_dbm, sweep_seconds = self._acquire_one_trace(inst)
                    self._sweep_index += 1

                    power_dbm, avg_count, integration_seconds = self._apply_averaging(
                        raw_dbm, sweep_seconds, settings.trace_type, settings.num_averages
                    )

                    frame = SpectrumFrame(
                        freq_hz=freq_hz.tolist(),
                        power_dbm=power_dbm.tolist(),
                        raw_dbm=raw_dbm.tolist(),
                        sweep_index=self._sweep_index,
                        timestamp=time.time(),
                        config=settings,
                        avg_count=avg_count,
                        integration_seconds=integration_seconds,
                    )

                    with self._frame_lock:
                        self._latest = frame
                        self._history.append(frame)

            except Exception as exc:
                logging.warning("SiglentDriver error: %s", exc)
                self._set_connected(False)
                time.sleep(2.0)
            finally:
                self._set_connected(False)
                if inst is not None:
                    try:
                        inst.close()
                    except Exception:
                        pass
                if rm is not None:
                    try:
                        rm.close()
                    except Exception:
                        pass

    @staticmethod
    def _find_instrument_by_serial(serial_text: str) -> str:
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()

        matches = [
            resource
            for resource in resources
            if resource.startswith("USB") and serial_text in resource
        ]

        if not matches:
            raise RuntimeError(
                "No USB instrument found containing serial text: "
                f"{serial_text}\nAvailable VISA resources:\n" + "\n".join(resources)
            )

        if len(matches) > 1:
            raise RuntimeError(
                "Multiple USB instruments matched serial text: "
                f"{serial_text}\nMatches:\n" + "\n".join(matches)
            )

        return matches[0]

    @staticmethod
    def _dbm_to_mw(dbm: np.ndarray) -> np.ndarray:
        return 10 ** (dbm / 10.0)

    @staticmethod
    def _mw_to_dbm(mw: np.ndarray) -> np.ndarray:
        mw = np.maximum(mw, 1e-30)
        return 10.0 * np.log10(mw)

    @staticmethod
    def _scpi_write_ignore_error(inst, command: str) -> None:
        try:
            inst.write(command)
        except Exception as exc:
            logging.debug("SCPI command failed: %s (%s)", command, exc)

    def _configure_frequency(self, inst, settings: SpecanSettings) -> None:
        # SpecanSettings' validator guarantees the pair for the mode is set.
        if settings.freq_mode == "start_stop":
            inst.write(f":FREQ:STAR {settings.start_hz}")
            inst.write(f":FREQ:STOP {settings.stop_hz}")
        else:
            inst.write(f":FREQ:CENT {settings.center_frequency_hz}")
            inst.write(f":FREQ:SPAN {settings.span_hz}")

    def _configure_instrument(self, inst, settings: SpecanSettings) -> None:
        self._configure_frequency(inst, settings)

        inst.write(f":BAND {settings.resolution_bandwidth_hz}")
        inst.write(f":BAND:VID {settings.video_bandwidth_hz}")
        inst.write(f":DISP:WIND:TRAC:Y:RLEV {settings.reference_level_dbm}")

        inst.write(":SWE:TIME:AUTO ON")
        inst.write(":INIT:CONT OFF")
        inst.write(":FORM:TRAC:DATA ASCii")

        if settings.attenuation_auto:
            self._scpi_write_ignore_error(inst, ":POW:ATT:AUTO ON")
        else:
            self._scpi_write_ignore_error(inst, ":POW:ATT:AUTO OFF")
            if settings.attenuation_db is not None:
                self._scpi_write_ignore_error(inst, f":POW:ATT {settings.attenuation_db}")

        if settings.preamp_on is True:
            self._scpi_write_ignore_error(inst, ":POW:GAIN ON")
        elif settings.preamp_on is False:
            self._scpi_write_ignore_error(inst, ":POW:GAIN OFF")

        # Both trace types acquire in write mode; averaging is done here in
        # _apply_averaging, not on the instrument.
        self._scpi_write_ignore_error(inst, ":TRAC1:MODE WRIT")

        time.sleep(0.2)

    def _acquire_one_trace(self, inst) -> tuple[np.ndarray, np.ndarray, float]:
        """One sweep: (freq_hz, power_dbm, seconds the sweep took). Timed
        around the trigger and completion wait only, so trace readout and
        the frequency queries don't count as integration."""
        started = time.monotonic()
        inst.write(":INIT:IMM")
        inst.query("*OPC?")
        sweep_seconds = time.monotonic() - started

        raw = inst.query(":TRAC:DATA?")
        power_dbm = np.array(
            [float(v) for v in raw.strip().split(",") if v.strip()],
            dtype=float,
        )
        if len(power_dbm) == 0:
            raise RuntimeError("Received empty trace from analyzer.")

        actual_start_hz = float(inst.query(":FREQ:STAR?"))
        actual_stop_hz = float(inst.query(":FREQ:STOP?"))
        freq_hz = np.linspace(actual_start_hz, actual_stop_hz, len(power_dbm))

        return freq_hz, power_dbm, sweep_seconds

    def _apply_averaging(
        self, raw_dbm: np.ndarray, sweep_seconds: float, trace_type: str, num_averages: int
    ) -> tuple[np.ndarray, int, float]:
        """(power_dbm, sweeps averaged, total seconds those sweeps took)."""
        if trace_type.lower().strip() != "average":
            return raw_dbm, 1, sweep_seconds

        current = (self._dbm_to_mw(raw_dbm), sweep_seconds)
        self._average_buffer.append(current)

        if len(self._average_buffer) > max(1, int(num_averages)):
            self._average_buffer.pop(0)

        lengths = {len(mw) for mw, _ in self._average_buffer}
        if len(lengths) != 1:
            self._average_buffer = [current]

        avg_mw = np.mean(np.stack([mw for mw, _ in self._average_buffer], axis=0), axis=0)
        integration_seconds = sum(secs for _, secs in self._average_buffer)
        return self._mw_to_dbm(avg_mw), len(self._average_buffer), integration_seconds
