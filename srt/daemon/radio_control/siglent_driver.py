"""siglent_driver.py

Threaded Siglent spectrum analyzer driver for the daemon process.
"""

from __future__ import annotations

import copy
import logging
import time
from collections import deque
from threading import Event, RLock, Thread
from typing import Any, Dict, List, Optional

import numpy as np
import pyvisa

from ..types import SpectrumConfig, SpectrumFrame


class SiglentDriver:
    """Owns VISA connection and acquisition loop for a Siglent analyzer."""

    def __init__(self, config: SpectrumConfig, history_limit: int = 200):
        self._config = SpectrumConfig.from_dict(
            config.to_dict() if isinstance(config, SpectrumConfig) else config
        )
        self._config_lock = RLock()

        self._latest: Optional[SpectrumFrame] = None
        self._frame_lock = RLock()
        self._history: deque[SpectrumFrame] = deque(maxlen=history_limit)

        self._average_buffer_mw: List[np.ndarray] = []

        self._reconfigure_flag = Event()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None

        self._connected = False
        self._connection_lock = RLock()
        self._sweep_index = 0

    # ------------------------------------------------------------------
    # Public API
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

    def update_config(self, partial_dict: Dict[str, Any]) -> None:
        if not partial_dict:
            return
        with self._config_lock:
            merged = self._config.to_dict()
            merged.update(partial_dict)
            new_config = SpectrumConfig.from_dict(merged)
            if new_config != self._config:
                self._config = new_config
                self._reconfigure_flag.set()

    def get_config(self) -> SpectrumConfig:
        with self._config_lock:
            return copy.copy(self._config)

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
                config = self.get_config()
                resource_name = self._find_instrument_by_serial(config.instrument_serial)

                rm = pyvisa.ResourceManager()
                inst = rm.open_resource(resource_name)
                inst.timeout = 60000
                inst.write_termination = "\n"
                inst.read_termination = "\n"

                logging.info("SiglentDriver connected to %s", resource_name)
                try:
                    logging.info("Siglent ID: %s", inst.query("*IDN?").strip())
                except Exception:
                    pass

                self._configure_instrument(inst, config)
                self._average_buffer_mw = []
                self._sweep_index = 0
                self._set_connected(True)

                while not self._stop_event.is_set():
                    if self._reconfigure_flag.is_set():
                        config = self.get_config()
                        self._configure_instrument(inst, config)
                        self._average_buffer_mw = []
                        self._sweep_index = 0
                        self._reconfigure_flag.clear()

                    freq_hz, raw_dbm = self._acquire_one_trace(inst)
                    self._sweep_index += 1

                    power_dbm, avg_count = self._apply_averaging(
                        raw_dbm, config.trace_type, config.num_averages
                    )

                    frame = SpectrumFrame(
                        freq_hz=freq_hz,
                        power_dbm=power_dbm,
                        raw_dbm=raw_dbm,
                        sweep_index=self._sweep_index,
                        timestamp=time.time(),
                        config=config,
                        avg_count=avg_count,
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

    def _configure_frequency(self, inst, config: SpectrumConfig) -> None:
        mode = config.freq_mode.lower().strip()
        if mode == "start_stop":
            inst.write(f":FREQ:STAR {config.start_hz}")
            inst.write(f":FREQ:STOP {config.stop_hz}")
        elif mode == "center_span":
            inst.write(f":FREQ:CENT {config.center_hz}")
            inst.write(f":FREQ:SPAN {config.span_hz}")
        else:
            raise ValueError("freq_mode must be 'start_stop' or 'center_span'.")

    def _configure_instrument(self, inst, config: SpectrumConfig) -> None:
        self._configure_frequency(inst, config)

        inst.write(f":BAND {config.rbw_hz}")
        inst.write(f":BAND:VID {config.vbw_hz}")
        inst.write(f":DISP:WIND:TRAC:Y:RLEV {config.ref_level_dbm}")

        inst.write(":SWE:TIME:AUTO ON")
        inst.write(":INIT:CONT OFF")
        inst.write(":FORM:TRAC:DATA ASCii")

        if config.atten_auto:
            self._scpi_write_ignore_error(inst, ":POW:ATT:AUTO ON")
        else:
            self._scpi_write_ignore_error(inst, ":POW:ATT:AUTO OFF")
            self._scpi_write_ignore_error(inst, f":POW:ATT {config.atten_db}")

        if config.preamp_on is True:
            self._scpi_write_ignore_error(inst, ":POW:GAIN ON")
        elif config.preamp_on is False:
            self._scpi_write_ignore_error(inst, ":POW:GAIN OFF")

        trace_type = config.trace_type.lower().strip()
        if trace_type in {"clear_write", "average"}:
            self._scpi_write_ignore_error(inst, ":TRAC1:MODE WRIT")
        else:
            raise ValueError("trace_type must be 'clear_write' or 'average'.")

        time.sleep(0.2)

    def _acquire_one_trace(self, inst) -> tuple[np.ndarray, np.ndarray]:
        inst.write(":INIT:IMM")
        inst.query("*OPC?")

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

        return freq_hz, power_dbm

    def _apply_averaging(
        self, raw_dbm: np.ndarray, trace_type: str, num_averages: int
    ) -> tuple[np.ndarray, int]:
        if trace_type.lower().strip() != "average":
            return raw_dbm, 1

        current_mw = self._dbm_to_mw(raw_dbm)
        self._average_buffer_mw.append(current_mw)

        if len(self._average_buffer_mw) > max(1, int(num_averages)):
            self._average_buffer_mw.pop(0)

        lengths = [len(arr) for arr in self._average_buffer_mw]
        if len(set(lengths)) != 1:
            self._average_buffer_mw = [current_mw]

        avg_mw = np.mean(np.stack(self._average_buffer_mw, axis=0), axis=0)
        return self._mw_to_dbm(avg_mw), len(self._average_buffer_mw)
