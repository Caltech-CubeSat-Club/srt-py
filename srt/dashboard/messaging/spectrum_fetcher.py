"""spectrum_fetcher.py

Thread Which Handles Receiving Spectrum Data from Siglent Spectrum Analyzer via PyVISA

"""

import numpy as np
from threading import Thread
from time import sleep
import time
import pyvisa


class SpectrumThread(Thread):
    """
    Thread for Fetching Spectrum Data from Siglent SSA3021X-Plus via PyVISA
    """

    def __init__(
        self,
        group=None,
        target=None,
        name=None,
        instrument_serial="SSA3PCED7R1040",
        start_hz=1415.5e6,
        stop_hz=1425.5e6,
        rbw_hz=1e3,
        vbw_hz=1e2,
        ref_level_dbm=-80,
        history_length=1000,
    ):
        """Initializer for the SpectrumThread

        Parameters
        ----------
        group : NoneType
            The ThreadGroup the Thread Belongs to (Currently Unimplemented in Python 3.8)
        target : callable
            Function that the Thread Should Run (Leave This Be For Command Sending)
        name : str
            Name of the Thread
        instrument_serial : str
            Serial number of the Siglent spectrum analyzer
        start_hz : float
            Starting frequency in Hz
        stop_hz : float
            Stopping frequency in Hz
        rbw_hz : float
            Resolution bandwidth in Hz
        vbw_hz : float
            Video bandwidth in Hz
        ref_level_dbm : float
            Reference level in dBm
        history_length : int
            Max Length of Spectrum Data History List
        """
        super().__init__(group=group, target=target, name=name, daemon=True)
        self.history_length = history_length
        self.spectrum = None
        self.history = []
        self.instrument_serial = instrument_serial
        self.start_hz = start_hz
        self.stop_hz = stop_hz
        self.rbw_hz = rbw_hz
        self.vbw_hz = vbw_hz
        self.ref_level_dbm = ref_level_dbm
        self.inst = None

    def _find_instrument_by_serial(self, serial_text):
        """Find spectrum analyzer by serial number"""
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()

        matches = [
            r for r in resources
            if r.startswith("USB") and serial_text in r
        ]

        if not matches:
            raise RuntimeError(
                f"No matching instrument found.\n"
                f"Available resources:\n" + "\n".join(resources)
            )

        if len(matches) > 1:
            raise RuntimeError(
                f"Multiple matching instruments found:\n"
                + "\n".join(matches)
            )

        return matches[0]

    def _configure_analyzer(self):
        """Configure the spectrum analyzer sweep parameters"""
        self.inst.write(f":FREQ:STAR {self.start_hz}")
        self.inst.write(f":FREQ:STOP {self.stop_hz}")
        self.inst.write(f":BAND {self.rbw_hz}")
        self.inst.write(f":BAND:VID {self.vbw_hz}")
        self.inst.write(f":DISP:WIND:TRAC:Y:RLEV {self.ref_level_dbm}")
        self.inst.write(":INIT:CONT ON")  # Continuous sweep mode
        self.inst.write(":FORM:TRAC:DATA ASCii")

    def _acquire_trace(self):
        """Acquire one spectrum trace and convert to numpy array"""
        self.inst.write(":INIT:IMM")
        self.inst.query("*OPC?")

        raw = self.inst.query(":TRAC:DATA?")
        spectrum = np.array([
            float(v)
            for v in raw.strip().split(",")
            if v.strip()
        ])

        return spectrum

    def run(self):
        """Acquires Spectrum Traces from Analyzer, Converts to Numpy, and Stores

        Returns
        -------
        None
        """
        try:
            rm = pyvisa.ResourceManager()
            resource_name = self._find_instrument_by_serial(
                self.instrument_serial
            )

            self.inst = rm.open_resource(resource_name)
            self.inst.timeout = 60000
            self.inst.write_termination = "\n"
            self.inst.read_termination = "\n"

            print(f"Connected to: {self.inst.query('*IDN?')}")

            self._configure_analyzer()

            while True:
                try:
                    spectrum = self._acquire_trace()

                    if len(self.history) >= self.history_length:
                        self.history.pop()

                    self.history.insert(0, (time.time(), spectrum))
                    self.spectrum = spectrum

                except Exception as e:
                    print(f"Error acquiring trace: {e}")
                    sleep(1)

        except Exception as e:
            print(f"Fatal error in SpectrumThread: {e}")
        finally:
            if self.inst:
                self.inst.close()

    def get_spectrum(self):
        """Return Most Recently Received Spectrum

        Returns
        -------
        self.spectrum : (N) ndarray
            Spectrum data in dBm
        """
        return self.spectrum

    def get_history(self):
        """Return Entire History List

        Returns
        -------
        [(float, ndarray)]
            Time and Numpy Spectrum Pairs History
        """
        return self.history.copy()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    thread = SpectrumThread()
    thread.start()
    sleep(2)
    print(thread.get_spectrum())
    data = thread.get_spectrum()
    if data is not None:
        plt.plot(range(len(data)), data)
        plt.xlabel("Frequency Bin")
        plt.ylabel("Power (dBm)")
        plt.show()
