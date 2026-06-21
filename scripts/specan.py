"""
Siglent SSA3021X-Plus
Dual-subband averaged spectrum acquisition with:

- Alternating sub-band sweeps
- Time-based or sweep-count-based averaging
- Preamp ON/OFF control
- Manual or automatic attenuation
- Automatic overload recovery
- Final simultaneous plotting
- CSV export

Tested conceptually against SSA3000X SCPI structure.

Requires:
    pip install pyvisa numpy matplotlib
"""

import math
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import pyvisa


# ============================================================
# USER SETTINGS
# ============================================================

INSTRUMENT_SERIAL = "SSA3PCED7R1040"

# Averaging mode:
#   "time"   -> integrate for INTEGRATION_TIME_S
#   "sweeps" -> acquire NUM_SWEEPS_PER_BAND sweeps
AVERAGING_MODE = "sweeps"

INTEGRATION_TIME_S = 2
NUM_SWEEPS_PER_BAND = 30

# ============================================================
# FRONT-END SETTINGS
# ============================================================

# RF preamp:
#   True  -> ON
#   False -> OFF
PREAMP_ENABLE = True

# Attenuation:
#   True  -> analyzer auto attenuation
#   False -> manual attenuation
AUTO_ATTENUATION = False

# Starting attenuation if AUTO_ATTENUATION = False
INITIAL_ATTENUATION_DB = 0

# Overload handling
AUTO_RECOVER_OVERLOAD = False
ATTENUATION_STEP_DB = 1
MAX_ATTENUATION_DB = 31

# ============================================================
# SWEEP SETTINGS
# ============================================================

AUTO_SWEEP_TIME = True
MANUAL_SWEEP_TIME_S = 4

SETTLE_TIME_S = 0.5

# ============================================================
# SUBBANDS
# ============================================================

SUBBANDS = [
  #  {
     #   "name": "1420.5_to_1427_MHz",
    #    "start_hz": 1590e6,
   #     "stop_hz": 1610e6,
  #      "rbw_hz": 1e3,
  #      "vbw_hz": 10e1,
  #      "ref_level_dbm": -70,
 #   },
    {
        "name": "1419_to_1422_MHz",
        "start_hz": 1415.5e6,
        "stop_hz": 1425.5e6,
        "rbw_hz": 1e3,
        "vbw_hz": 1e2,
        "ref_level_dbm": -82.8,
    },
]


# ============================================================
# DATA STRUCTURE
# ============================================================

@dataclass
class AveragedSpectrum:
    name: str
    x_hz: np.ndarray
    avg_dbm: np.ndarray
    avg_mw: np.ndarray
    sweep_count: int
    elapsed_s: float
    final_attenuation_db: float
    sweep_time_s: float


# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def dbm_to_mw(dbm):
    return 10 ** (dbm / 10.0)


def mw_to_dbm(mw):
    mw = np.maximum(mw, 1e-30)
    return 10 * np.log10(mw)


def find_instrument_by_serial(serial_text):
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


# ============================================================
# FRONT-END CONTROL
# ============================================================

def set_frontend(inst, attenuation_db):
    """
    Configure preamp and attenuation.
    """

    # Preamp
    if PREAMP_ENABLE:
        inst.write(":POW:GAIN ON")
    else:
        inst.write(":POW:GAIN OFF")

    # Attenuation
    if AUTO_ATTENUATION:
        inst.write(":POW:ATT:AUTO ON")
    else:
        inst.write(":POW:ATT:AUTO OFF")
        inst.write(f":POW:ATT {attenuation_db}")


def query_current_attenuation(inst):
    return float(inst.query(":POW:ATT?"))


# ============================================================
# OVERLOAD DETECTION
# ============================================================

def instrument_has_overload(inst):
    """
    Try several methods to detect overload.
    """

    # Method 1: system error queue
    try:
        err = inst.query(":SYST:ERR?").strip().lower()

        overload_keywords = [
            "adc",
            "overload",
            "overflow",
            "if overload",
        ]

        for keyword in overload_keywords:
            if keyword in err:
                print(f"Detected overload/system error: {err}")
                return True

    except Exception:
        pass

    return False


# ============================================================
# SWEEP CONFIGURATION
# ============================================================

def configure_subband(inst, band):
    """
    Configure one sub-band.
    Returns analyzer-reported sweep time.
    """

    inst.write(f":FREQ:STAR {band['start_hz']}")
    inst.write(f":FREQ:STOP {band['stop_hz']}")

    inst.write(f":BAND {band['rbw_hz']}")
    inst.write(f":BAND:VID {band['vbw_hz']}")

    inst.write(f":DISP:WIND:TRAC:Y:RLEV {band['ref_level_dbm']}")

    set_frontend(inst, INITIAL_ATTENUATION_DB)

    inst.write(":INIT:CONT OFF")

    inst.write(":FORM:TRAC:DATA ASCii")

    if AUTO_SWEEP_TIME:
        inst.write(":SWE:TIME:AUTO ON")
    else:
        inst.write(":SWE:TIME:AUTO OFF")
        inst.write(f":SWE:TIME {MANUAL_SWEEP_TIME_S}")

    time.sleep(SETTLE_TIME_S)

    sweep_time_s = float(inst.query(":SWE:TIME?"))

    return sweep_time_s


# ============================================================
# TRACE ACQUISITION
# ============================================================

def acquire_trace_with_overload_protection(inst):
    """
    Acquire one trace with automatic attenuation stepping.
    """

    current_att = INITIAL_ATTENUATION_DB

    while True:

        set_frontend(inst, current_att)

        inst.write(":INIT:IMM")
        inst.query("*OPC?")

        overload = instrument_has_overload(inst)

        if not overload:
            break

        if not AUTO_RECOVER_OVERLOAD:
            raise RuntimeError(
                "ADC overload detected."
            )

        current_att += ATTENUATION_STEP_DB

        if current_att > MAX_ATTENUATION_DB:
            raise RuntimeError(
                f"Unable to clear overload before "
                f"{MAX_ATTENUATION_DB} dB attenuation."
            )

        print(
            f"Overload detected -> increasing attenuation "
            f"to {current_att} dB"
        )

    raw = inst.query(":TRAC:DATA?")

    y_dbm = np.array([
        float(v)
        for v in raw.strip().split(",")
        if v.strip()
    ])

    return y_dbm, current_att


# ============================================================
# FREQUENCY AXIS
# ============================================================

def get_frequency_axis(inst, point_count):
    start_hz = float(inst.query(":FREQ:STAR?"))
    stop_hz = float(inst.query(":FREQ:STOP?"))

    return np.linspace(start_hz, stop_hz, point_count)


# ============================================================
# SWEEP COUNT DETERMINATION
# ============================================================

def determine_sweeps_per_band(inst, band):
    sweep_time_s = configure_subband(inst, band)

    if AVERAGING_MODE.lower() == "time":

        sweeps = math.ceil(
            INTEGRATION_TIME_S / sweep_time_s
        )

    elif AVERAGING_MODE.lower() == "sweeps":

        sweeps = NUM_SWEEPS_PER_BAND

    else:
        raise ValueError(
            "AVERAGING_MODE must be "
            "'time' or 'sweeps'"
        )

    sweeps = max(1, int(sweeps))

    return sweeps, sweep_time_s


# ============================================================
# CSV OUTPUT
# ============================================================

def save_spectrum_csv(result):
    filename = f"{result.name}_averaged.csv"

    data = np.column_stack((
        result.x_hz,
        result.avg_dbm,
        result.avg_mw
    ))

    np.savetxt(
        filename,
        data,
        delimiter=",",
        header=(
            "frequency_hz,"
            "average_amplitude_dbm,"
            "average_power_mw"
        ),
        comments=""
    )

    print(f"Saved: {filename}")


# ============================================================
# PLOTTING
# ============================================================

def plot_results(results):

    fig, axes = plt.subplots(
        nrows=len(results),
        ncols=1,
        figsize=(10, 4 * len(results)),
        constrained_layout=True
    )

    if len(results) == 1:
        axes = [axes]

    for ax, result in zip(axes, results):

        ax.plot(
            result.x_hz / 1e6,
            result.avg_dbm
        )

        ax.set_title(
            f"{result.name} | "
            f"{result.sweep_count} sweeps | "
            f"{result.elapsed_s:.1f} s | "
            f"final ATT={result.final_attenuation_db:.1f} dB"
        )

        ax.set_xlabel("Frequency (MHz)")
        ax.set_ylabel("Average amplitude (dBm)")
        ax.grid(True)

    plt.show()


# ============================================================
# MAIN
# ============================================================

def main():

    rm = pyvisa.ResourceManager()

    resource_name = find_instrument_by_serial(
        INSTRUMENT_SERIAL
    )

    print(f"Opening: {resource_name}")

    inst = rm.open_resource(resource_name)

    inst.timeout = 60000
    inst.write_termination = "\n"
    inst.read_termination = "\n"

    try:

        print(inst.query("*IDN?"))

        # Determine sweep counts
        sweep_plan = {}

        for band in SUBBANDS:

            sweeps, sweep_time_s = determine_sweeps_per_band(
                inst,
                band
            )

            sweep_plan[band["name"]] = {
                "count": sweeps,
                "sweep_time_s": sweep_time_s
            }

        print("\nAcquisition plan:")

        for band in SUBBANDS:

            plan = sweep_plan[band["name"]]

            print(
                f"  {band['name']}: "
                f"{plan['count']} sweeps "
                f"(sweep time = {plan['sweep_time_s']:.3f} s)"
            )

        # ====================================================
        # ACCUMULATORS
        # ====================================================

        accumulators_mw = {}
        frequency_axes_hz = {}

        completed_sweeps = {
            band["name"]: 0
            for band in SUBBANDS
        }

        final_attenuations = {}

        start_times = {}
        end_times = {}

        # ====================================================
        # ALTERNATING ACQUISITION
        # ====================================================

        acquisition_active = True

        while acquisition_active:

            acquisition_active = False

            for band in SUBBANDS:

                name = band["name"]

                target_sweeps = sweep_plan[name]["count"]

                if completed_sweeps[name] >= target_sweeps:
                    continue

                acquisition_active = True

                if name not in start_times:
                    start_times[name] = time.monotonic()

                configure_subband(inst, band)

                y_dbm, used_att = (
                    acquire_trace_with_overload_protection(inst)
                )

                y_mw = dbm_to_mw(y_dbm)

                if name not in accumulators_mw:

                    accumulators_mw[name] = np.zeros_like(y_mw)

                    frequency_axes_hz[name] = (
                        get_frequency_axis(
                            inst,
                            len(y_mw)
                        )
                    )

                accumulators_mw[name] += y_mw

                completed_sweeps[name] += 1

                final_attenuations[name] = used_att

                end_times[name] = time.monotonic()

                print(
                    f"{name}: "
                    f"{completed_sweeps[name]}/"
                    f"{target_sweeps} sweeps",
                    end="\r"
                )

        print("\nAcquisition complete.")

        # ====================================================
        # FINALIZE RESULTS
        # ====================================================

        results = []

        for band in SUBBANDS:

            name = band["name"]

            count = completed_sweeps[name]

            avg_mw = accumulators_mw[name] / count

            avg_dbm = mw_to_dbm(avg_mw)

            elapsed_s = (
                end_times[name]
                - start_times[name]
            )

            result = AveragedSpectrum(
                name=name,
                x_hz=frequency_axes_hz[name],
                avg_dbm=avg_dbm,
                avg_mw=avg_mw,
                sweep_count=count,
                elapsed_s=elapsed_s,
                final_attenuation_db=final_attenuations[name],
                sweep_time_s=sweep_plan[name]["sweep_time_s"]
            )

            results.append(result)

            save_spectrum_csv(result)

        # ====================================================
        # PLOT
        # ====================================================

        plot_results(results)

    finally:

        inst.close()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()