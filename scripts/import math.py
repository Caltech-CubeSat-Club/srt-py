import math
import time
from dataclasses import dataclass

import pyvisa
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# User settings
# ============================================================

INSTRUMENT_SERIAL = "SSA3PCED7R1040"
Y_FAC_TEXT_FILENAME = "y_fac_only"

START_HZ = 1415.5e6
STOP_HZ = 1425.5e6

RBW_HZ = 1e3
VBW_HZ = 1e2
REF_LEVEL_DBM = -80

# Preferred mode:
#   "time"   -> integrate for INTEGRATION_TIME_S seconds
#   "sweeps" -> average NUM_SWEEPS sweeps
AVERAGING_MODE = "sweeps"

INTEGRATION_TIME_S = 30.0
NUM_SWEEPS = 30

SETTLE_TIME_S = 0.2

AUTO_SWEEP_TIME = True
MANUAL_SWEEP_TIME_S = 0.5

SAVE_CSV = True

SWEEP1_CSV_FILENAME = "integrated_sweep_1.csv"
SWEEP2_CSV_FILENAME = "integrated_sweep_2.csv"
Y_FACTOR_CSV_FILENAME = "y_factor_difference.csv"

# Difference convention:
#   "second_minus_first" -> delta_db = sweep2 - sweep1
#   "first_minus_second" -> delta_db = sweep1 - sweep2
DIFFERENCE_MODE = "first_minus_second"


# ============================================================
# Data containers
# ============================================================

@dataclass
class IntegratedSweep:
    name: str
    x_hz: np.ndarray
    avg_dbm: np.ndarray
    avg_mw: np.ndarray
    sweep_count: int
    sweep_time_s: float
    elapsed_time_s: float


# ============================================================
# Unit conversion helpers
# ============================================================

def dbm_to_mw(dbm: np.ndarray) -> np.ndarray:
    return 10 ** (dbm / 10.0)


def mw_to_dbm(mw: np.ndarray) -> np.ndarray:
    mw = np.maximum(mw, 1e-30)
    return 10.0 * np.log10(mw)


def db_to_linear_factor(db_value: np.ndarray) -> np.ndarray:
    return 10 ** (db_value / 10.0)


# ============================================================
# Instrument helpers
# ============================================================

def find_instrument_by_serial(serial_text: str) -> str:
    rm = pyvisa.ResourceManager()
    resources = rm.list_resources()

    matches = [
        resource for resource in resources
        if resource.startswith("USB") and serial_text in resource
    ]

    if not matches:
        raise RuntimeError(
            f"No USB instrument found containing serial text: {serial_text}\n"
            f"Available VISA resources:\n" + "\n".join(resources)
        )

    if len(matches) > 1:
        raise RuntimeError(
            f"Multiple USB instruments matched serial text: {serial_text}\n"
            f"Matches:\n" + "\n".join(matches)
        )

    return matches[0]


def configure_sweep(inst) -> float:
    inst.write(f":FREQ:STAR {START_HZ}")
    inst.write(f":FREQ:STOP {STOP_HZ}")
    inst.write(f":BAND {RBW_HZ}")
    inst.write(f":BAND:VID {VBW_HZ}")
    inst.write(f":DISP:WIND:TRAC:Y:RLEV {REF_LEVEL_DBM}")

    # Single-sweep mode
    inst.write(":INIT:CONT OFF")

    # ASCII is easiest to debug.
    # For faster acquisition later, this could be changed to REAL,32.
    inst.write(":FORM:TRAC:DATA ASCii")

    if AUTO_SWEEP_TIME:
        inst.write(":SWE:TIME:AUTO ON")
    else:
        inst.write(":SWE:TIME:AUTO OFF")
        inst.write(f":SWE:TIME {MANUAL_SWEEP_TIME_S}")

    time.sleep(SETTLE_TIME_S)

    sweep_time_s = float(inst.query(":SWE:TIME?"))
    return sweep_time_s


def determine_sweep_count(sweep_time_s: float) -> int:
    if AVERAGING_MODE.lower() == "time":
        if sweep_time_s <= 0:
            raise RuntimeError(f"Invalid sweep time reported by analyzer: {sweep_time_s}")

        return max(1, math.ceil(INTEGRATION_TIME_S / sweep_time_s))

    elif AVERAGING_MODE.lower() == "sweeps":
        return max(1, int(NUM_SWEEPS))

    else:
        raise ValueError("AVERAGING_MODE must be either 'time' or 'sweeps'.")


def acquire_one_trace(inst) -> np.ndarray:
    inst.write(":INIT:IMM")
    inst.query("*OPC?")

    raw = inst.query(":TRAC:DATA?")
    y_dbm = np.array([float(v) for v in raw.strip().split(",") if v.strip()])

    if len(y_dbm) == 0:
        raise RuntimeError("Received empty trace data from analyzer.")

    return y_dbm


def get_frequency_axis(inst, point_count: int) -> np.ndarray:
    actual_start_hz = float(inst.query(":FREQ:STAR?"))
    actual_stop_hz = float(inst.query(":FREQ:STOP?"))
    return np.linspace(actual_start_hz, actual_stop_hz, point_count)


# ============================================================
# Integrated sweep logic
# ============================================================

def acquire_integrated_sweep(inst, name: str) -> IntegratedSweep:
    sweep_time_s = configure_sweep(inst)
    sweep_count = determine_sweep_count(sweep_time_s)

    print()
    print(f"Starting {name}")
    print(f"  Frequency range: {START_HZ / 1e9:.6f} to {STOP_HZ / 1e9:.6f} GHz")
    print(f"  RBW:             {RBW_HZ:.3f} Hz")
    print(f"  VBW:             {VBW_HZ:.3f} Hz")
    print(f"  Sweep time:      {sweep_time_s:.6f} s")
    print(f"  Sweep count:     {sweep_count}")

    sum_mw = None
    x_hz = None

    t0 = time.monotonic()

    for i in range(sweep_count):
        y_dbm = acquire_one_trace(inst)
        y_mw = dbm_to_mw(y_dbm)

        if sum_mw is None:
            sum_mw = np.zeros_like(y_mw)
            x_hz = get_frequency_axis(inst, len(y_mw))

        if len(y_mw) != len(sum_mw):
            raise RuntimeError(
                f"Trace length changed during {name}: "
                f"expected {len(sum_mw)}, got {len(y_mw)}"
            )

        sum_mw += y_mw

        print(f"  {name}: sweep {i + 1}/{sweep_count}", end="\r")

    elapsed_time_s = time.monotonic() - t0
    print()
    print(f"Finished {name} in {elapsed_time_s:.2f} s")

    avg_mw = sum_mw / sweep_count
    avg_dbm = mw_to_dbm(avg_mw)

    return IntegratedSweep(
        name=name,
        x_hz=x_hz,
        avg_dbm=avg_dbm,
        avg_mw=avg_mw,
        sweep_count=sweep_count,
        sweep_time_s=sweep_time_s,
        elapsed_time_s=elapsed_time_s,
    )


def compute_difference_and_y_factor(
    sweep1: IntegratedSweep,
    sweep2: IntegratedSweep,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(sweep1.x_hz) != len(sweep2.x_hz):
        raise RuntimeError(
            "Sweep frequency arrays have different lengths. "
            "Make sure both sweeps use the same start, stop, and sweep points."
        )

    if not np.allclose(sweep1.x_hz, sweep2.x_hz):
        raise RuntimeError(
            "Sweep frequency arrays do not match. "
            "Make sure both sweeps use the same start, stop, and sweep points."
        )

    if DIFFERENCE_MODE == "second_minus_first":
        delta_db = sweep2.avg_dbm - sweep1.avg_dbm
    elif DIFFERENCE_MODE == "first_minus_second":
        delta_db = sweep1.avg_dbm - sweep2.avg_dbm
    else:
        raise ValueError(
            "DIFFERENCE_MODE must be 'second_minus_first' or 'first_minus_second'."
        )

    y_factor = db_to_linear_factor(delta_db)

    # This is the requested two-column array:
    # first column = frequency, second column = Y factor
    y_factor_array = np.column_stack((sweep1.x_hz, y_factor))

    return delta_db, y_factor, y_factor_array


# ============================================================
# Saving and plotting
# ============================================================

def save_integrated_sweep_csv(result: IntegratedSweep, filename: str):
    data = np.column_stack((result.x_hz, result.avg_dbm, result.avg_mw))

    np.savetxt(
        filename,
        data,
        delimiter=",",
        header="frequency_hz,average_amplitude_dbm,average_power_mw",
        comments="",
    )

    print(f"Saved: {filename}")


def save_y_factor_csv(y_factor_array: np.ndarray, filename: str):
    np.savetxt(
        filename,
        y_factor_array,
        delimiter=",",
        header="frequency_hz,Y_factor",
        comments="",
    )

    print(f"Saved: {filename}")


def plot_all(
    sweep1: IntegratedSweep,
    sweep2: IntegratedSweep,
    delta_db: np.ndarray,
    y_factor: np.ndarray,
):
    fig, axes = plt.subplots(
        nrows=4,
        ncols=1,
        figsize=(10, 12),
        constrained_layout=True,
    )

    axes[0].plot(sweep1.x_hz / 1e9, sweep1.avg_dbm)
    axes[0].set_title(
        f"{sweep1.name}: averaged spectrum | "
        f"{sweep1.sweep_count} sweeps | "
        f"{sweep1.elapsed_time_s:.1f} s"
    )
    axes[0].set_xlabel("Frequency (GHz)")
    axes[0].set_ylabel("Power (dBm)")
    axes[0].grid(True)

    axes[1].plot(sweep2.x_hz / 1e9, sweep2.avg_dbm)
    axes[1].set_title(
        f"{sweep2.name}: averaged spectrum | "
        f"{sweep2.sweep_count} sweeps | "
        f"{sweep2.elapsed_time_s:.1f} s"
    )
    axes[1].set_xlabel("Frequency (GHz)")
    axes[1].set_ylabel("Power (dBm)")
    axes[1].grid(True)

    axes[2].plot(sweep1.x_hz / 1e9, delta_db)
    axes[2].set_title(f"Difference: {DIFFERENCE_MODE}")
    axes[2].set_xlabel("Frequency (GHz)")
    axes[2].set_ylabel("Difference (dB)")
    axes[2].grid(True)

    axes[3].plot(sweep1.x_hz / 1e9, y_factor)
    axes[3].set_title("Linear Y factor")
    axes[3].set_xlabel("Frequency (GHz)")
    axes[3].set_ylabel("Y = 10^(dB / 10)")
    axes[3].set_ylim(0, 5)
    axes[3].grid(True)

    plt.show()

def save_y_factor_txt(y_factor: np.ndarray, filename: str):
    np.savetxt(
        filename,
        y_factor,
        fmt="%.12e",
        header="Y_factor_linear",
        comments="",
    )

    print(f"Saved: {filename}")


# ============================================================
# Main
# ============================================================

def main():
    rm = pyvisa.ResourceManager()
    resource_name = find_instrument_by_serial(INSTRUMENT_SERIAL)

    print(f"Opening: {resource_name}")

    inst = rm.open_resource(resource_name)
    inst.timeout = 60000
    inst.write_termination = "\n"
    inst.read_termination = "\n"

    try:
        print(inst.query("*IDN?"))

        sweep1 = acquire_integrated_sweep(inst, "Sweep 1")

        print()
        input("Sweep 1 complete. Press Enter when ready to acquire Sweep 2...")

        sweep2 = acquire_integrated_sweep(inst, "Sweep 2")

        delta_db, y_factor, y_factor_array = compute_difference_and_y_factor(
            sweep1,
            sweep2,
        )

        if SAVE_CSV:
            save_integrated_sweep_csv(sweep1, SWEEP1_CSV_FILENAME)
            save_integrated_sweep_csv(sweep2, SWEEP2_CSV_FILENAME)
            save_y_factor_csv(y_factor_array, Y_FACTOR_CSV_FILENAME)

        print()
        print("Y-factor array is available in Python as:")
        print("  y_factor_array")
        print()
        print("Shape:")
        print(f"  {y_factor_array.shape}")
        print()
        print("Columns:")
        print("  y_factor_array[:, 0] = frequency_hz")
        print("  y_factor_array[:, 1] = Y_factor")
        print(y_factor_array)

        plot_all(sweep1, sweep2, delta_db, y_factor)

    finally:
        inst.close()


if __name__ == "__main__":
    main()