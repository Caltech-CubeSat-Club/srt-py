import time
from dataclasses import dataclass
from typing import Optional

import pyvisa
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# User-configurable settings
# ============================================================

INSTRUMENT_SERIAL = "SSA3PCED7R1040"

# Frequency setup mode:
#   "start_stop"  -> uses START_HZ and STOP_HZ
#   "center_span" -> uses CENTER_HZ and SPAN_HZ
FREQ_MODE = "start_stop"

START_HZ = 1e9
STOP_HZ = 1.9e9

CENTER_HZ = 1.4205e9
SPAN_HZ = 200.0e6
                                                                                                                                                                                                                                                                                                                                                                                                        
RBW_HZ = 1e6
VBW_HZ = 100e3

REF_LEVEL_DBM = -30

# Y-axis display control:
#
# Y_AXIS_MODE:
#   "auto"         -> auto-scale every update
#   "fixed_limits" -> use Y_LIM_DBM directly
#   "db_per_div"   -> use REF_LEVEL_DBM as top of screen and Y_DB_PER_DIV scale
#
# Example:
#   REF_LEVEL_DBM = -20
#   Y_DB_PER_DIV = 10
#   Y_NUM_DIVS = 10
#
# gives a plot range from -120 dBm to -20 dBm.
Y_AXIS_MODE = "db_per_div"
                                                                                                                                                                                                                                            
Y_DB_PER_DIV = 10
Y_NUM_DIVS = 10

# Used only when Y_AXIS_MODE = "fixed_limits"
Y_LIM_DBM = (-84, -85)

# Attenuation:
#   ATTEN_AUTO = True lets analyzer choose attenuation
#   ATTEN_AUTO = False uses ATTEN_DB
ATTEN_AUTO = False
ATTEN_DB = 0

# Preamp:
#   PREAMP_ON = True
#   PREAMP_ON = False
#   PREAMP_ON = None leaves current instrument setting unchanged
PREAMP_ON: Optional[bool] = True

# Trace mode:
#   "clear_write" -> live trace directly from analyzer
#   "average"     -> sliding software average in this script
TRACE_TYPE = "clear_write"

# Used only if TRACE_TYPE = "average".
# The displayed trace is the sliding average of the most recent NUM_AVERAGES sweeps.
NUM_AVERAGES = 300

# Display update timing
PLOT_UPDATE_INTERVAL_S = 0.05
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
# Optional x-axis units:
#   "Hz", "kHz", "MHz", "GHz"
X_UNITS = "MHz"

# Save the most recent displayed trace when you close the plot?
SAVE_LAST_TRACE_CSV = True
LAST_TRACE_FILENAME = "last_live_trace.csv"


# ============================================================
# Data helpers
# ============================================================
                                                                                                                       
@dataclass
class TraceData:
    freq_hz: np.ndarray
    power_dbm: np.ndarray


def dbm_to_mw(dbm: np.ndarray) -> np.ndarray:
    return 10 ** (dbm / 10.0)


def mw_to_dbm(mw: np.ndarray) -> np.ndarray:
    mw = np.maximum(mw, 1e-30)
    return 10.0 * np.log10(mw)


def x_scale_factor_and_label():
    units = X_UNITS.lower()

    if units == "hz":
        return 1.0, "Frequency (Hz)"
    if units == "khz":
        return 1e3, "Frequency (kHz)"
    if units == "mhz":
        return 1e6, "Frequency (MHz)"
    if units == "ghz":
        return 1e9, "Frequency (GHz)"

    raise ValueError("X_UNITS must be one of: Hz, kHz, MHz, GHz")


def apply_y_axis_scaling(ax, y_dbm: np.ndarray):
    mode = Y_AXIS_MODE.lower()

    if mode == "auto":
        ymin = np.nanmin(y_dbm)
        ymax = np.nanmax(y_dbm)
        margin = max(1.0, 0.05 * abs(ymax - ymin))
        ax.set_ylim(ymin - margin, ymax + margin)

    elif mode == "fixed_limits":
        ax.set_ylim(*Y_LIM_DBM)

    elif mode == "db_per_div":
        y_top = REF_LEVEL_DBM
        y_bottom = REF_LEVEL_DBM - Y_DB_PER_DIV * Y_NUM_DIVS

        ax.set_ylim(y_bottom, y_top)

        ticks = np.arange(y_bottom, y_top + Y_DB_PER_DIV, Y_DB_PER_DIV)
        ax.set_yticks(ticks)

    else:
        raise ValueError(
            "Y_AXIS_MODE must be 'auto', 'fixed_limits', or 'db_per_div'."
        )


# ============================================================
# VISA / instrument helpers
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


def scpi_write_ignore_error(inst, command: str):
    """
    Some SCPI commands vary slightly by firmware/model/options.
    This helper attempts a command and warns rather than crashing.
    """
    try:
        inst.write(command)
    except Exception as exc:
        print(f"Warning: command failed: {command}")
        print(f"  {exc}")


def configure_frequency(inst):
    if FREQ_MODE.lower() == "start_stop":
        inst.write(f":FREQ:STAR {START_HZ}")
        inst.write(f":FREQ:STOP {STOP_HZ}")

    elif FREQ_MODE.lower() == "center_span":
        inst.write(f":FREQ:CENT {CENTER_HZ}")
        inst.write(f":FREQ:SPAN {SPAN_HZ}")

    else:
        raise ValueError("FREQ_MODE must be either 'start_stop' or 'center_span'.")


def configure_instrument(inst):
    """
    Configure core analyzer settings.
    """

    configure_frequency(inst)

    inst.write(f":BAND {RBW_HZ}")
    inst.write(f":BAND:VID {VBW_HZ}")
    inst.write(f":DISP:WIND:TRAC:Y:RLEV {REF_LEVEL_DBM}")

    # Let analyzer choose sweep time based on span/RBW/VBW.
    inst.write(":SWE:TIME:AUTO ON")

    # Single-sweep mode. The script triggers each update.
    inst.write(":INIT:CONT OFF")

    # ASCII is easiest to debug.
    # Later, REAL,32 can be used for faster transfers.
    inst.write(":FORM:TRAC:DATA ASCii")

    # Attenuator.
    # These command forms are common on Siglent SSA units, but may vary by firmware.
    if ATTEN_AUTO:
        scpi_write_ignore_error(inst, ":POW:ATT:AUTO ON")
    else:
        scpi_write_ignore_error(inst, ":POW:ATT:AUTO OFF")
        scpi_write_ignore_error(inst, f":POW:ATT {ATTEN_DB}")

    # Preamp.
    # Leave unchanged if PREAMP_ON is None.
    if PREAMP_ON is True:
        scpi_write_ignore_error(inst, ":POW:GAIN ON")
    elif PREAMP_ON is False:
        scpi_write_ignore_error(inst, ":POW:GAIN OFF")

    # Analyzer trace behavior.
    # We still do software averaging below if TRACE_TYPE == "average".
    if TRACE_TYPE.lower() == "clear_write":
        scpi_write_ignore_error(inst, ":TRAC1:MODE WRIT")
    elif TRACE_TYPE.lower() == "average":
        scpi_write_ignore_error(inst, ":TRAC1:MODE WRIT")
    else:
        raise ValueError("TRACE_TYPE must be either 'clear_write' or 'average'.")

    time.sleep(0.2)

    try:
        sweep_time_s = float(inst.query(":SWE:TIME?"))
        print(f"Analyzer sweep time: {sweep_time_s:.6f} s")
    except Exception:
        print("Warning: could not query sweep time.")


def acquire_one_trace(inst) -> TraceData:
    """
    Trigger one sweep and read trace data.
    """

    inst.write(":INIT:IMM")
    inst.query("*OPC?")

    raw = inst.query(":TRAC:DATA?")
    power_dbm = np.array([float(v) for v in raw.strip().split(",") if v.strip()])

    if len(power_dbm) == 0:
        raise RuntimeError("Received empty trace from analyzer.")

    actual_start_hz = float(inst.query(":FREQ:STAR?"))
    actual_stop_hz = float(inst.query(":FREQ:STOP?"))

    freq_hz = np.linspace(actual_start_hz, actual_stop_hz, len(power_dbm))

    return TraceData(freq_hz=freq_hz, power_dbm=power_dbm)


# ============================================================
# Live plotting
# ============================================================

def live_spectrum_window(inst):
    """
    Opens a live-updating Matplotlib spectrum window.
    Close the window to stop acquisition.
    """

    scale, xlabel = x_scale_factor_and_label()

    plt.ion()

    fig, ax = plt.subplots(figsize=(11, 6))
    line, = ax.plot([], [])

    ax.set_xlabel(xlabel)
    ax.set_ylabel("Power (dBm)")
    ax.set_title("Live Siglent Spectrum")
    ax.grid(True)

    average_buffer_mw = []

    last_trace = None
    sweep_index = 0

    print()
    print("Live spectrum running.")
    print("Close the plot window to stop.")
    print()

    while plt.fignum_exists(fig.number):
        trace = acquire_one_trace(inst)
        sweep_index += 1

        if TRACE_TYPE.lower() == "clear_write":
            display_dbm = trace.power_dbm
            avg_status_text = "clear & write"

        elif TRACE_TYPE.lower() == "average":
            current_mw = dbm_to_mw(trace.power_dbm)

            # Sliding average:
            # Keep only the most recent NUM_AVERAGES traces.
            average_buffer_mw.append(current_mw)

            if len(average_buffer_mw) > NUM_AVERAGES:
                average_buffer_mw.pop(0)

            # Guard against trace length changes.
            lengths = [len(arr) for arr in average_buffer_mw]
            if len(set(lengths)) != 1:
                average_buffer_mw = [current_mw]

            avg_mw = np.mean(np.stack(average_buffer_mw, axis=0), axis=0)
            display_dbm = mw_to_dbm(avg_mw)

            avg_status_text = f"sliding average {len(average_buffer_mw)}/{NUM_AVERAGES}"

        else:
            raise ValueError("TRACE_TYPE must be either 'clear_write' or 'average'.")

        last_trace = TraceData(freq_hz=trace.freq_hz, power_dbm=display_dbm)

        x = trace.freq_hz / scale
        y = display_dbm

        line.set_data(x, y)

        ax.set_xlim(x[0], x[-1])
        apply_y_axis_scaling(ax, y)

        ax.set_title(
            f"Live Siglent Spectrum | {avg_status_text} | sweep {sweep_index}"
        )

        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(PLOT_UPDATE_INTERVAL_S)

    plt.ioff()

    if SAVE_LAST_TRACE_CSV and last_trace is not None:
        save_trace_csv(last_trace, LAST_TRACE_FILENAME)


def save_trace_csv(trace: TraceData, filename: str):
    data = np.column_stack((trace.freq_hz, trace.power_dbm))

    np.savetxt(
        filename,
        data,
        delimiter=",",
        header="frequency_hz,power_dbm",
        comments="",
    )

    print(f"Saved last trace to: {filename}")


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

        configure_instrument(inst)
        live_spectrum_window(inst)

    finally:
        inst.close()
        print("Instrument connection closed.")


if __name__ == "__main__":
    main()