# on_off_yfactor_flux_density_adc_reject.py

import math
import time
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

import pyvisa
import numpy as np
import matplotlib.pyplot as plt

# Sky temperature tools
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz
from astropy.time import Time
from astroquery.skyview import SkyView


# ============================================================
# User-configurable spectrum analyzer settings
# ============================================================

SIGLENT_SERIAL = "SSA3PCED7R1040"

# Frequency setup mode:
#   "start_stop"  -> uses START_HZ and STOP_HZ
#   "center_span" -> uses CENTER_HZ and SPAN_HZ
FREQ_MODE = "start_stop"

# Useful for 21 cm line work
CENTER_HZ = 1420.40575177e6
SPAN_HZ = 5.0e6

START_HZ = 1422e6
STOP_HZ = 1423e6

#START_HZ = 1420e6
#STOP_HZ = 1421e6

RBW_HZ = 10e3
VBW_HZ = 1e3
REF_LEVEL_DBM = -50

ATTEN_AUTO = True
ATTEN_DB = 0

# PREAMP_ON:
#   True  -> turn preamp on
#   False -> turn preamp off
#   None  -> leave current setting unchanged
PREAMP_ON: Optional[bool] = True

# Optional trace point setting.
# Set to None to leave instrument default.
SWEEP_POINTS: Optional[int] = None
# Example:
# SWEEP_POINTS = 751

# Analyzer transfer mode.
# ASCII is easiest to debug.
TRACE_FORMAT = "ASCII"
# TRACE_FORMAT = "REAL32"


# ============================================================
# Integration settings
# ============================================================

# Averaging mode:
#   "time"   -> use REQUESTED_INTEGRATION_TIME_S
#   "sweeps" -> use NUM_SWEEPS directly
AVERAGING_MODE = "sweeps"

# Integration time meaning:
#   "per_frequency_bin" -> recommended for radiometry.
#                          Uses dwell time = sweep_time / trace_points.
#
#   "wall_clock"        -> total elapsed acquisition time per ON/OFF scan.
INTEGRATION_TIME_MODE = "per_frequency_bin"

REQUESTED_INTEGRATION_TIME_S = 5.0
NUM_SWEEPS = 300

# Let the analyzer choose sweep time based on RBW/VBW/span
AUTO_SWEEP_TIME = True
MANUAL_SWEEP_TIME_S = 1.0

SETTLE_TIME_S = 0.2
VISA_TIMEOUT_MS = 60000


# ============================================================
# ADC overload rejection
# ============================================================

REJECT_ADC_OVERLOAD_TRACES = True

# If too many bad traces occur, abort rather than looping forever.
MAX_BAD_TRACES_PER_SCAN = 100

# Things to look for in the Siglent error/status queue.
# Siglent manuals refer to ADC overload as ADC_ERROR / code 606.
ADC_OVERLOAD_KEYWORDS = [
    "ADC",
    "OVERLOAD",
    "ADC_ERROR",
    "606",
]


# ============================================================
# Observation / calibration settings
# ============================================================

# Receiver noise temperature in K
T_RECEIVER_K = 80

# Dish / flux-density conversion
DISH_DIAMETER_M = 6.0
ANTENNA_EFFICIENCY = 0.7
BOLTZMANN_J_PER_K = 1.380649e-23

# Single-polarization correction:
# For a single-pol receiver observing an unpolarized source,
# measured flux is half the total Stokes-I flux, so multiply by 2.
POLARIZATION_CORRECTION_FACTOR = 2.0

# Moore Lab / Caltech location
OBSERVING_LOCATION = EarthLocation(
    lat="34d08m23.2s",
    lon="-118d07m28.0s",
    height=263.0 * u.m,
)

# Beam model for sky background averaging
BEAM_WIDTH_DEG = 3.0
SKYVIEW_SURVEY = "1420MHz (Bonn)"
SKYVIEW_PIXELS = 65

# If True, query SkyView for Tsky.
# If False, manually enter Tsky values when prompted.
USE_SKYVIEW_FOR_TSKY = True

# Optional manual override.
# Leave as None to compute or prompt.
MANUAL_T_SKY_ON_K: Optional[float] = None
MANUAL_T_SKY_OFF_K: Optional[float] = None


# ============================================================
# Output settings
# ============================================================

OUTPUT_CSV = "on_off_yfactor_flux_density.csv"
OUTPUT_METADATA_JSON = "on_off_observation_metadata.json"

SHOW_PLOTS = True
SAVE_PLOT = True
OUTPUT_PNG = "on_off_yfactor_flux_density.png"


# ============================================================
# Data containers
# ============================================================

@dataclass
class IntegratedSpectrum:
    label: str
    freq_hz: np.ndarray
    avg_dbm: np.ndarray
    avg_mw: np.ndarray

    # Number of good sweeps used in the average
    sweep_count: int

    # Rejected ADC-overload traces
    bad_sweep_count: int
    attempted_sweep_count: int

    analyzer_sweep_time_s: float
    dwell_time_per_bin_s: float
    elapsed_wall_time_s: float

    utc_start_iso: str
    utc_end_iso: str

    az_deg: float
    el_deg: float
    t_sky_k: float


@dataclass
class SkyTemperatureResult:
    t_sky_k: float
    ra_deg: float
    dec_deg: float
    gal_l_deg: float
    gal_b_deg: float
    utc_iso: str
    az_deg: float
    el_deg: float


# ============================================================
# Basic conversions
# ============================================================

def dbm_to_mw(dbm: np.ndarray) -> np.ndarray:
    return 10.0 ** (dbm / 10.0)


def mw_to_dbm(mw: np.ndarray) -> np.ndarray:
    mw = np.maximum(mw, 1e-30)
    return 10.0 * np.log10(mw)


def db_to_linear(db_value: np.ndarray) -> np.ndarray:
    return 10.0 ** (db_value / 10.0)


def effective_area_m2() -> float:
    return math.pi * (DISH_DIAMETER_M / 2.0) ** 2 * ANTENNA_EFFICIENCY


# ============================================================
# VISA / Siglent helpers
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
    try:
        inst.write(command)
    except Exception as exc:
        print(f"Warning: SCPI command failed: {command}")
        print(f"  {exc}")


def configure_frequency(inst):
    if FREQ_MODE.lower() == "start_stop":
        inst.write(f":FREQ:STAR {START_HZ}")
        inst.write(f":FREQ:STOP {STOP_HZ}")

    elif FREQ_MODE.lower() == "center_span":
        inst.write(f":FREQ:CENT {CENTER_HZ}")
        inst.write(f":FREQ:SPAN {SPAN_HZ}")

    else:
        raise ValueError("FREQ_MODE must be 'start_stop' or 'center_span'.")


def configure_analyzer(inst):
    configure_frequency(inst)

    inst.write(f":BAND {RBW_HZ}")
    inst.write(f":BAND:VID {VBW_HZ}")
    inst.write(f":DISP:WIND:TRAC:Y:RLEV {REF_LEVEL_DBM}")

    if SWEEP_POINTS is not None:
        scpi_write_ignore_error(inst, f":SWE:POIN {SWEEP_POINTS}")

    if AUTO_SWEEP_TIME:
        inst.write(":SWE:TIME:AUTO ON")
    else:
        inst.write(":SWE:TIME:AUTO OFF")
        inst.write(f":SWE:TIME {MANUAL_SWEEP_TIME_S}")

    # Single-sweep mode; script triggers each sweep.
    inst.write(":INIT:CONT OFF")

    if TRACE_FORMAT.upper() == "ASCII":
        inst.write(":FORM:TRAC:DATA ASCii")
    elif TRACE_FORMAT.upper() == "REAL32":
        inst.write(":FORM:TRAC:DATA REAL,32")
    else:
        raise ValueError("TRACE_FORMAT must be 'ASCII' or 'REAL32'.")

    if ATTEN_AUTO:
        scpi_write_ignore_error(inst, ":POW:ATT:AUTO ON")
    else:
        scpi_write_ignore_error(inst, ":POW:ATT:AUTO OFF")
        scpi_write_ignore_error(inst, f":POW:ATT {ATTEN_DB}")

    if PREAMP_ON is True:
        scpi_write_ignore_error(inst, ":POW:GAIN ON")
    elif PREAMP_ON is False:
        scpi_write_ignore_error(inst, ":POW:GAIN OFF")

    # Clear/write trace mode on analyzer; software averaging is done here.
    scpi_write_ignore_error(inst, ":TRAC1:MODE WRIT")

    time.sleep(SETTLE_TIME_S)


def get_analyzer_sweep_time_s(inst) -> float:
    return float(inst.query(":SWE:TIME?"))


# ============================================================
# Siglent error / ADC-overload helpers
# ============================================================

def drain_siglent_error_queue(inst, max_reads: int = 20) -> list[str]:
    """
    Read and clear the Siglent SCPI error/status queue.

    Returns a list of all non-zero / non-empty error messages found.

    Typical no-error responses may look like:
        0,"No error"
        0,No error
        +0,"No error"
    """

    messages = []

    for _ in range(max_reads):
        try:
            response = inst.query(":SYST:ERR?").strip()
        except Exception as exc:
            messages.append(f"ERROR_QUEUE_QUERY_FAILED: {exc}")
            break

        normalized = response.upper().replace(" ", "")

        no_error = (
            normalized.startswith("0,")
            or normalized.startswith("+0,")
            or "NOERROR" in normalized
        )

        if no_error:
            break

        messages.append(response)

    return messages


def clear_siglent_status(inst):
    """
    Clear stale event/error information before judging a new sweep.
    """

    try:
        inst.write("*CLS")
    except Exception:
        pass

    try:
        drain_siglent_error_queue(inst)
    except Exception:
        pass


def error_messages_indicate_adc_overload(messages: list[str]) -> bool:
    combined = " ".join(messages).upper()
    return any(keyword.upper() in combined for keyword in ADC_OVERLOAD_KEYWORDS)


def check_for_adc_overload(inst) -> tuple[bool, list[str]]:
    messages = drain_siglent_error_queue(inst)
    overloaded = error_messages_indicate_adc_overload(messages)
    return overloaded, messages


# ============================================================
# Trace acquisition
# ============================================================

def acquire_one_trace(inst) -> tuple[np.ndarray, np.ndarray, bool, list[str]]:
    """
    Trigger one sweep and return:

        freq_hz
        power_dbm
        reject_trace
        status_messages

    If the Siglent reports ADC overload, reject_trace=True.
    """

    if REJECT_ADC_OVERLOAD_TRACES:
        clear_siglent_status(inst)

    inst.write(":INIT:IMM")
    inst.query("*OPC?")

    if TRACE_FORMAT.upper() == "ASCII":
        raw = inst.query(":TRAC:DATA?")
        power_dbm = np.array(
            [float(v) for v in raw.strip().split(",") if v.strip()],
            dtype=float,
        )

    elif TRACE_FORMAT.upper() == "REAL32":
        power_dbm = inst.query_binary_values(
            ":TRAC:DATA?",
            datatype="f",
            is_big_endian=False,
            container=np.array,
        )

    else:
        raise ValueError("TRACE_FORMAT must be 'ASCII' or 'REAL32'.")

    if len(power_dbm) == 0:
        raise RuntimeError("Received empty trace from analyzer.")

    actual_start_hz = float(inst.query(":FREQ:STAR?"))
    actual_stop_hz = float(inst.query(":FREQ:STOP?"))
    freq_hz = np.linspace(actual_start_hz, actual_stop_hz, len(power_dbm))

    reject_trace = False
    status_messages = []

    if REJECT_ADC_OVERLOAD_TRACES:
        overloaded, status_messages = check_for_adc_overload(inst)
        if overloaded:
            reject_trace = True

    return freq_hz, power_dbm, reject_trace, status_messages


# ============================================================
# Sky temperature calculation
# ============================================================

def compute_sky_temperature_k(
    az_deg: float,
    el_deg: float,
    utc_iso: str,
) -> SkyTemperatureResult:
    """
    Compute average 1420 MHz sky temperature in the beam.

    Alt/Az at Moore Lab -> ICRS -> SkyView 1420 MHz Bonn map ->
    average over a 3-degree beam cutout.
    """

    observing_time = Time(utc_iso)

    altaz_coord = SkyCoord(
        alt=el_deg * u.deg,
        az=az_deg * u.deg,
        frame=AltAz(obstime=observing_time, location=OBSERVING_LOCATION),
    )

    icrs_coord = altaz_coord.transform_to("icrs")
    gal_coord = altaz_coord.transform_to("galactic")

    hdulists = SkyView.get_images(
        position=icrs_coord,
        survey=[SKYVIEW_SURVEY],
        pixels=(SKYVIEW_PIXELS, SKYVIEW_PIXELS),
    )

    if not hdulists:
        raise RuntimeError("SkyView returned no images. Check survey coverage/position.")

    hdu = hdulists[0][0]
    header = hdu.header
    deg_per_pixel = abs(header["CDELT1"])

    beam_width_pixels = BEAM_WIDTH_DEG / deg_per_pixel
    half_beam_width_pixels = beam_width_pixels / 2.0

    # Your uploaded script divides by 1000 to convert the Bonn map values to K.
    cutout_k = np.asarray(hdu.data, dtype=float) / 1000.0

    ny, nx = cutout_k.shape
    cy, cx = ny // 2, nx // 2

    x0 = max(0, cx - int(half_beam_width_pixels))
    x1 = min(nx, cx + int(half_beam_width_pixels))
    y0 = max(0, cy - int(half_beam_width_pixels))
    y1 = min(ny, cy + int(half_beam_width_pixels))

    t_sky_k = float(np.nanmean(cutout_k[y0:y1, x0:x1]))

    return SkyTemperatureResult(
        t_sky_k=t_sky_k,
        ra_deg=float(icrs_coord.ra.deg),
        dec_deg=float(icrs_coord.dec.deg),
        gal_l_deg=float(gal_coord.l.deg),
        gal_b_deg=float(gal_coord.b.deg),
        utc_iso=utc_iso,
        az_deg=az_deg,
        el_deg=el_deg,
    )


# ============================================================
# Acquisition and averaging
# ============================================================

def determine_sweep_count(
    analyzer_sweep_time_s: float,
    trace_points: int,
) -> tuple[int, float]:
    """
    Determine number of sweeps.

    For radiometry, the important value is usually integration time per
    frequency bin, not total sweep wall time.

    dwell_time_per_bin ~= analyzer_sweep_time / number_of_trace_points
    """

    dwell_time_per_bin_s = analyzer_sweep_time_s / trace_points

    if AVERAGING_MODE.lower() == "sweeps":
        return max(1, int(NUM_SWEEPS)), dwell_time_per_bin_s

    if AVERAGING_MODE.lower() != "time":
        raise ValueError("AVERAGING_MODE must be 'time' or 'sweeps'.")

    if INTEGRATION_TIME_MODE.lower() == "per_frequency_bin":
        if dwell_time_per_bin_s <= 0:
            raise RuntimeError("Invalid dwell time per frequency bin.")

        sweep_count = math.ceil(REQUESTED_INTEGRATION_TIME_S / dwell_time_per_bin_s)
        return max(1, sweep_count), dwell_time_per_bin_s

    if INTEGRATION_TIME_MODE.lower() == "wall_clock":
        if analyzer_sweep_time_s <= 0:
            raise RuntimeError("Invalid analyzer sweep time.")

        sweep_count = math.ceil(REQUESTED_INTEGRATION_TIME_S / analyzer_sweep_time_s)
        return max(1, sweep_count), dwell_time_per_bin_s

    raise ValueError("INTEGRATION_TIME_MODE must be 'per_frequency_bin' or 'wall_clock'.")


def prompt_float(prompt: str) -> float:
    return float(input(prompt).strip())


def acquire_integrated_spectrum(
    inst,
    label: str,
    manual_tsky_k: Optional[float],
) -> IntegratedSpectrum:
    print()
    print("=" * 72)
    print(f"{label.upper()} SCAN")
    print("=" * 72)
    print("Point the dish manually, then enter the pointing information.")
    print()

    az_deg = prompt_float(f"{label} azimuth deg: ")
    el_deg = prompt_float(f"{label} elevation deg: ")

    input(f"Press Enter to start the {label} integration...")

    utc_start = datetime.now(timezone.utc)
    utc_start_iso = utc_start.replace(microsecond=0).isoformat().replace("+00:00", "Z")

    configure_analyzer(inst)

    print("Taking setup sweep to determine trace length and dwell time...")
    setup_freq_hz, _setup_dbm, setup_rejected, setup_messages = acquire_one_trace(inst)

    if setup_rejected:
        print("Warning: setup sweep had ADC overload.")
        print("Messages:", setup_messages)
        print("Continuing, but increase attenuation / disable preamp / raise ref level if this persists.")

    trace_points = len(setup_freq_hz)

    analyzer_sweep_time_s = get_analyzer_sweep_time_s(inst)
    sweep_count, dwell_time_per_bin_s = determine_sweep_count(
        analyzer_sweep_time_s=analyzer_sweep_time_s,
        trace_points=trace_points,
    )

    print()
    print(f"{label} integration settings:")
    print(f"  Analyzer sweep time:       {analyzer_sweep_time_s:.6g} s")
    print(f"  Trace points:              {trace_points}")
    print(f"  Dwell time per bin:        {dwell_time_per_bin_s:.6g} s")
    print(f"  Good sweeps to average:    {sweep_count}")
    print(f"  Estimated wall time:       {sweep_count * analyzer_sweep_time_s:.2f} s")
    print(f"  Estimated bin integration: {sweep_count * dwell_time_per_bin_s:.2f} s")
    print()

    sum_mw = None
    freq_hz = None

    good_sweeps = 0
    bad_sweeps = 0
    attempted_sweeps = 0

    t0 = time.monotonic()

    while good_sweeps < sweep_count:
        attempted_sweeps += 1

        this_freq_hz, power_dbm, reject_trace, status_messages = acquire_one_trace(inst)

        if reject_trace:
            bad_sweeps += 1

            print()
            print(
                f"{label}: rejected ADC-overload trace "
                f"{bad_sweeps}/{MAX_BAD_TRACES_PER_SCAN} "
                f"(good {good_sweeps}/{sweep_count}, attempted {attempted_sweeps})"
            )

            if status_messages:
                print("  Siglent status/error messages:")
                for msg in status_messages:
                    print(f"    {msg}")

            if bad_sweeps >= MAX_BAD_TRACES_PER_SCAN:
                raise RuntimeError(
                    f"Too many ADC-overload traces during {label} scan. "
                    f"Rejected {bad_sweeps} traces. "
                    f"Try increasing attenuation, disabling preamp, or raising ref level."
                )

            continue

        power_mw = dbm_to_mw(power_dbm)

        if sum_mw is None:
            sum_mw = np.zeros_like(power_mw)
            freq_hz = this_freq_hz

        if len(power_mw) != len(sum_mw):
            raise RuntimeError(
                f"Trace length changed during {label}: "
                f"expected {len(sum_mw)}, got {len(power_mw)}"
            )

        if not np.allclose(this_freq_hz, freq_hz):
            raise RuntimeError(f"Frequency axis changed during {label} scan.")

        sum_mw += power_mw
        good_sweeps += 1

        print(
            f"{label}: good sweep {good_sweeps}/{sweep_count} "
            f"| rejected {bad_sweeps} "
            f"| attempted {attempted_sweeps}",
            end="\r",
        )

    elapsed_wall_time_s = time.monotonic() - t0
    utc_end = datetime.now(timezone.utc)
    utc_end_iso = utc_end.replace(microsecond=0).isoformat().replace("+00:00", "Z")

    print()
    print(
        f"{label} complete: "
        f"{good_sweeps} good sweeps, "
        f"{bad_sweeps} rejected ADC-overload sweeps, "
        f"{attempted_sweeps} total attempted sweeps."
    )
    print(f"{label} elapsed wall time: {elapsed_wall_time_s:.2f} s")

    avg_mw = sum_mw / good_sweeps
    avg_dbm = mw_to_dbm(avg_mw)

    if manual_tsky_k is not None:
        t_sky_k = manual_tsky_k
        print(f"Using manual {label} T_sky = {t_sky_k:.3f} K")

    elif USE_SKYVIEW_FOR_TSKY:
        print(f"Querying SkyView for {label} T_sky at UTC {utc_start_iso}...")
        sky = compute_sky_temperature_k(
            az_deg=az_deg,
            el_deg=el_deg,
            utc_iso=utc_start_iso,
        )
        t_sky_k = sky.t_sky_k
        print(f"{label} T_sky = {t_sky_k:.3f} K")
        print(f"  RA/Dec:       {sky.ra_deg:.6f}, {sky.dec_deg:.6f} deg")
        print(f"  Galactic l/b: {sky.gal_l_deg:.6f}, {sky.gal_b_deg:.6f} deg")

    else:
        t_sky_k = prompt_float(f"Enter {label} T_sky in K: ")

    return IntegratedSpectrum(
        label=label,
        freq_hz=freq_hz,
        avg_dbm=avg_dbm,
        avg_mw=avg_mw,
        sweep_count=good_sweeps,
        bad_sweep_count=bad_sweeps,
        attempted_sweep_count=attempted_sweeps,
        analyzer_sweep_time_s=analyzer_sweep_time_s,
        dwell_time_per_bin_s=dwell_time_per_bin_s,
        elapsed_wall_time_s=elapsed_wall_time_s,
        utc_start_iso=utc_start_iso,
        utc_end_iso=utc_end_iso,
        az_deg=az_deg,
        el_deg=el_deg,
        t_sky_k=t_sky_k,
    )


# ============================================================
# Science calculations
# ============================================================

def compute_yfactor_temperature_flux(
    on: IntegratedSpectrum,
    off: IntegratedSpectrum,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(on.freq_hz) != len(off.freq_hz):
        raise RuntimeError("ON and OFF spectra have different lengths.")

    if not np.allclose(on.freq_hz, off.freq_hz):
        raise RuntimeError("ON and OFF frequency axes do not match.")

    # Y = P_on / P_off
    delta_db = on.avg_dbm - off.avg_dbm
    y_factor = db_to_linear(delta_db)

    # User-provided equation:
    #
    # T_source = Y*T_sky2 + (Y-1)*T_receiver - T_sky1
    #
    # Here:
    #   T_sky1 = ON/source-pointing sky background, source not included
    #   T_sky2 = OFF-pointing sky background
    t_source_k = (
        y_factor * off.t_sky_k
        + (y_factor - 1.0) * T_RECEIVER_K
        - off.t_sky_k
    )

    # Flux density:
    #
    # PSD = k_B * T_source
    # S = PSD / A_e
    # Jy = S * 1e26
    #
    # Single-pol correction:
    # multiply by 2 for total flux density of an unpolarized source.
    ae_m2 = effective_area_m2()

    flux_density_jy = (
        POLARIZATION_CORRECTION_FACTOR
        * BOLTZMANN_J_PER_K
        * t_source_k
        / ae_m2
        * 1.0e26
    )

    return y_factor, t_source_k, flux_density_jy


# ============================================================
# Output
# ============================================================

def save_csv(
    freq_hz: np.ndarray,
    on_dbm: np.ndarray,
    off_dbm: np.ndarray,
    y_factor: np.ndarray,
    flux_density_jy: np.ndarray,
    t_source_k: np.ndarray,
):
    data = np.column_stack(
        (
            freq_hz,
            on_dbm,
            off_dbm,
            y_factor,
            flux_density_jy,
            t_source_k,
        )
    )

    np.savetxt(
        OUTPUT_CSV,
        data,
        delimiter=",",
        header="frequency_hz,on_spectrum_dbm,off_spectrum_dbm,Y_factor,flux_density_jy,source_temperature_k",
        comments="",
    )

    print(f"Saved CSV: {OUTPUT_CSV}")


def spectrum_metadata_dict(s: IntegratedSpectrum) -> dict:
    return {
        "label": s.label,
        "sweep_count_good": s.sweep_count,
        "bad_sweep_count_adc_overload": s.bad_sweep_count,
        "attempted_sweep_count": s.attempted_sweep_count,
        "analyzer_sweep_time_s": s.analyzer_sweep_time_s,
        "dwell_time_per_bin_s": s.dwell_time_per_bin_s,
        "elapsed_wall_time_s": s.elapsed_wall_time_s,
        "utc_start_iso": s.utc_start_iso,
        "utc_end_iso": s.utc_end_iso,
        "az_deg": s.az_deg,
        "el_deg": s.el_deg,
        "t_sky_k": s.t_sky_k,
        "freq_hz": "stored in CSV",
        "avg_dbm": "stored in CSV",
        "avg_mw": "not stored in metadata",
    }


def save_metadata(
    on: IntegratedSpectrum,
    off: IntegratedSpectrum,
):
    metadata = {
        "spectrum_analyzer": {
            "freq_mode": FREQ_MODE,
            "center_hz": CENTER_HZ,
            "span_hz": SPAN_HZ,
            "start_hz": START_HZ,
            "stop_hz": STOP_HZ,
            "rbw_hz": RBW_HZ,
            "vbw_hz": VBW_HZ,
            "ref_level_dbm": REF_LEVEL_DBM,
            "atten_auto": ATTEN_AUTO,
            "atten_db": ATTEN_DB,
            "preamp_on": PREAMP_ON,
            "sweep_points_requested": SWEEP_POINTS,
            "trace_format": TRACE_FORMAT,
        },
        "integration": {
            "averaging_mode": AVERAGING_MODE,
            "integration_time_mode": INTEGRATION_TIME_MODE,
            "requested_integration_time_s": REQUESTED_INTEGRATION_TIME_S,
            "num_sweeps_setting": NUM_SWEEPS,
        },
        "adc_overload_rejection": {
            "enabled": REJECT_ADC_OVERLOAD_TRACES,
            "max_bad_traces_per_scan": MAX_BAD_TRACES_PER_SCAN,
            "keywords": ADC_OVERLOAD_KEYWORDS,
        },
        "receiver": {
            "t_receiver_k": T_RECEIVER_K,
        },
        "dish": {
            "diameter_m": DISH_DIAMETER_M,
            "antenna_efficiency": ANTENNA_EFFICIENCY,
            "effective_area_m2": effective_area_m2(),
        },
        "polarization": {
            "single_polarization_receiver": True,
            "source_assumed_unpolarized": True,
            "flux_density_correction_factor": POLARIZATION_CORRECTION_FACTOR,
        },
        "sky_temperature": {
            "use_skyview": USE_SKYVIEW_FOR_TSKY,
            "survey": SKYVIEW_SURVEY,
            "beam_width_deg": BEAM_WIDTH_DEG,
            "location": "Moore Laboratory, Caltech",
        },
        "on_scan": spectrum_metadata_dict(on),
        "off_scan": spectrum_metadata_dict(off),
    }

    with open(OUTPUT_METADATA_JSON, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    print(f"Saved metadata: {OUTPUT_METADATA_JSON}")


def plot_results(
    freq_hz: np.ndarray,
    on_dbm: np.ndarray,
    off_dbm: np.ndarray,
    y_factor: np.ndarray,
    t_source_k: np.ndarray,
    flux_density_jy: np.ndarray,
):
    freq_mhz = freq_hz / 1e6

    fig, axes = plt.subplots(
        nrows=4,
        ncols=1,
        figsize=(11, 13),
        constrained_layout=True,
    )

    axes[0].plot(freq_mhz, on_dbm, label="ON")
    axes[0].plot(freq_mhz, off_dbm, label="OFF")
    axes[0].set_xlabel("Frequency (MHz)")
    axes[0].set_ylabel("Power (dBm)")
    axes[0].set_title("Integrated ON/OFF spectra")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(freq_mhz, y_factor)
    axes[1].set_xlabel("Frequency (MHz)")
    axes[1].set_ylabel("Y = P_on / P_off")
    axes[1].set_title("Y factor")
    axes[1].grid(True)

    axes[2].plot(freq_mhz, t_source_k)
    axes[2].set_xlabel("Frequency (MHz)")
    axes[2].set_ylabel("Brightness temperature (K)")
    axes[2].set_title("Source brightness temperature")
    axes[2].grid(True)

    axes[3].plot(freq_mhz, flux_density_jy)
    axes[3].set_xlabel("Frequency (MHz)")
    axes[3].set_ylabel("Flux density (Jy)")
    axes[3].set_title("Flux density with single-pol x2 correction")
    axes[3].grid(True)

    if SAVE_PLOT:
        fig.savefig(OUTPUT_PNG, dpi=200)
        print(f"Saved plot: {OUTPUT_PNG}")

    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close(fig)


# ============================================================
# Main sequence
# ============================================================

def main():
    rm = pyvisa.ResourceManager()
    resource_name = find_instrument_by_serial(SIGLENT_SERIAL)

    print(f"Opening spectrum analyzer: {resource_name}")

    inst = rm.open_resource(resource_name)
    inst.timeout = VISA_TIMEOUT_MS
    inst.write_termination = "\n"
    inst.read_termination = "\n"

    try:
        try:
            inst.clear()
        except Exception:
            pass

        print("Connected to:", inst.query("*IDN?").strip())

        print()
        print("This script will run:")
        print("  1. ON/source scan")
        print("  2. OFF/background scan")
        print("  3. ADC-overload rejection during averaging")
        print("  4. Y-factor, brightness temperature, and flux density calculation")
        print()

        on = acquire_integrated_spectrum(
            inst=inst,
            label="ON",
            manual_tsky_k=MANUAL_T_SKY_ON_K,
        )

        print()
        input("Move dish to OFF position. Press Enter when ready to continue...")

        off = acquire_integrated_spectrum(
            inst=inst,
            label="OFF",
            manual_tsky_k=MANUAL_T_SKY_OFF_K,
        )

        y_factor, t_source_k, flux_density_jy = compute_yfactor_temperature_flux(
            on=on,
            off=off,
        )

        save_csv(
            freq_hz=on.freq_hz,
            on_dbm=on.avg_dbm,
            off_dbm=off.avg_dbm,
            y_factor=y_factor,
            flux_density_jy=flux_density_jy,
            t_source_k=t_source_k,
        )

        save_metadata(on=on, off=off)

        print()
        print("Summary:")
        print(f"  ON  T_sky: {on.t_sky_k:.3f} K")
        print(f"  OFF T_sky: {off.t_sky_k:.3f} K")
        print(f"  T_receiver: {T_RECEIVER_K:.3f} K")
        print(f"  Effective area: {effective_area_m2():.3f} m^2")
        print(f"  Polarization correction factor: {POLARIZATION_CORRECTION_FACTOR:.3f}")
        print(f"  ON good/rejected/attempted sweeps:  {on.sweep_count}/{on.bad_sweep_count}/{on.attempted_sweep_count}")
        print(f"  OFF good/rejected/attempted sweeps: {off.sweep_count}/{off.bad_sweep_count}/{off.attempted_sweep_count}")
        print(f"  Median Y factor: {np.nanmedian(y_factor):.6g}")
        print(f"  Median T_source: {np.nanmedian(t_source_k):.6g} K")
        print(f"  Median flux density: {np.nanmedian(flux_density_jy):.6g} Jy")

        plot_results(
            freq_hz=on.freq_hz,
            on_dbm=on.avg_dbm,
            off_dbm=off.avg_dbm,
            y_factor=y_factor,
            t_source_k=t_source_k,
            flux_density_jy=flux_density_jy,
        )

    finally:
        inst.close()
        print("Spectrum analyzer connection closed.")


if __name__ == "__main__":
    main()