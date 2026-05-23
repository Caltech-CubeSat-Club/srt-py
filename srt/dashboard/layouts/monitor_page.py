"""monitor_page.py

Function for Generating Monitor Page and Creating Callback

"""

import dash
import importlib

try:
    from dash import dcc
except Exception:
    dcc = importlib.import_module("dash_core_components")

import dash_bootstrap_components as dbc

try:
    from dash import html
except Exception:
    html = importlib.import_module("dash_html_components")

from dash.exceptions import PreventUpdate

from dash.dependencies import Input, Output, State

from pathlib import Path
from time import time
import base64
import csv
import io
import numpy as np

from .navbar import generate_navbar
from .graphs import (
    generate_az_el_graph,
    generate_az_time_graph,
    generate_el_time_graph,
    generate_pointing_error_graph,
    generate_power_history_graph,
    generate_spectrum_graph,
    generate_zoom_graph,
)

from astropy.table import Table
from srt import config_loader


root_folder = Path(__file__).parent.parent.parent.parent


def run_srt_daemon(configuration_dir, configuration_dict):
    """Module-level multiprocessing target for launching the daemon."""
    from srt.daemon import daemon as srt_d

    daemon = srt_d.SmallRadioTelescopeDaemon(configuration_dir, configuration_dict)
    daemon.srt_daemon_main()


def get_all_objects(config_file="config/sky_coords.csv",):
    table = Table.read(Path(root_folder, config_file), format="ascii.csv")
    all_objects = ["Sun", "Moon"]
    for index, row in enumerate(table):
        name = row["name"]
        all_objects.append(name)
    return all_objects

# ---------------------------------------------------------------------------
# Antenna state panel helpers
# ---------------------------------------------------------------------------
 
# Map FSM state strings to Bootstrap contextual colors.
_STATE_COLOR = {
    "disconnected": "secondary",
    "connecting": "secondary",
    "startup_sync": "warning",
    "ready": "primary",
    "slewing": "info",
    "tracking": "success",
    "calibrating": "warning",
    "fault": "danger",
    "recovering": "warning",
    "shutdown": "dark",
}
 
_CALSTS_COLOR = {
    "Not Calibrated": "danger",
    "Calibrating Now": "warning",
    "Calibration OK": "success",
}
 
_BRAKE_COLOR = {True: "danger", False: "success"}   # True = brake ON (motion blocked)
_ESTOP_COLOR = {True: "danger", False: "success"}
 
 
def _indicator_pill(label, active, color_true="danger", color_false="success",
                    true_text=None, false_text=None):
    """Renders a small labelled pill badge.
 
    active=True  → color_true (e.g. red for a fault, green for 'brake off')
    active=False → color_false
    true_text / false_text override the displayed text when active/inactive.
    """
    color = color_true if active else color_false
    text = (true_text if active else false_text) or label
    return dbc.Badge(
        text,
        color=color,
        className="me-1 mb-1",
        style={"fontSize": "0.72rem", "letterSpacing": "0.04em"},
    )
 
 
def generate_antenna_state_panel():
    """Placeholder layout; content is filled by the update callback."""
    return dbc.Card(
        [
            dbc.CardHeader(
                html.H5("Antenna State", className="mb-0"),
                style={},
            ),
            dbc.CardBody(
                html.Div(id="antenna-state-body"),
                style={},
            ),
        ],
        className="mb-2",
        style={},
    )
 
 
def _build_antenna_state_body(motor_status: dict[str, any] | None):
    """Build the antenna state panel content from a motor_status dict from daemon thread.
 
    motor_status keys:
        "beam_width",
        "location",
        "motor_azel",
        "motor_cmd_azel",
        "vlsr",
        "object_locs",
        "object_time_locs",
        "az_limits",
        "el_limits",
        "stow_loc",
        "cal_loc",
        "horizon_points",
        "center_frequency",
        "frequency_correction",
        "bandwidth",
        "motor_offsets",
        "queued_item",
        "queue_size",
        "emergency_contact",
        "error_logs",
        "temp_cal",
        "temp_sys",
        "cal_power",
        "n_point_data",
        "beam_switch_data",
        "pointing_error",
        "pointing_error_history",
        "rotor_diagnostics",
        "rotor_fsm_status",
        "observation_events",
        "time",
        "command_history",
        "serial_communications"
    """
    if motor_status is None:
        return html.P("Motor status unavailable.", style={"color": "#555"})
 
    fsm_state: str = motor_status.get("rotor_fsm_status", {}).get("state", "Unknown")
    state_color: str = _STATE_COLOR.get(fsm_state, "secondary")

    rotor_diagnostics: dict[str, any] = motor_status.get("rotor_diagnostics", {})
    # rotor_diagnostics keys = [
        #     "mode", "CalSts",
        #     "AzBrkOn", "ElBrkOn", "EmStopOn",
        #     "azerr", "elerr", "amp_currents",
        #     "ElUpPreLim", "ElDnPreLim", "ElUpFinLim", "ElDnFinLim",
        #     "AzCwPreLim", "AzCcwPreLim", "AzCwFinLim", "AzCcwFinLim",
        #     "AzLT180", "SimMode", "safe_mode",
        # ]
 
    cal_sts: str = rotor_diagnostics.get("CalSts", "Unknown")
    cal_color: str = _CALSTS_COLOR.get(cal_sts, "secondary")
 
    # RunningTask: 0=Stop, 1=Track (5 is never actually set in firmware)
    mode: str = rotor_diagnostics.get("mode", "Unknown")
    mode_color: str = "success" if mode == "Track" else ("secondary" if mode == "Stop" else "warning")
 
    az_brk: bool | None = rotor_diagnostics.get("AzBrkOn", None)
    el_brk: bool | None = rotor_diagnostics.get("ElBrkOn", None)
    estop: bool | None = rotor_diagnostics.get("EmStopOn", None)
    sim: bool | None = rotor_diagnostics.get("SimMode", None)
 
    az: float = motor_status.get("motor_azel",(float("nan"), float("nan")))[0]
    el: float = motor_status.get("motor_azel",(float("nan"), float("nan")))[1]
    azerr: float = rotor_diagnostics.get("azerr", float("nan"))
    elerr: float = rotor_diagnostics.get("elerr", float("nan"))
    last_err: str = motor_status.get("rotor_fsm_status", {}).get("last_error", "")
    last_trans: str = motor_status.get("rotor_fsm_status", {}).get("last_transition", "")
    safe_mode: bool = rotor_diagnostics.get("safe_mode", False)
    retry_count: int = motor_status.get("fsm_status", {}).get("retry_count", 0)
 
    # Limit switch states — show only active ones prominently, dim inactive
    limit_switches = {
        "El↑ Pre": rotor_diagnostics.get("ElUpPreLim", False),
        "El↓ Pre": rotor_diagnostics.get("ElDnPreLim", False),
        "El↑ Fin": rotor_diagnostics.get("ElUpFinLim", False),
        "El↓ Fin": rotor_diagnostics.get("ElDnFinLim", False),
        "Az CW Pre": rotor_diagnostics.get("AzCwPreLim", False),
        "Az CCW Pre": rotor_diagnostics.get("AzCcwPreLim", False),
        "Az CW Fin": rotor_diagnostics.get("AzCwFinLim", False),
        "Az CCW Fin": rotor_diagnostics.get("AzCcwFinLim", False),
    }
    any_limit = any(limit_switches.values())
 
    amp_currents = rotor_diagnostics.get("amp_currents", {})
 
    # --- Build layout ---
 
    top_badges = html.Div(
        [
            dbc.Badge(fsm_state.upper(), color=state_color,
                      className="me-2 mb-1", style={"fontSize": "0.85rem"}),
            dbc.Badge(cal_sts, color=cal_color,
                      className="me-2 mb-1", style={"fontSize": "0.85rem"}),
            dbc.Badge(f"Loop: {mode}", color=mode_color,
                      className="me-2 mb-1", style={"fontSize": "0.85rem"}),
            *([ dbc.Badge("SAFE MODE", color="warning", className="me-2 mb-1",
                           style={"fontSize": "0.85rem"}) ] if safe_mode else []),
            *([ dbc.Badge("SIM", color="info", className="me-2 mb-1",
                           style={"fontSize": "0.85rem"}) ] if sim else []),
        ],
        className="mb-2",
    )
 
    position_row = dbc.Row(
        [
            dbc.Col([
                html.Small("Az", style={"color": "#555"}),
                html.Div(f"{az:.3f}°", style={"fontSize": "1.1rem", "fontFamily": "monospace"}),
                html.Small(f"err: {azerr:+.4f}m°", style={"color": "#666", "fontFamily": "monospace"}),
            ], width=3),
            dbc.Col([
                html.Small("El", style={"color": "#555"}),
                html.Div(f"{el:.3f}°", style={"fontSize": "1.1rem", "fontFamily": "monospace"}),
                html.Small(f"err: {elerr:+.4f}m°", style={"color": "#666", "fontFamily": "monospace"}),
            ], width=3),
            dbc.Col([
                html.Small("Az Brake", style={"color": "#555"}),
                html.Div(_indicator_pill(
                    "Az Brake",
                    az_brk if az_brk is not None else False,
                    color_true="danger", color_false="success",
                    true_text="ON (locked)", false_text="OFF (free)",
                )) if az_brk is not None else html.Div("?", style={"color": "#999"}),
            ], width=3),
            dbc.Col([
                html.Small("El Brake", style={"color": "#555"}),
                html.Div(_indicator_pill(
                    "El Brake",
                    el_brk if el_brk is not None else False,
                    color_true="danger", color_false="success",
                    true_text="ON (locked)", false_text="OFF (free)",
                )) if el_brk is not None else html.Div("?", style={"color": "#999"}),
            ], width=3),
        ],
        className="mb-2",
    )
 
    estop_row = html.Div(
        [
            _indicator_pill("Physical E-Stop Button", estop if estop is not None else False,
                            color_true="danger", color_false="success",
                            true_text="⚠ Physical E-STOP Button ACTIVE", false_text="Physical E-Stop Button: clear"),
            _indicator_pill("Retries", retry_count > 0, color_true="warning",
                            color_false="success",
                            true_text=f"Retries: {retry_count}", false_text="Comms OK"),
        ],
        className="mb-2",
    )
 
    # Limit switch section — collapsed grid, highlighted red when any active
    limit_pills = html.Div(
        [_indicator_pill(
            name, active,
            color_true="danger", color_false="secondary",
            true_text=f"⚠ {name}", false_text=name,
        ) for name, active in limit_switches.items()],
        className="mb-1",
    )
    limit_section = html.Div(
        [
            html.Small(
                "Limit Switches" + (" — ⚠ ACTIVE" if any_limit else " — clear"),
                style={"color": "#e05050", "display": "block", "marginBottom": "3px"},
            ),
            limit_pills,
        ],
        className="mb-2",
    )
 
    # Amp current summary (3 amps, commanded vs actual)
    amp_rows = []
    for amp_id in ("2A01", "2A02", "2A03"):
        info = amp_currents.get(amp_id, {})
        cmd = info.get("commanded", None)
        act = info.get("actual", None)
        if cmd is None or cmd == -999999:
            cmd_str = "—"
        else:
            cmd_str = str(cmd)
        if act is None or act == -999999:
            act_str = "—"
        else:
            act_str = str(act)
        amp_rows.append(
            dbc.Row([
                dbc.Col(html.Small(amp_id, style={"color": "#555"}), width=2),
                dbc.Col(html.Small(f"cmd: {cmd_str}", style={"fontFamily": "monospace"}), width=5),
                dbc.Col(html.Small(f"act: {act_str}", style={"fontFamily": "monospace"}), width=5),
            ], className="mb-0")
        )
    amp_section = html.Div(
        [html.Small("Amplifier Currents", style={"color": "#555", "display": "block", "marginBottom": "3px"})]
        + amp_rows,
        className="mb-2",
    )
 
    error_section = html.Div()
    if last_err:
        error_section = dbc.Alert(
            [html.Strong("Last error: "), last_err],
            color="danger",
            className="mb-2 py-1 px-2",
            style={"fontSize": "0.8rem"},
        )
 
    meta = html.Small(
        f"Last transition: {last_trans}",
        style={"color": "#888", "display": "block"},
    )
 
    return html.Div([top_badges, position_row, estop_row, limit_section, amp_section, error_section, meta])
 


def generate_first_row():
    """Generates First Row (Power and Spectrum) Display

    Returns
    -------
    Div Containing First Row Objects
    """
    return html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [dcc.Graph(id="power-graph")],
                        className="pretty_container six columns",
                    ),
                    html.Div(
                        [
                            dcc.Graph(id="cal-spectrum-histogram"),
                            dcc.Graph(id="raw-spectrum-histogram"),
                        ],
                        className="pretty_container six columns",
                    ),
                ],
                className="flex-display",
                style={
                    "justify-content": "center",
                    "margin": "5px",
                },
            ),
        ]
    )


def generate_srt_azel():
    """Generates AzEl  Display

    Returns
    -------
    Div: html.Div 
        containing n point graph az el pointing
    """
    return html.Div(
        html.Div(
            [
                html.Div(
                    [
                        dcc.Graph(id="az-el-graph"),
                    ],
                    className="pretty_container twelve columns",
                ),

            ],
            className="flex-display",
            style={"margin": dict(l=10, r=5, t=5, b=5)},
        ),
    )

def generate_npointlayout():
    """Generates N Point Display

    Returns
    -------
    Div: html.Div 
        containing n point graph if srt
    """
    return html.Div(
        [
            html.Div(
                [
                    dcc.Store(id="npoint_info", storage_type="session"),
                    html.Div(
                        [dcc.Graph(id="npoint-graph")],
                        className="pretty_container six columns",
                    ),
                    # html.Div(
                    #     [dcc.Graph(id="beamsswitch-graph")],
                    #     className="pretty_container six columns",
                    # ),
                ],
                className="flex-display",
                style={
                    "justify-content": "left",
                    "margin": "5px",
                },
            ),
        ]
    )

def generate_srt_second_row():
    """Generates Mount Status Display and zoomed in map

    Returns
    -------
    Div: html.Div 
        containing mount status diagnostics
    """
    return html.Div(
        [
            html.Div(
                [
                    html.Div(
                        generate_antenna_state_panel(),
                        className="pretty_container six columns",
                        style={"alignSelf": "stretch"},
                    ),
                    html.Div(
                        [dcc.Graph(id="zoom-graph")],
                        className="pretty_container six columns",
                    ),
                ],
                className="flex-display",
                style={
                    "justify-content": "left",
                    "margin": "5px",
                },
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.Div(
                                [
                                    html.H5(
                                        "Observation Events",
                                        style={"display": "inline-block", "marginRight": "10px"},
                                    ),
                                    dbc.Button(
                                        "Download CSV",
                                        id="btn-export-obs",
                                        size="sm",
                                        color="secondary",
                                    ),
                                ],
                                style={"display": "flex", "alignItems": "center", "gap": "10px"},
                            ),
                            html.Div(
                                id="obs-events-panel",
                                style={"maxHeight": "220px", "overflowY": "auto"},
                            ),
                        ],
                        className="pretty_container twelve columns",
                    ),
                ],
                className="flex-display",
                style={"justify-content": "left", "margin": "5px"},
            ),
        ]
    )


def generate_webcam_row():
    """Generates webcam live-view row."""
    return html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [
                            html.H5("Live Webcam"),
                            html.Img(
                                id="live-webcam-feed",
                                src="/video_feed",
                                style={
                                    "width": "100%",
                                    "height": "auto",
                                    "maxHeight": "480px",
                                    "objectFit": "contain",
                                    "backgroundColor": "black",
                                    "borderRadius": "4px",
                                },
                            ),
                            html.Small(
                                "MJPEG stream served by dashboard backend",
                                style={"color": "gray"},
                            ),
                        ],
                        className="pretty_container twelve columns",
                    )
                ],
                className="flex-display",
                style={"justify-content": "left", "margin": "5px"},
            )
        ]
    )


def generate_second_row():
    """Generates Second Row (AzEl and AzEl Zoom) Display

    Returns
    -------
    Div Containing Second Row Objects
    """
    return html.Div(
        html.Div(
            [
                html.Div(
                    [dcc.Graph(id="az-el-graph")],
                    className="pretty_container six columns",
                ),

                html.Div(
                    [dcc.Graph(id="zoom-graph")],
                    className="pretty_container six columns",
                ),
            ],
            className="flex-display",
            style={"margin": dict(l=10, r=5, t=5, b=5)},
        ),
    )


def generate_third_row():
    """Generates Third Row (Pointing Error Time) Display

    Returns
    -------
    Div Containing Third Row Objects
    """
    return html.Div([
        html.Div(
                    [
                        html.Label("Select Time Range in Minutes", style={
                            "color": "darkgray", "margin-top": "10px", "margin-left": "20px"}),
                        dcc.Slider(5, 60, 5, id="timeinput"),
                        dcc.Graph(id="az-el-elevation")],
                    className="pretty_container twelve columns",
                    ),
    ],
        className="flex-display",
        style={"margin": dict(l=10, r=5, t=5, b=5)},
    )


def generate_popups():
    """Generates all 'Pop-up' Modal Components

    Returns
    -------
    Div Containing all Modal Components
    """
    all_objects = get_all_objects()

    return html.Div(
        [dbc.Modal(
            [
                dbc.ModalHeader("Point at Object"),
                dbc.ModalBody(
                    [
                        html.H5("Confirm Pointing to This Object?"),
                        dcc.RadioItems(
                            options=[
                                {"label": "Direct Point", "value": ""},
                                {"label": "N-Point Scan", "value": " n"},
                                {"label": "Beam-switch", "value": " b"},
                            ],
                            id="point-options",
                            value="",
                        ),
                    ]
                ),
                dbc.ModalFooter(
                    [
                        dbc.Button(
                            "Yes",
                            id="az-el-graph-btn-yes",
                            className="ml-auto",
                            # block=True,
                            color="primary",
                        ),
                        dbc.Button(
                            "No",
                            id="az-el-graph-btn-no",
                            className="ml-auto",
                            # block=True,
                            color="secondary",
                        ),
                    ]
                ),
            ],
            id="az-el-graph-modal",
        ),

            dbc.Modal(
                [
                    dbc.ModalHeader("Select Object to Observe"),
                    dbc.ModalBody(
                        [
                            dcc.Dropdown(
                                all_objects, placeholder='Select an Object', id='obj-dropdown')
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="obs-obj-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="obs-obj-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]

                    ),
                ],
                id="obs-obj-modal",

        ),

            dbc.Modal(
                [
                    dbc.ModalHeader("Enter Aim Coordinates"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="obj-az",
                                type="number",
                                debounce=True,
                                placeholder="Azimuth",
                            ),
                            dcc.Input(
                                id="obj-el",
                                type="number",
                                debounce=True,
                                placeholder="Elevation",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="obs-coords-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="obs-coords-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="obs-coords-modal",
        ),



            dbc.Modal(
                [
                    dbc.ModalHeader("Enter Location Coordinates"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="coords-lat",
                                type="number",
                                debounce=True,
                                placeholder="Latitude",
                            ),
                            dcc.Input(
                                id="coords-long",
                                type="number",
                                debounce=True,
                                placeholder="Longitude",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="coords-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="coords-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="coords-modal",
        ),


            dbc.Modal(
                [
                    dbc.ModalHeader("Enter Azimuth and Elevation"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="azimuth",
                                type="number",
                                debounce=True,
                                placeholder="Azimuth",
                            ),
                            dcc.Input(
                                id="elevation",
                                type="number",
                                debounce=True,
                                placeholder="Elevation",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="point-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="point-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="point-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Enter the New Center Frequency"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="frequency",
                                type="number",
                                debounce=True,
                                placeholder="Center Frequency (MHz)",
                                style={"width": "100%"},
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="freq-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="freq-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="freq-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Enter the New Sample Frequency"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="samples",
                                type="number",
                                debounce=True,
                                placeholder="Sample Frequency (MHz)",
                                style={"width": "100%"},
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="samp-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="samp-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="samp-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Enter the Motor Offsets"),
                    dbc.ModalBody(
                        [
                            dcc.Input(
                                id="offset-azimuth",
                                type="number",
                                debounce=True,
                                placeholder="Azimuth Offset (deg)",
                            ),
                            dcc.Input(
                                id="offset-elevation",
                                type="number",
                                debounce=True,
                                placeholder="Elevation Offset (deg)",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="offset-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="offset-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="offset-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Start Recording"),
                    dbc.ModalBody(
                        [
                            html.H5("Select a File Type"),
                            dcc.RadioItems(
                                options=[
                                    {"label": "Digital RF (Raw Data)",
                                     "value": ""},
                                    {
                                        "label": ".rad Format (Spectrum)",
                                        "value": "*.rad",
                                    },
                                    {
                                        "label": ".fits Format (Spectrum)",
                                        "value": "*.fits",
                                    },
                                ],
                                id="record-options",
                                value="",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="record-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="record-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="record-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Command File"),
                    dbc.ModalBody(
                        [
                            html.H4("Upload Command File"),
                            dcc.Upload(
                                id="upload-data",
                                children=html.Div(
                                    ["Drag and Drop or ",
                                        html.A("Select Files")]
                                ),
                                style={
                                    "width": "95%",
                                    "hSystemeight": "60px",
                                    "lineHeight": "60px",
                                    "borderWidth": "1px",
                                    "borderStyle": "dashed",
                                    "borderRadius": "5px",
                                    "textAlign": "center",
                                    "margin": "10px",
                                },
                                # Allow multiple files to be uploaded
                                multiple=False,
                            ),
                            html.Div(
                                id="output-data-upload", style={"text-align": "center"}
                            ),
                        ]
                    ),
                ],
                id="cmd-file-modal",
        ),
            dbc.Modal(
                [
                    dbc.ModalHeader("Start Daemon"),
                    dbc.ModalBody(
                        [
                            html.H6(
                                "Are you sure you want to try to start the background SRT Process?"
                            ),
                            html.H6(
                                "If the process is already running, this may fail"),
                            html.H5(
                                "Process is Already Running",
                                id="start-warning",
                                style={"text-align": "center"},
                            ),
                            dcc.Dropdown(
                                options=[],
                                placeholder="Select a Config File",
                                id="start-config-file",
                            ),
                        ]
                    ),
                    dbc.ModalFooter(
                        [
                            dbc.Button(
                                "Yes",
                                id="start-btn-yes",
                                className="ml-auto",
                                # block=True,
                                color="primary",
                            ),
                            dbc.Button(
                                "No",
                                id="start-btn-no",
                                className="ml-auto",
                                # block=True,
                                color="secondary",
                            ),
                        ]
                    ),
                ],
                id="start-modal",
        ),
        ]
    )


def generate_layout(config_dict=None):
    """Generates the Basic Layout for the Monitor Page

    Parameters
    ----------
    config_dict : dict, optional
        Configuration dictionary

    Returns
    -------
    layout: html.div
        Monitor Page Layout
    """
    # Check if we should hide radio plots
    radio_enabled = True
    webcam_enabled = False
    if config_dict:
        control_profile = config_dict.get("CONTROL_PROFILE", "FULL_SRT").upper()
        radio_enabled = control_profile != "POINTING_ONLY"
        webcam_enabled = bool(config_dict.get("WEBCAM_ENABLE", False))
    
    drop_down_buttons_srt = {
        # "Observe": [
        #     dbc.DropdownMenuItem("Select Object", id="btn-obs-obj"),
        #     dbc.DropdownMenuItem("Enter Coordinates", id="btn-obs-coords"),
        # ],
        "Antenna": [
            dbc.DropdownMenuItem("Stow", id="btn-stow"),
            dbc.DropdownMenuItem("Set AzEl", id="btn-point-azel"),
            dbc.DropdownMenuItem("Set Offsets", id="btn-set-offset"),
        ],
        "Radio": [
            dbc.DropdownMenuItem("Set Frequency", id="btn-set-freq"),
            dbc.DropdownMenuItem("Set Bandwidth", id="btn-set-samp"),
        ],
        "Routine": [
            dbc.DropdownMenuItem("Start Recording", id="btn-start-record"),
            dbc.DropdownMenuItem("Stop Recording", id="btn-stop-record"),
            dbc.DropdownMenuItem("Calibrate Encoders", id="btn-calibrate"),
            dbc.DropdownMenuItem("Upload CMD File", id="btn-cmd-file"),
        ],
        "Power": [
            dbc.DropdownMenuItem("Start Daemon", id="btn-start"),
            dbc.DropdownMenuItem("Shutdown", id="btn-quit"),
        ],
    }

    # Hide radio menus in POINTING_ONLY mode.
    if not radio_enabled:
        drop_down_buttons_srt.pop("Radio", None)
    
    base_srt = [
        generate_navbar(
            drop_down_buttons_srt,
            extra_buttons=[
                dbc.Button(
                    "E-STOP",
                    id="btn-estop",
                    color="danger",
                    className="m-1",
                    style={"fontWeight": "bold"},
                )
            ],
        ),
        dbc.Alert("Recording", color="danger",
                  id="recording-alert", is_open=False),
    ]
    if radio_enabled:
        base_srt.append(generate_first_row())
    base_srt.extend([
        generate_srt_azel(),
        generate_srt_second_row(),
        generate_third_row(),
        *( [generate_webcam_row()] if webcam_enabled else [] ),
        generate_popups(),
        html.Div(id="signal", style={"display": "none"}),
        dcc.Download(id="obs-events-download"),
    ])
    
    layout = html.Div(base_srt)
    
    return layout


def register_callbacks(
    app, config, status_thread, command_thread #, raw_spectrum_thread, cal_spectrum_thread
):
    """Registers the Callbacks for the Monitor Page

    Parameters
    ----------
    app : Dash Object
        Dash Object to Set Up Callbacks to
    config : dict
        Contains All Settings for Dashboard / Daemon
    status_thread : Thread
        Thread for Getting Status from Daemon
    command_thread : Thread
        Thread for Sending Commands to Daemon
    raw_spectrum_thread : Thread
        Thread for Getting Raw Spectrum Data from Daemon
    cal_spectrum_thread : Thread
        Thread for Getting Calibrated Spectrum Data from Daemon

    Returns
    -------
    None
    """

    control_profile = str(config.get("CONTROL_PROFILE", "FULL_SRT")).upper()
    radio_enabled = control_profile != "POINTING_ONLY"

    # if radio_enabled:
    #     @app.callback(
    #         Output("cal-spectrum-histogram", "figure"),
    #         [Input("interval-component", "n_intervals")],
    #     )
    #     def update_cal_spectrum_histogram(n):
    #         spectrum = cal_spectrum_thread.get_spectrum()
    #         status = status_thread.get_status()
    #         if status is None or spectrum is None:
    #             return ""
    #         bandwidth = float(status["bandwidth"])
    #         cf = float(status["center_frequency"])
    #         return generate_spectrum_graph(bandwidth, cf, spectrum, is_spec_cal=True)

    #     @app.callback(
    #         Output("raw-spectrum-histogram", "figure"),
    #         [Input("interval-component", "n_intervals")],
    #     )
    #     def update_raw_spectrum_histogram(n):

    #         spectrum = raw_spectrum_thread.get_spectrum()
    #         status = status_thread.get_status()
    #         if status is None or spectrum is None:
    #             return ""
    #         bandwidth = float(status["bandwidth"])
    #         cf = float(status["center_frequency"])
    #         return generate_spectrum_graph(bandwidth, cf, spectrum, is_spec_cal=False)

    #     @app.callback(
    #         Output("power-graph",
    #                "figure"), [Input("interval-component", "n_intervals")]
    #     )
    #     def update_power_graph(n):
    #         status = status_thread.get_status()
    #         if status is None:
    #             return ""
    #         tsys = float(status["temp_sys"])
    #         tcal = float(status["temp_cal"])
    #         cal_pwr = float(status["cal_power"])
    #         spectrum_history = raw_spectrum_thread.get_history()
    #         if spectrum_history is None:
    #             return ""
    #         return generate_power_history_graph(tsys, tcal, cal_pwr, spectrum_history)

    @app.callback(
        Output("mount-status-panel", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_mount_status_panel(n):
        """Render a compact status table for Caltech 6m diagnostics."""
        status = status_thread.get_status()
        if status is None:
            return html.Div("Waiting for daemon status")

        diagnostics = status.get("rotor_diagnostics", {}) or {}
        fsm = status.get("rotor_fsm_status", {}) or diagnostics.get("fsm_status", {})
        point_err = status.get("pointing_error")
        rows = [
            ["Mode", diagnostics.get("mode", "-")],
            ["Calibration", diagnostics.get("CalSts", "-")],
            ["Az Brake", "ON" if diagnostics.get("AzBrkOn") else "OFF"],
            ["El Brake", "ON" if diagnostics.get("ElBrkOn") else "OFF"],
            ["E-Stop", "ON" if diagnostics.get("EmStopOn") else "OFF"],
            [
                "Motor Az/El (deg)",
                f"{status['motor_azel'][0]:.3f}, {status['motor_azel'][1]:.3f}",
            ],
            [
                "Cmd Az/El (deg)",
                f"{status['motor_cmd_azel'][0]:.3f}, {status['motor_cmd_azel'][1]:.3f}",
            ],
            [
                "Pointing Err (mdeg)",
                "-" if not (point_err and len(point_err) == 2) else f"{point_err[0]:.1f}, {point_err[1]:.1f}",
            ],
            ["FSM State", (fsm or {}).get("state", "-")],
            ["FSM Last Transition", (fsm or {}).get("last_transition", "-")],
            ["FSM Retries", (fsm or {}).get("retry_count", "-")],
        ]

        amp = diagnostics.get("amp_currents")
        if isinstance(amp, dict):
            for axis in ["2A01", "2A02", "2A03"]:
                vals = amp.get(axis, {}) if isinstance(amp.get(axis), dict) else {}
                rows.append(
                    [
                        f"{axis} Current",
                        f"cmd {vals.get('commanded', '?')}, act {vals.get('actual', '?')}",
                    ]
                )

        return dbc.Table(
            [
                html.Thead(html.Tr([html.Th("Metric"), html.Th("Value")])),
                html.Tbody([html.Tr([html.Td(k), html.Td(v)]) for k, v in rows]),
            ],
            bordered=True,
            striped=True,
            hover=True,
            size="sm",
        )

    @app.callback(
        Output("obs-events-panel", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_obs_events_panel(n):
        status = status_thread.get_status()
        if status is None:
            return html.Div("Waiting for daemon status")
        events = status.get("observation_events", [])
        if not events:
            return html.Div("No observation events yet")

        rows = []
        for event in reversed(events[-20:]):
            iso_time = event.get("iso_time", "")
            event_name = event.get("event", "")
            meta = event.get("metadata", {}) or {}
            detail = ""
            if meta.get("object"):
                detail = f"object={meta.get('object')}"
            else:
                target = meta.get("target_azel")
                if isinstance(target, (list, tuple)) and len(target) == 2:
                    detail = f"az={float(target[0]):.3f}, el={float(target[1]):.3f}"

            rows.append(
                html.Div(
                    [
                        html.Span(iso_time, style={"color": "#666", "marginRight": "8px"}),
                        html.Strong(event_name),
                        html.Span(f"  {detail}" if detail else "", style={"marginLeft": "8px"}),
                    ],
                    style={"padding": "4px 0", "borderBottom": "1px solid #eee"},
                )
            )

        return html.Div(rows)

    @app.callback(
        Output("btn-calibrate", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_encoder_calibrate_button_label(n):
        status = status_thread.get_status()
        if status is None:
            return "Calibrate Encoders"
        diagnostics = status.get("rotor_diagnostics", {}) or {}
        cal_sts = diagnostics.get("CalSts")
        if cal_sts:
            return f"Cal Encoders ({cal_sts})"
        return "Calibrate Encoders"

    @app.callback(
        Output("antenna-state-body", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_antenna_state(n):
        status = status_thread.get_status()
        if status is None:
            return _build_antenna_state_body(None)
        motor_status = status.get("motor_status", status)
        return _build_antenna_state_body(motor_status)

    @app.callback(
        Output("obs-events-download", "data"),
        [Input("btn-export-obs", "n_clicks")],
        prevent_initial_call=True,
    )
    def export_observation_events_csv(n_clicks):
        status = status_thread.get_status()
        if status is None:
            raise PreventUpdate
        events = status.get("observation_events", [])
        if not events:
            raise PreventUpdate

        buffer = io.StringIO()
        writer = csv.writer(buffer)
        writer.writerow(
            [
                "iso_time",
                "event",
                "time",
                "sequence",
                "object",
                "target_az",
                "target_el",
                "point_index",
                "point_total",
                "end_reason",
            ]
        )

        for entry in events:
            meta = entry.get("metadata", {}) or {}
            target = meta.get("target_azel") if isinstance(meta, dict) else None
            target_az = ""
            target_el = ""
            if isinstance(target, (list, tuple)) and len(target) == 2:
                target_az = target[0]
                target_el = target[1]

            writer.writerow(
                [
                    entry.get("iso_time", ""),
                    entry.get("event", ""),
                    entry.get("time", ""),
                    meta.get("sequence", "") if isinstance(meta, dict) else "",
                    meta.get("object", "") if isinstance(meta, dict) else "",
                    target_az,
                    target_el,
                    meta.get("point_index", "") if isinstance(meta, dict) else "",
                    meta.get("point_total", "") if isinstance(meta, dict) else "",
                    meta.get("end_reason", "") if isinstance(meta, dict) else "",
                ]
            )

        filename = f"observation_events_{int(time())}.csv"
        return dcc.send_string(buffer.getvalue(), filename)

    @app.callback(
        Output("start-warning", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_start_daemon_warning(n):
        status = status_thread.get_status()
        if status is None:
            return "SRT Daemon Not Detected"
        latest_time = status["time"]
        if time() - latest_time < 10:
            return "SRT Daemon Already On!"
        else:
            return "SRT Daemon Disconnected"

    @app.callback(
        Output("start-config-file", "options"),
        [Input("interval-component", "n_intervals")],
    )
    def update_start_daemon_options(n):
        files = [
            {"label": file.name, "value": file.name}
            for file in Path(config["CONFIG_DIR"]).glob("*")
            if file.is_file() and file.name.endswith(".yaml") and file.name != "schema.yaml"
        ]
        return files

    @app.callback(
        Output("output-data-upload", "children"),
        [Input("upload-data", "contents")],
        [State("upload-data", "filename"),
         State("upload-data", "last_modified"),
         State("auth-session", "data")],
    )
    def update_output(contents, name, date, session_data):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return html.Div(["Unauthorized: Please log in to upload commands"])
        
        if contents is not None:
            content_type, content_string = contents.split(",")
            decoded = base64.b64decode(content_string)
            try:
                if "txt" in name or "cmd" in name:
                    # Assume that the user uploaded a txt file
                    loaded_file = io.StringIO(decoded.decode("utf-8"))
                    lines = [line.rstrip() for line in loaded_file]
                    for line in lines:
                        command_thread.add_to_queue(line)
                    return html.Div(["Command File Uploaded Successfully"])
            except Exception as e:
                print(e)
            return html.Div(["There was an error processing this file."])
        else:
            return html.Div(["Awaiting Command File"])


    @app.callback(
        Output("az-el-graph", "figure"),
        [Input("interval-component", "n_intervals")],
    )
    def update_az_el_graph_srt(n):
        status = status_thread.get_status()
        if status is not None:
            future_points = status.get("object_time_locs", {})
            return generate_az_el_graph(
                status["az_limits"],
                status["el_limits"],
                status["object_locs"],
                status["motor_azel"],
                status["stow_loc"],
                status["cal_loc"],
                status["horizon_points"],
                status["beam_width"],
                future_points,
            )
        return ""


    @app.callback(
        Output("zoom-graph",
               "figure"), [Input("interval-component", "n_intervals")]
    )
    def update_zoom_graph(n):
        status = status_thread.get_status()
        if status is not None:
            return generate_zoom_graph(
                status["az_limits"],
                status["el_limits"],
                status["object_locs"],
                status["motor_azel"],
                status["stow_loc"],
                status["cal_loc"],
                status["horizon_points"],
                status["beam_width"],
            )
        return ""

    @app.callback(
        Output("az-el-elevation", "figure"),
        [
            Input("interval-component", "n_intervals"),
            Input("timeinput", "value"),
        ]
    )
    def update_az_el_time_graph(n, range):
        status = status_thread.get_status()
        if status is not None:
            error_hist = status.get("pointing_error_history", [])
            current_err = status.get("pointing_error")
            if (not error_hist) and current_err is not None and len(current_err) == 2:
                error_hist = [
                    {
                        "time": time(),
                        "azerr_mdeg": float(current_err[0]),
                        "elerr_mdeg": float(current_err[1]),
                    }
                ]
            if error_hist:
                return generate_pointing_error_graph(error_hist, range)
        return ""

    @ app.callback(
        Output("az-el-graph-modal", "is_open"),
        [
            Input("az-el-graph", "clickData"),
            Input("az-el-graph-btn-yes", "n_clicks"),
            Input("az-el-graph-btn-no", "n_clicks"),
        ],
        [State("az-el-graph-modal", "is_open"),
         State("point-options", "value"),
         State("auth-session", "data")],
    )
    def az_el_click_func(clickData, n_clicks_yes, n_clicks_no, is_open, mode, session_data):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return is_open
        
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "az-el-graph-btn-yes":
                command_thread.add_to_queue(
                    f"{clickData['points'][0]['text']}{mode}")
            if (
                n_clicks_yes
                or n_clicks_no
                or (
                    clickData
                    and not clickData["points"][0]["text"] == "Antenna Location"
                )
            ):
                return not is_open
            return is_open

    @app.callback(
        Output("point-modal", "is_open"),
        [
            Input("btn-point-azel", "n_clicks"),
            Input("point-btn-yes", "n_clicks"),
            Input("point-btn-no", "n_clicks"),
        ],
        [
            State("point-modal", "is_open"),
            State("azimuth", "value"),
            State("elevation", "value"),
            State("auth-session", "data"),
        ],
    )
    def point_click_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, az, el, session_data):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return is_open
        
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "point-btn-yes":
                command_thread.add_to_queue(f"azel {az} {el}")
            if n_clicks_yes or n_clicks_no or n_clicks_btn:
                return not is_open
            return is_open

    if radio_enabled:
        @app.callback(
            Output("freq-modal", "is_open"),
            [
                Input("btn-set-freq", "n_clicks"),
                Input("freq-btn-yes", "n_clicks"),
                Input("freq-btn-no", "n_clicks"),
            ],
            [
                State("freq-modal", "is_open"),
                State("frequency", "value"),
                State("auth-session", "data"),
            ],
        )
        def freq_click_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, freq, session_data):
            # Reject if not authenticated
            if not (session_data and session_data.get("authenticated")):
                return is_open

            ctx = dash.callback_context
            if not ctx.triggered:
                return is_open
            else:
                button_id = ctx.triggered[0]["prop_id"].split(".")[0]
                if button_id == "freq-btn-yes":
                    command_thread.add_to_queue(f"freq {freq}")
                if n_clicks_yes or n_clicks_no or n_clicks_btn:
                    return not is_open
                return is_open

        @ app.callback(
            Output("samp-modal", "is_open"),
            [
                Input("btn-set-samp", "n_clicks"),
                Input("samp-btn-yes", "n_clicks"),
                Input("samp-btn-no", "n_clicks"),
            ],
            [
                State("samp-modal", "is_open"),
                State("samples", "value"),
                State("auth-session", "data"),
            ],
        )
        def samp_click_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, samp, session_data):
            # Reject if not authenticated
            if not (session_data and session_data.get("authenticated")):
                return is_open

            ctx = dash.callback_context
            if not ctx.triggered:
                return is_open
            else:
                button_id = ctx.triggered[0]["prop_id"].split(".")[0]
                if button_id == "samp-btn-yes":
                    command_thread.add_to_queue(f"samp {samp}")
                if n_clicks_yes or n_clicks_no or n_clicks_btn:
                    return not is_open
                return is_open

    @ app.callback(
        Output("offset-modal", "is_open"),
        [
            Input("btn-set-offset", "n_clicks"),
            Input("offset-btn-yes", "n_clicks"),
            Input("offset-btn-no", "n_clicks"),
        ],
        [
            State("offset-modal", "is_open"),
            State("offset-azimuth", "value"),
            State("offset-elevation", "value"),
            State("auth-session", "data"),
        ],
    )
    def offset_click_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, az, el, session_data):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return is_open
        
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "offset-btn-yes":
                command_thread.add_to_queue(f"offset {az} {el}")
            if n_clicks_yes or n_clicks_no or n_clicks_btn:
                return not is_open
            return is_open

    @ app.callback(
        Output("record-modal", "is_open"),
        [
            Input("btn-start-record", "n_clicks"),
            Input("record-btn-yes", "n_clicks"),
            Input("record-btn-no", "n_clicks"),
        ],
        [State("record-modal", "is_open"), State("record-options",
                                                 "value"), State("recording-alert", "is_open"),
         State("auth-session", "data")],
    )
    def record_click_func(
        n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, record_option, is_open_alert, session_data
    ):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return is_open
        
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "record-btn-yes":
                command_thread.add_to_queue(f"record {record_option}")
                print("alert")

            if n_clicks_yes or n_clicks_no or n_clicks_btn:
                print("open")
                return not is_open
            return is_open

    @ app.callback(
        Output("recording-alert", "is_open"),
        [Input("record-btn-yes", "n_clicks"),
         Input("btn-stop-record", "n_clicks")],
        [],
    )
    def record_alert_func(n_clicks_start, n_clicks_stop):
        ctx = dash.callback_context
        if not ctx.triggered:
            return False
        else:
            if not n_clicks_start:
                return False
            if not n_clicks_stop:
                return True
            if n_clicks_start == n_clicks_stop:
                return False
            return True

    @ app.callback(
        Output("cmd-file-modal", "is_open"),
        [
            Input("btn-cmd-file", "n_clicks"),
        ],
        [State("cmd-file-modal", "is_open")],
    )
    def cmd_file_click_func(n_clicks_btn, is_open):
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            if n_clicks_btn:
                return not is_open
            return is_open

    @ app.callback(
        Output("start-modal", "is_open"),
        [
            Input("btn-start", "n_clicks"),
            Input("start-btn-yes", "n_clicks"),
            Input("start-btn-no", "n_clicks"),
        ],
        [State("start-modal", "is_open"), State("start-config-file", "value")],
    )
    def start_click_func(
        n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, config_file_name
    ):
        ctx = dash.callback_context
        if not ctx.triggered:
            return is_open
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "start-btn-yes":
                try:
                    from srt import config_loader
                    from multiprocessing import Process

                    selected_config = config_file_name or "config.yaml"
                    config_path = Path(config["CONFIG_DIR"], selected_config)
                    if not config_path.is_file():
                        raise FileNotFoundError(
                            f"Config file not found: {config_path}"
                        )
                    config_dict = config_loader.load_yaml(config_path)
                    daemon_process = Process(
                        target=run_srt_daemon,
                        args=(config["CONFIG_DIR"], config_dict),
                        name="SRT-Daemon",
                    )
                    daemon_process.start()
                except Exception as e:
                    print(str(e))
            if n_clicks_yes or n_clicks_no or n_clicks_btn:
                return not is_open
            return is_open

    @app.callback(
        Output("signal", "children"),
        [
            Input("btn-estop", "n_clicks"),
            Input("btn-stop-record", "n_clicks"),
            Input("btn-quit", "n_clicks"),
            Input("btn-calibrate", "n_clicks"),
        ],
        [State("recording-alert", "is_open"), State("auth-session", "data")]
    )
    def cmd_button_pressed_srt(
        n_clicks_estop,
        n_clicks_stop_record,
        n_clicks_shutdown,
        n_clicks_calibrate,
        is_open,
        session_data
    ):
        # Reject if not authenticated
        if not (session_data and session_data.get("authenticated")):
            return ""

        ctx = dash.callback_context
        if not ctx.triggered:
            return ""
        else:
            button_id = ctx.triggered[0]["prop_id"].split(".")[0]
            if button_id == "btn-estop":
                command_thread.add_to_queue("spa")
            elif button_id == "btn-stop-record":
                command_thread.add_to_queue("roff")
                return not is_open
            elif button_id == "btn-quit":
                command_thread.add_to_queue("quit")
            elif button_id == "btn-calibrate":
                command_thread.add_to_queue("calibrate_encoders")
