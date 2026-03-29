"""monitor_page.py

Function for Generating Monitor Page and Creating Callback

"""

import dash

try:
    from dash import dcc
except:
    import dash_core_components as dcc

import dash_bootstrap_components as dbc

try:
    from dash import html
except:
    import dash_html_components as html

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
                    [dcc.Graph(id="az-el-graph")],
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
        containing mount status diagnostics if srt
    """
    return html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [
                            html.H5("6m Mount Status"),
                            dcc.Markdown(id="mount-status-panel"),
                        ],
                        className="pretty_container six columns",
                        style={"maxHeight": "420px", "overflowY": "auto"},
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


# def generate_point_popup():

#     return html.Div(
#         [dbc.Modal(
#             [
#                 dbc.ModalHeader("Point at Object"),
#                 dbc.ModalBody(
#                     [
#                         html.H5("Confirm Pointing to This Object?"),
#                         dcc.RadioItems(
#                             options=[
#                                 {"label": "Direct Point", "value": ""},
#                                 {"label": "N-Point Scan", "value": " n"},
#                                 {"label": "Beam-switch", "value": " b"},
#                             ],
#                             id="point-options",
#                             value="",
#                         ),
#                     ]
#                 ),
#                 dbc.ModalFooter(
#                     [
#                         dbc.Button(
#                             "Yes",
#                             id="az-el-graph-btn-yes",
#                             className="ml-auto",
#                             # block=True,
#                             color="primary",
#                         ),
#                         dbc.Button(
#                             "No",
#                             id="az-el-graph-btn-no",
#                             className="ml-auto",
#                             # block=True,
#                             color="secondary",
#                         ),
#                     ]
#                 ),
#             ],
#             id="az-el-graph-modal",

#         ),]
#     )


def generate_popups(software):
    """Generates all 'Pop-up' Modal Components

    Returns
    -------
    Div Containing all Modal Components
    """
    all_objects = get_all_objects()

    if software == "Very Small Radio Telescope":
        vsrt_open = True
        srt_open = False
    else:
        vsrt_open = False
        srt_open = True

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
            # is_open=srt_open,
        ),

            dbc.Modal(
            [
                dbc.ModalHeader("Point at Object"),
                dbc.ModalBody(
                    [
                        html.H5("Confirm Pointing to This Object?"),
                    ]
                ),
                dbc.ModalFooter(
                    [
                        dbc.Button(
                            "Yes",
                            id="az-el-vsrt-btn-yes",
                            className="ml-auto",
                            # block=True,
                            color="primary",
                        ),
                        dbc.Button(
                            "No",
                            id="az-el-vsrt-btn-no",
                            className="ml-auto",
                            # block=True,
                            color="secondary",
                        ),
                    ]
                ),
            ],
            id="az-el-vsrt-modal",
            # is_open=vsrt_open,
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


def generate_layout(software, config_dict=None):
    """Generates the Basic Layout for the Monitor Page

    Parameters
    ----------
    software : str
        Name of the software/system
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
    
    drop_down_buttons_vsrt = {
        "Coordinates": [
            dbc.DropdownMenuItem("Set Location", id="btn-set-coords"),
        ],
        "Observe": [
            dbc.DropdownMenuItem("Select Object", id="btn-obs-obj"),
            dbc.DropdownMenuItem("Enter Coordinates", id="btn-obs-coords"),
        ],
        "Radio": [
            dbc.DropdownMenuItem("Set Frequency", id="btn-set-freq"),
            dbc.DropdownMenuItem("Set Bandwidth", id="btn-set-samp"),
        ],
        "Routine": [
            dbc.DropdownMenuItem("Start Recording", id="btn-start-record"),
            dbc.DropdownMenuItem("Stop Recording", id="btn-stop-record"),
            dbc.DropdownMenuItem("Calibrate Encoders", id="btn-calibrate"),
            dbc.DropdownMenuItem("Export Obs CSV", id="btn-export-obs"),
            dbc.DropdownMenuItem("Upload CMD File", id="btn-cmd-file"),
        ],
        "Power": [
            dbc.DropdownMenuItem("Start Daemon", id="btn-start"),
            dbc.DropdownMenuItem("Shutdown", id="btn-quit"),
        ],
    }
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
            dbc.DropdownMenuItem("Export Obs CSV", id="btn-export-obs"),
            dbc.DropdownMenuItem("Upload CMD File", id="btn-cmd-file"),
        ],
        "Power": [
            dbc.DropdownMenuItem("Start Daemon", id="btn-start"),
            dbc.DropdownMenuItem("Shutdown", id="btn-quit"),
        ],
    }
    
    # Build layout content, conditionally including radio plots
    base_vsrt = [
        generate_navbar(drop_down_buttons_vsrt),
        dbc.Alert("Recording", color="danger",
                  id="recording-alert", is_open=False),
    ]
    if radio_enabled:
        base_vsrt.append(generate_first_row())
    base_vsrt.extend([
        generate_second_row(),
        generate_third_row(),
        *( [generate_webcam_row()] if webcam_enabled else [] ),
        generate_popups(software),
        html.Div(id="signal", style={"display": "none"}),
        dcc.Download(id="obs-events-download"),
    ])
    
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
        generate_popups(software),
        html.Div(id="signal", style={"display": "none"}),
        dcc.Download(id="obs-events-download"),
    ])
    
    if software == "Very Small Radio Telescope":
        layout = html.Div(base_vsrt)
    else:
        layout = html.Div(base_srt)
    
    return layout


def register_callbacks(
    app, config, status_thread, command_thread, raw_spectrum_thread, cal_spectrum_thread, software
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
    # config_path = Path(config["CONFIG_DIR"], config_file)
    # config_dict = config_loader.load_yaml(config_path)

    if software == "Very Small Radio Telescope":
        vsrt = True
    else:
        vsrt = False

    control_profile = str(config.get("CONTROL_PROFILE", "FULL_SRT")).upper()
    radio_enabled = control_profile != "POINTING_ONLY"

    if radio_enabled:
        @app.callback(
            Output("cal-spectrum-histogram", "figure"),
            [Input("interval-component", "n_intervals")],
        )
        def update_cal_spectrum_histogram(n):
            spectrum = cal_spectrum_thread.get_spectrum()
            status = status_thread.get_status()
            if status is None or spectrum is None:
                return ""
            bandwidth = float(status["bandwidth"])
            cf = float(status["center_frequency"])
            return generate_spectrum_graph(bandwidth, cf, spectrum, is_spec_cal=True)

        @app.callback(
            Output("raw-spectrum-histogram", "figure"),
            [Input("interval-component", "n_intervals")],
        )
        def update_raw_spectrum_histogram(n):

            spectrum = raw_spectrum_thread.get_spectrum()
            status = status_thread.get_status()
            if status is None or spectrum is None:
                return ""
            bandwidth = float(status["bandwidth"])
            cf = float(status["center_frequency"])
            return generate_spectrum_graph(bandwidth, cf, spectrum, is_spec_cal=False)

        @app.callback(
            Output("power-graph",
                   "figure"), [Input("interval-component", "n_intervals")]
        )
        def update_power_graph(n):
            status = status_thread.get_status()
            if status is None:
                return ""
            tsys = float(status["temp_sys"])
            tcal = float(status["temp_cal"])
            cal_pwr = float(status["cal_power"])
            spectrum_history = raw_spectrum_thread.get_history()
            if spectrum_history is None:
                return ""
            return generate_power_history_graph(tsys, tcal, cal_pwr, spectrum_history)

    if not vsrt:
        @app.callback(
            Output("mount-status-panel", "children"),
            [Input("interval-component", "n_intervals")],
        )
        def update_mount_status_panel(n):
            """Render a real-time diagnostic panel for Caltech 6m status and serial comms."""
            status = status_thread.get_status()
            if status is None:
                return "#### 6m Mount\n- Waiting for daemon status"

            diagnostics = status.get("rotor_diagnostics", {}) or {}
            fsm = status.get("rotor_fsm_status", {}) or diagnostics.get("fsm_status", {})
            lines = ["#### 6m Mount"]

            mode = diagnostics.get("mode")
            cal_sts = diagnostics.get("CalSts")
            az_brk = diagnostics.get("AzBrkOn")
            el_brk = diagnostics.get("ElBrkOn")
            estop = diagnostics.get("EmStopOn")

            if mode is not None:
                lines.append(f"- Mode: {mode}")
            if cal_sts is not None:
                lines.append(f"- Calibration: {cal_sts}")
            if az_brk is not None:
                lines.append(f"- Az Brake: {'ON' if az_brk else 'OFF'}")
            if el_brk is not None:
                lines.append(f"- El Brake: {'ON' if el_brk else 'OFF'}")
            if estop is not None:
                lines.append(f"- E-Stop: {'ON' if estop else 'OFF'}")

            if isinstance(fsm, dict) and fsm:
                lines.append("\n#### FSM")
                state = fsm.get("state")
                transition = fsm.get("last_transition")
                retries = fsm.get("retry_count")
                last_error = fsm.get("last_error")
                if state is not None:
                    lines.append(f"- State: {state}")
                if transition:
                    lines.append(f"- Last Transition: {transition}")
                if retries is not None:
                    lines.append(f"- Last Command Retries: {retries}")
                if last_error:
                    lines.append(f"- Last Error: {last_error}")

            point_err = status.get("pointing_error")
            if point_err is not None and len(point_err) == 2:
                lines.append(
                    f"- Pointing Err (mdeg): Az {point_err[0]:.1f}, El {point_err[1]:.1f}"
                )

            lines.append(
                f"- Motor Az, El (deg): {status['motor_azel'][0]:.3f}, {status['motor_azel'][1]:.3f}"
            )
            lines.append(
                f"- Cmd Az, El (deg): {status['motor_cmd_azel'][0]:.3f}, {status['motor_cmd_azel'][1]:.3f}"
            )

            amp = diagnostics.get("amp_currents")
            if isinstance(amp, dict) and amp:
                lines.append("\n#### Amplifier Currents")
                for axis in ["2A01", "2A02", "2A03"]:
                    vals = amp.get(axis)
                    if not isinstance(vals, dict):
                        continue
                    cmd = vals.get("commanded", "?")
                    act = vals.get("actual", "?")
                    lines.append(f"- {axis}: cmd {cmd}, act {act}")

            comms = status.get("serial_communications", [])
            cmd_hist = status.get("command_history", [])
            obs_events = status.get("observation_events", [])
            lines.append("\n#### Serial")
            if not comms:
                lines.append("- No serial messages yet")
            else:
                for entry in reversed(comms[-10:]):
                    direction = str(entry.get("direction", "sent")).upper()
                    payload = entry.get("payload", "")
                    prefix = "RX" if direction == "RECV" else "TX"
                    lines.append(f"- {prefix}: {payload}")

            lines.append("\n#### Commands")
            if not cmd_hist:
                lines.append("- No command history yet")
            else:
                for entry in reversed(cmd_hist[-8:]):
                    command = entry.get("command", "")
                    success = entry.get("success", False)
                    retries = entry.get("retries", 0)
                    total_ms = entry.get("total_ms", 0.0)
                    queue_wait_ms = entry.get("queue_wait_ms", 0.0)
                    status_label = "OK" if success else "ERR"
                    lines.append(
                        f"- {status_label}: {command} | retries={retries} | total={total_ms}ms | queue={queue_wait_ms}ms"
                    )
                    if not success and entry.get("error"):
                        lines.append(f"  error: {entry.get('error')}")

            lines.append("\n#### Observation Events")
            if not obs_events:
                lines.append("- No observation events yet")
            else:
                for event in reversed(obs_events[-8:]):
                    iso_time = event.get("iso_time", "")
                    event_name = event.get("event", "")
                    meta = event.get("metadata", {})
                    target = meta.get("target_azel")
                    object_name = meta.get("object")
                    if target and len(target) == 2:
                        lines.append(
                            f"- {iso_time} {event_name}: az={float(target[0]):.3f}, el={float(target[1]):.3f}"
                        )
                    elif object_name:
                        lines.append(f"- {iso_time} {event_name}: object={object_name}")
                    else:
                        lines.append(f"- {iso_time} {event_name}")

            return "\n".join(lines)

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
        Output("az-el-graph",
               "figure"), [Input("interval-component", "n_intervals")]
    )
    def update_az_el_graph(n):
        status = status_thread.get_status()
        if status is not None:
            return generate_az_el_graph(
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
                if not vsrt:
                    command_thread.add_to_queue(
                        f"{clickData['points'][0]['text']}{mode}")
                elif vsrt:
                    command_thread.add_to_queue(
                        f"object {clickData['points'][0]['text']}")
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

    # @ app.callback(
    #     Output("az-el-vsrt-modal", "is_open"),
    #     [
    #         Input("az-el-graph", "clickData"),
    #         Input("az-el-vsrt-btn-yes", "n_clicks"),
    #         Input("az-el-vsrt-btn-no", "n_clicks"),
    #     ],
    #     [State("az-el-vsrt-modal", "is_open"),
    #      State("point-options", "value"),],
    # )
    # def az_el_click_vsrt_func(clickData, n_clicks_yes, n_clicks_no, is_open, mode):

    #     ctx = dash.callback_context
    #     if not ctx.triggered:
    #         return is_open
    #     else:
    #         button_id = ctx.triggered[0]["prop_id"].split(".")[0]
    #         if button_id == "az-el-graph-btn-yes":
    #             command_thread.add_to_queue(f"object {object}")
    #         if (
    #             n_clicks_yes
    #             or n_clicks_no
    #             or (
    #                 clickData
    #                 and not clickData["points"][0]["text"] == "Antenna Location"
    #             )
    #         ):
    #             return not is_open
    #         return is_open

    if vsrt:
        @ app.callback(
            Output("obs-obj-modal", "is_open"),
            [
                Input("btn-obs-obj", "n_clicks"),
                Input("obs-obj-btn-yes", "n_clicks"),
                Input("obs-obj-btn-no", "n_clicks"),
            ],
            [State("obs-obj-modal", "is_open"),
             State("obj-dropdown", "value"),
             State("auth-session", "data")
             ],
        )
        def set_obs_obj_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, object, session_data):
            # Reject if not authenticated
            if not (session_data and session_data.get("authenticated")):
                return is_open

            ctx = dash.callback_context
            if not ctx.triggered:
                return is_open
            else:
                button_id = ctx.triggered[0]["prop_id"].split(".")[0]
                if button_id == "obs-obj-btn-yes":
                    command_thread.add_to_queue(f"object {object}")
                if n_clicks_yes or n_clicks_no or n_clicks_btn:
                    return not is_open
                return is_open

        @ app.callback(
            Output("obs-coords-modal", "is_open"),
            [
                Input("btn-obs-coords", "n_clicks"),
                Input("obs-coords-btn-yes", "n_clicks"),
                Input("obs-coords-btn-no", "n_clicks"),
            ],
            [State("obs-coords-modal", "is_open"),
             State("obj-az", "value"),
             State("obj-el", "value"),
             State("auth-session", "data")
             ],
        )
        def set_obs_coords_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, az, el, session_data):
            # Reject if not authenticated
            if not (session_data and session_data.get("authenticated")):
                return is_open

            ctx = dash.callback_context
            if not ctx.triggered:
                return is_open
            else:
                button_id = ctx.triggered[0]["prop_id"].split(".")[0]
                if button_id == "obs-coords-btn-yes":
                    command_thread.add_to_queue(f"obj_coords {az} {el}")
                if n_clicks_yes or n_clicks_no or n_clicks_btn:
                    return not is_open
                return is_open

        @ app.callback(
            Output("coords-modal", "is_open"),
            [
                Input("btn-set-coords", "n_clicks"),
                Input("coords-btn-yes", "n_clicks"),
                Input("coords-btn-no", "n_clicks"),
            ],
            [State("coords-modal", "is_open"),
             State("coords-lat", "value"),
             State("coords-long", "value"),
             State("auth-session", "data")],
        )
        def coords_click_func(n_clicks_btn, n_clicks_yes, n_clicks_no, is_open, lat, long, session_data):
            # Reject if not authenticated
            if not (session_data and session_data.get("authenticated")):
                return is_open

            ctx = dash.callback_context
            if not ctx.triggered:
                return is_open
            else:
                button_id = ctx.triggered[0]["prop_id"].split(".")[0]
                if button_id == "coords-btn-yes":
                    command_thread.add_to_queue(f"coords {lat} {long}")
                if n_clicks_yes or n_clicks_no or n_clicks_btn:
                    return not is_open
                return is_open

    @ app.callback(
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

    @ app.callback(
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

    if vsrt:
        @ app.callback(
            Output("signal", "children"),
            [
                Input("btn-stop-record", "n_clicks"),
                Input("btn-quit", "n_clicks"),
                Input("btn-calibrate", "n_clicks"),
            ],
            [State("recording-alert", "is_open"), State("auth-session", "data")]
        )
        def cmd_button_pressed_vsrt(
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
                if button_id == "btn-stop-record":
                    command_thread.add_to_queue("roff")
                    return not is_open
                elif button_id == "btn-quit":
                    command_thread.add_to_queue("quit")
                elif button_id == "btn-calibrate":
                    command_thread.add_to_queue("calibrate_encoders")
    else:
        @ app.callback(
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
