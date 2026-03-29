"""app.py

Dash Small Radio Telescope Web App Dashboard

"""

import dash

try:
    from dash import dcc
except:
    import dash_core_components as dcc

try:
    from dash import html
except:
    import dash_html_components as html

import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State, ClientsideFunction

import flask
import plotly.io as pio
import numpy as np
from time import time
from pathlib import Path
import base64
import math
import platform
from threading import Event, Lock, Thread
from typing import Any, cast

from .layouts import monitor_page, system_page  # , figure_page
from .layouts.sidebar import generate_sidebar
from .messaging.status_fetcher import StatusThread
from .messaging.command_dispatcher import CommandThread
from .messaging.spectrum_fetcher import SpectrumThread

try:
    import cv2
except Exception:
    cv2 = None


def generate_app(config_dir, config_dict):
    """Generates App and Server Objects for Hosting Dashboard

    Parameters
    ----------
    config_dir : str
        Path to the Configuration Directory
    config_dict : dict
        Configuration Directory (Output of YAML Parser)

    Returns
    -------
    (server, app)
    """
    config_dict["CONFIG_DIR"] = config_dir
    software = config_dict["SOFTWARE"]

    # Set Up Dash app/server objects
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP],
        meta_tags=[
            {"name": "viewport", "content": "width=device-width, initial-scale=1"}
        ],
    )
    server = cast(Any, app.server)
    app.title = software

    webcam_enabled = bool(config_dict.get("WEBCAM_ENABLE", False))
    webcam_device = int(config_dict.get("WEBCAM_DEVICE_INDEX", 0))
    webcam_fps = max(1.0, float(config_dict.get("WEBCAM_TARGET_FPS", 10.0)))
    webcam_jpeg_quality = int(config_dict.get("WEBCAM_JPEG_QUALITY", 70))
    webcam_jpeg_quality = max(10, min(95, webcam_jpeg_quality))
    webcam_width = int(config_dict.get("WEBCAM_MAX_WIDTH", 960))
    webcam_width = max(160, webcam_width)

    webcam_state = {
        "latest_frame": None,
        "error": "",
    }
    webcam_lock = Lock()
    webcam_stop_event = Event()
    webcam_thread = None

    def _open_camera(device_index):
        if cv2 is None:
            return None
        if platform.system().lower().startswith("win"):
            capture = cv2.VideoCapture(device_index, cv2.CAP_DSHOW)
            if capture is not None and capture.isOpened():
                return capture
        capture = cv2.VideoCapture(device_index)
        if capture is not None and capture.isOpened():
            return capture
        return None

    def _webcam_worker():
        if cv2 is None:
            with webcam_lock:
                webcam_state["error"] = "OpenCV not installed (pip install opencv-python)."
            return

        capture = _open_camera(webcam_device)
        if capture is None:
            with webcam_lock:
                webcam_state["error"] = f"Unable to open webcam device index {webcam_device}."
            return

        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        frame_interval = 1.0 / webcam_fps
        last_emit = 0.0

        try:
            while not webcam_stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    with webcam_lock:
                        webcam_state["error"] = "Webcam frame read failed."
                    continue

                now_ts = time()
                if now_ts - last_emit < frame_interval:
                    continue
                last_emit = now_ts

                h, w = frame.shape[:2]
                if w > webcam_width and webcam_width > 0:
                    new_h = int((webcam_width / float(w)) * float(h))
                    frame = cv2.resize(frame, (webcam_width, max(1, new_h)))

                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), webcam_jpeg_quality],
                )
                if not ok:
                    with webcam_lock:
                        webcam_state["error"] = "Failed to JPEG-encode webcam frame."
                    continue

                with webcam_lock:
                    webcam_state["latest_frame"] = encoded.tobytes()
                    webcam_state["error"] = ""
        finally:
            capture.release()

    if webcam_enabled:
        webcam_thread = Thread(target=_webcam_worker, name="dashboard-webcam", daemon=True)
        webcam_thread.start()

        @server.route("/video_feed")
        def video_feed():
            def generate_stream():
                while True:
                    with webcam_lock:
                        frame = webcam_state.get("latest_frame")
                        err = webcam_state.get("error", "")

                    if frame is None:
                        message = err or "Waiting for webcam frames..."
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: text/plain\r\n\r\n"
                            + message.encode("utf-8", errors="replace")
                            + b"\r\n"
                        )
                    else:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + frame
                            + b"\r\n"
                        )

            return flask.Response(
                generate_stream(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @server.route("/video_status")
        def video_status():
            with webcam_lock:
                err = webcam_state.get("error", "")
                has_frame = webcam_state.get("latest_frame") is not None
            return {"enabled": True, "has_frame": has_frame, "error": err}

    # Start Listening for Radio and Status Data
    status_thread = StatusThread(port=5555)
    status_thread.start()

    command_thread = CommandThread(port=5556)
    command_thread.start()

    raw_spectrum_thread = SpectrumThread(port=5561)
    raw_spectrum_thread.start()

    cal_spectrum_thread = SpectrumThread(port=5563)
    cal_spectrum_thread.start()

    # Dictionary of Pages and matching URL prefixes
    pages = {
        "Monitor Page": "monitor-page",
        "System Page": "system-page",
        #    "Figure Page": "figure-page"
    }
    if "DASHBOARD_REFRESH_MS" in config_dict.keys():
        refresh_time = config_dict["DASHBOARD_REFRESH_MS"]  # ms
    else:
        refresh_time = 1000
    pio.templates.default = "seaborn"  # Style Choice for Graphs
    curfold = Path(__file__).parent.absolute()
    # Generate Sidebar Objects
    side_title = software
    image_filename = curfold.joinpath(
        "images", "6m_logo.jpg"
    )  # replace with your own image
    # Check if file is there and if not put in a single pixel image.
    if image_filename.exists():
        encoded_image = base64.b64encode(open(image_filename, "rb").read())
    else:
        encoded_image = b"iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII="

    side_content = {
        "Status": dcc.Markdown(id="sidebar-status"),
        "Pages": html.Div(
            [
                html.H4("Pages"),
                dbc.Nav(
                    [
                        dbc.NavLink(
                            page_name,
                            href=f"/{pages[page_name]}",
                            id=f"{pages[page_name]}-link",
                        )
                        for page_name in pages
                    ],
                    vertical=True,
                    pills=True,
                ),
            ]
        ),
        "Image": html.Div(
            [
                html.Img(
                    src="data:image/png;base64,{}".format(
                        encoded_image.decode()
                    ),
                    style={"height": "100%", "width": "100%"},
                ),
            ]
        ),
    }
    sidebar = generate_sidebar(side_title, side_content)

    # Build Dashboard Framework
    auth_required = bool(config_dict.get("DASHBOARD_REQUIRE_AUTH", False))
    auth_username = str(config_dict.get("DASHBOARD_USERNAME", "admin"))
    auth_password = str(config_dict.get("DASHBOARD_PASSWORD", "admin"))
    
    content = html.Div(id="page-content")
    
    # Login modal for authentication
    login_modal = dbc.Modal(
        [
            dbc.ModalHeader("Dashboard Login", close_button=False),
            dbc.ModalBody(
                [
                    html.Div(
                        id="login-error-alert",
                        children=[],
                        style={"marginBottom": "15px"}
                    ),
                    dbc.InputGroup(
                        [
                            dbc.InputGroupText("Username"),
                            dbc.Input(
                                id="login-username",
                                placeholder="Username",
                                type="text",
                                autoComplete="username",
                            ),
                        ],
                        className="mb-3",
                    ),
                    dbc.InputGroup(
                        [
                            dbc.InputGroupText("Password"),
                            dbc.Input(
                                id="login-password",
                                placeholder="Password",
                                type="password",
                                autoComplete="current-password",
                            ),
                        ],
                        className="mb-3",
                    ),
                ]
            ),
            dbc.ModalFooter(
                dbc.Button(
                    "Login",
                    id="login-submit-btn",
                    className="ms-auto",
                    color="primary",
                    n_clicks=0,
                )
            ),
        ],
        id="login-modal",
        backdrop="static",
        keyboard=False,
        is_open=auth_required,
        centered=True,
    )
    
    layout = html.Div(
        [
            dcc.Location(id="url"),
            dcc.Store(id="auth-session", storage_type="session"),
            login_modal,
            html.Div(
                [
                    sidebar,
                    content,
                    dcc.Interval(id="interval-component",
                                 interval=refresh_time, n_intervals=0),
                    html.Div(id="output-clientside"),
                ],
                id="mainContainer",
                style={
                    "height": "100vh",
                    "min_height": "100vh",
                    "width": "100%",
                    "display": "inline-block" if not auth_required else "none",
                },
            ),
        ]
    )

    app.layout = layout  # Set App Layout to Dashboard Framework
    app.validation_layout = html.Div(
        [
            layout,
            monitor_page.generate_layout(config_dict["SOFTWARE"], config_dict),
            system_page.generate_layout(),
            #    figure_page.generate_layout()
        ]
    )  # Necessary for Allowing Other Files to Create Callbacks

    # Authentication callbacks
    if auth_required:
        @app.callback(
            [
                Output("auth-session", "data"),
                Output("login-modal", "is_open"),
                Output("login-error-alert", "children"),
            ],
            [
                Input("login-submit-btn", "n_clicks"),
                Input("auth-session", "data"),
            ],
            [
                State("login-username", "value"),
                State("login-password", "value"),
            ],
            prevent_initial_call=False,
        )
        def handle_login(n_clicks, session_data, username, password):
            # If already authenticated, keep modal closed
            if session_data and session_data.get("authenticated"):
                return session_data, False, []
            
            # If no submit yet, show modal
            if not n_clicks or n_clicks == 0:
                return {}, True, []
            
            # Check credentials
            if username == auth_username and password == auth_password:
                return {"authenticated": True}, False, []
            else:
                error_alert = dbc.Alert(
                    "Invalid username or password",
                    color="danger",
                    dismissable=False,
                )
                return {}, True, [error_alert]
        
        @app.callback(
            Output("mainContainer", "style"),
            [Input("auth-session", "data")],
        )
        def toggle_main_container_visibility(session_data):
            style = {
                "height": "100vh",
                "min_height": "100vh",
                "width": "100%",
                "display": "inline-block" if (session_data and session_data.get("authenticated")) else "none",
            }
            return style

    # Create Resizing JS Script Callback
    app.clientside_callback(
        ClientsideFunction(namespace="clientside", function_name="resize"),
        Output("output-clientside", "children"),
        [Input("page-content", "children")],
    )
    # Create Callbacks for Monitoring Page Objects
    monitor_page.register_callbacks(
        app,
        config_dict,
        status_thread,
        command_thread,
        raw_spectrum_thread,
        cal_spectrum_thread,
        software
    )
    # Create Callbacks for System Page Objects
    system_page.register_callbacks(app, config_dict, status_thread)

    # # Create Callbacks for figure page callbacks
    # figure_page.register_callbacks(app,config_dict, status_thread)
    # Activates Downloadable Saves - Caution
    if config_dict["DASHBOARD_DOWNLOADS"]:

        @server.route("/download/<path:path>")
        def download(path):
            """Serve a file from the upload directory."""
            return flask.send_from_directory(
                Path(config_dict["SAVE_DIRECTORY"]).expanduser(),
                path,
                as_attachment=True,
            )

    @app.callback(
        [Output(f"{pages[page_name]}-link", "active") for page_name in pages],
        [Input("url", "pathname")],
    )
    def toggle_active_links(pathname):
        """Sets the Page Links to Highlight to Current Page

        Parameters
        ----------
        pathname : str
            Current Page Pathname

        Returns
        -------
        list
            Sparse Bool List Which is True Only on the Proper Page Link
        """
        if pathname == "/":
            # Treat page 1 as the homepage / index
            return tuple([i == 0 for i, _ in enumerate(pages)])
        return [pathname == f"/{pages[page_name]}" for page_name in pages]

    @app.callback(
        Output("sidebar", "className"),
        [Input("sidebar-toggle", "n_clicks")],
        [State("sidebar", "className")],
    )
    def toggle_classname(n, classname):
        """Changes Sidebar's className When it is Collapsed

        Notes
        -----
        As per the Dash example this is based on, changing the sidebar's className
        changes the CSS that applying to it, allowing for hiding the sidebar

        Parameters
        ----------
        n
            Num Clicks on Button
        classname : str
            Current Classname

        Returns
        -------

        """
        if n and classname == "":
            return "collapsed"
        return ""

    @app.callback(
        Output("sidebar-status", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_status_display(n):
        """Updates the Status Part of the Sidebar

        Parameters
        ----------
        n : int
            Number of Intervals that Have Occurred (Unused)

        Returns
        -------
        str
            Content for the Sidebar, Formatted as Markdown
        """
        status = status_thread.get_status()
        if status is None:
            return """
            #### SRT Not Connected
            - Waiting for daemon status stream
            """
        else:
            az = status["motor_azel"][0]
            el = status["motor_azel"][1]
            az_offset = status["motor_offsets"][0]
            el_offset = status["motor_offsets"][1]
            time_dif = time() - status["time"]
            if time_dif > 5:
                status_string = "SRT Daemon Not Available"
            else:
                status_string = "SRT Active"

        lines = [
            f"#### {status_string}",
            f"- Motor Az, El: {az:.1f}, {el:.1f} deg",
        ]

        point_err = status.get("pointing_error")
        if point_err is not None and len(point_err) == 2:
            lines.append(
                f"- Pointing Err Az, El: {point_err[0]:.1f}, {point_err[1]:.1f} mdeg"
            )

        if config_dict["SOFTWARE"] != "Very Small Radio Telescope":
            lines.append(f"- Motor Offsets: {az_offset:.1f}, {el_offset:.1f} deg")

        return "\n".join(lines)

    @app.callback(Output("page-content", "children"), [Input("url", "pathname")])
    def render_page_content(pathname):
        """Renders the Correct Content of the Page Portion

        Parameters
        ----------
        pathname : str
            URL Path Requested

        Returns
        -------
        Content of page-content
        """

        if pathname in ["/", f"/{pages['Monitor Page']}"]:
            return monitor_page.generate_layout(config_dict["SOFTWARE"], config_dict)
        elif pathname == f"/{pages['System Page']}":
            return system_page.generate_layout()
        # elif pathname == f"/{pages['Figure Page']}":
        #     return figure_page.generate_layout()
        # If the user tries to reach a different page, return a 404 message
        return dbc.Jumbotron(
            [
                html.H1("404: Not found", className="text-danger"),
                html.Hr(),
                html.P(f"The pathname {pathname} was not recognised..."),
            ]
        )

    return server, app
