"""system_page.py

Function for Generating System Page and Creating Callback

"""

commands = '''
**Common SRT Commands:**

| Command      | Parameters | Notes | Info                                        |
|--------------|------------|-------|--------------------------------------------|
| *            | Any text   | 1     | Makes Line a Comment                       |
| stow         | None       |       | Sends the Antenna to Stow                  |
| quit         | None       | 3     | Stows and Gracefully Closes Daemon         |
| azel         | az el  |       | Points at Azimuth 'az', Elevation 'el'     |
| offset       | az el  |       | Offsets from Current Object by 'az', 'el'  |
| wait         | time     |       | Stops Execution and Waits for 'time' Secs. |
| time       | None       |       | Waits for 'time' Seconds                   |
| LST:hh:mm:ss | None       |       | Waits Until Next Time hh:mm:ss in UTC      |
| Y:D:H:M:S    | None       |       | Waits Until Year:DayOfYear:Hour:Minute:Sec |
| name       | n or b | 5     | Points Antenna at Object named 'name'      |
| spa       | None       |       | Emergency Stop                             |
| calibrate_encoders | None | 2     | Run 6m antenna encoder calibration routine    |

**Notes:**

1. Lines starting with '*' are comments.

2. 'calibrate' should be run at the cal position.

3. 'quit' stows and stops the daemon.

4. File type for 'record' is set by extension (.fits, .rad, or none for raw).

5. Object names are from sky_coords.csv (e.g. Sun, Moon).

6. Parameters are separated by spaces. Most commands are case-insensitive.
'''

import importlib

try:
    from dash import dcc
except Exception:
    dcc = importlib.import_module("dash_core_components")

try:
    from dash import html
except Exception:
    html = importlib.import_module("dash_html_components")

from dash.dependencies import Input, Output, State
from dash.exceptions import PreventUpdate

from urllib.parse import quote as urlquote
from datetime import datetime
from pathlib import Path


def generate_layout():
    """Generates the Basic Layout for the System Page

    Returns
    -------
    System Page Layout
    """
    layout = html.Div(
        [
            html.Div(
                [
                    html.Div(
                        [],
                        className="one-third column",
                    ),
                    html.Div(
                        [
                            html.H4(
                                "SRT System Page",
                                style={"margin-bottom": "0px", "text-align": "center"},
                            ),
                        ],
                        className="one-third column",
                        id="title",
                    ),
                    html.Div([], className="one-third column", id="button"),
                ],
                id="header",
                className="row flex-display",
                style={"margin-bottom": "25px"},
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.H4(
                                "Emergency Contact Info",
                                id="text-contact",
                                style={"text-align": "center"},
                            ),
                            dcc.Markdown(id="emergency-contact-info"),
                        ],
                        className="pretty_container four columns",
                    ),
                    html.Div(
                        [
                            html.H4(
                                id="text-queue-status", style={"text-align": "center"}
                            ),
                            dcc.Markdown(id="command-display"),
                        ],
                        className="pretty_container four columns",
                    ),
                    html.Div(
                        [
                            html.H4(
                                "Recordings",
                                id="text-recordings",
                                style={"text-align": "center"},
                            ),
                            html.Div(
                                id="recordings-list",
                                style={
                                    "height": 150,
                                    "overflow": "hidden",
                                    "overflow-y": "scroll",
                                },
                            ),
                        ],
                        className="pretty_container four columns",
                    ),
                ],
                className="flex-display",
                style={"justify-content": "center"},
            ),
            html.Div(
                [
                    html.Div(
                        [
                            html.H4(
                                "Message Logs",
                                style={"text-align": "center"},
                            ),
                            dcc.Checklist(
                                id="log-channel-filter",
                                options=[
                                    {"label": "Daemon", "value": "daemon"},
                                    {"label": "Serial", "value": "serial"},
                                    {"label": "Commands", "value": "commands"},
                                    {"label": "Observations", "value": "observations"},
                                ],
                                value=["daemon", "serial", "commands", "observations"],
                                inline=True,
                                style={"marginBottom": "10px"},
                            ),
                            html.Div(
                                id="message-logs",
                                style={
                                    "height": 240,
                                    "overflow": "hidden",
                                    "overflowY": "scroll",
                                    "fontFamily": "monospace",
                                    "fontSize": "12px",
                                    "lineHeight": "1.4",
                                    "whiteSpace": "pre-wrap",
                                    "wordWrap": "break-word",
                                    "padding": "8px",
                                    "backgroundColor": "#f5f5f5",
                                },
                            ),
                            html.Div(
                                [
                                    dcc.Input(
                                        id="manual-command-input",
                                        type="text",
                                        placeholder="Enter command (e.g. azel 120 30) and press Enter",
                                        style={
                                            "width": "100%",
                                            "padding": "6px",
                                            "marginTop": "8px",
                                            "boxSizing": "border-box",
                                        },
                                    ),
                                    html.Div(
                                        id="manual-command-status",
                                        style={
                                            "fontSize": "0.85em",
                                            "color": "#666",
                                            "marginTop": "4px",
                                            "minHeight": "18px",
                                        },
                                    ),
                                    html.Div(
                                        [
                                            html.Hr(),
                                            html.Details([
                                                html.Summary("Available Commands & Syntax Reference"),
                                                dcc.Markdown(commands),
                                                html.Div("See docs/command_files.md for more details.", style={"fontSize": "0.85em", "color": "#888", "marginTop": "8px"}),
                                                ], style={"marginTop": "16px", "marginBottom": "8px"}),
                                        ]
                                    ),
                                ],
                                style={"marginTop": "10px", "paddingTop": "10px", "borderTop": "1px solid #ddd"},
                            ),
                        ],
                        className="pretty_container twelve columns",
                    ),
                ],
                className="flex-display",
                style={"justify-content": "center", "margin": "5px"},
            ),

        ]
    )
    return layout


def register_callbacks(app, config, status_thread, command_thread):
    """Registers the Callbacks for the System Page

    Parameters
    ----------
    app : Dash Object
        Dash Object to Set Up Callbacks to
    config : dict
        Contains All Settings for Dashboard / Daemon
    status_thread : Thread
        Thread for Getting Status from Daemon

    Returns
    -------
    None
    """

    @app.callback(
        Output("emergency-contact-info", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_contact_info(n):
        status = status_thread.get_status()
        if status is None or "emergency_contact" not in status:
            return ""
        status = status["emergency_contact"]
        status_string = f"""
         - Name: {status["name"]}
         - Email: {status["email"]}
         - Phone Number: {status["phone_number"]}
        """
        return status_string

    @app.callback(
        Output("message-logs", "children"),
        [
            Input("interval-component", "n_intervals"),
            Input("log-channel-filter", "value"),
        ],
    )
    def update_message_logs(n, channels):
        status = status_thread.get_status()
        if status is None:
            return ""
        channels = channels or []

        # Helper function to parse timestamps from various formats
        def parse_timestamp(iso_time_str):
            """Parse ISO timestamp string, treating missing timezone as UTC."""
            if not iso_time_str:
                return 0.0
            try:
                # If string ends with Z, it's UTC
                if iso_time_str.endswith('Z'):
                    ts = datetime.fromisoformat(iso_time_str.replace("Z", "+00:00")).timestamp()
                # If it has +00:00 or other timezone, parse as-is
                elif '+' in iso_time_str or iso_time_str.count('-') > 2:
                    ts = datetime.fromisoformat(iso_time_str).timestamp()
                # If no timezone indicator, assume UTC and append it
                else:
                    ts = datetime.fromisoformat(iso_time_str + "+00:00").timestamp()
                return ts
            except (ValueError, AttributeError):
                return 0.0

        entries = []

        if "daemon" in channels:
            for log_time, log_txt in status.get("error_logs", []):
                try:
                    ts = float(log_time)
                except (ValueError, TypeError):
                    ts = 0.0
                entries.append(
                    (
                        ts,
                        "daemon",
                        f"{datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')} | {log_txt}",
                    )
                )

        if "serial" in channels:
            for entry in status.get("serial_communications", []):
                # Try to extract timestamp from multiple possible field names/formats
                iso_time = entry.get("time", "") or entry.get("iso_time", "")
                ts = parse_timestamp(iso_time)
                
                direction = str(entry.get("direction", "?")).upper()
                payload = entry.get("payload", "")
                entries.append((ts, "serial", f"{iso_time} | {direction}: {payload}"))

        if "commands" in channels:
            for entry in status.get("command_history", []):
                iso_time = entry.get("time", "") or entry.get("iso_time", "")
                ts = parse_timestamp(iso_time)
                
                cmd = entry.get("command", "")
                success = entry.get("success", False)
                total_ms = entry.get("total_ms", 0.0)
                retries = entry.get("retries", 0)
                label = "OK" if success else "ERR"
                entries.append(
                    (
                        ts,
                        "commands",
                        f"{iso_time} | {label} {cmd} (retries={retries}, total={total_ms}ms)",
                    )
                )

        if "observations" in channels:
            for entry in status.get("observation_events", []):
                iso_time = entry.get("iso_time", "") or entry.get("time", "")
                ts = parse_timestamp(iso_time)
                
                event_name = entry.get("event", "")
                meta = entry.get("metadata", {}) or {}
                obj = meta.get("object")
                target = meta.get("target_azel")
                details = ""
                if obj:
                    details = f" object={obj}"
                elif isinstance(target, (list, tuple)) and len(target) == 2:
                    details = f" az={target[0]}, el={target[1]}"
                entries.append((ts, "observations", f"{iso_time} | {event_name}{details}"))

        # Sort by timestamp, with list position as tiebreaker to maintain order for messages with same timestamp
        entries_with_idx = [(i, ts, channel, msg) for i, (ts, channel, msg) in enumerate(entries)]
        entries_with_idx.sort(key=lambda x: (x[1], x[0]), reverse=False)
        
        # Define colors for each channel
        channel_colors = {
            "daemon": "#d32f2f",  # Red
            "serial": "#1976d2",  # Blue
            "commands": "#388e3c", # Green
            "observations": "#f57c00", # Orange
        }
        
        children = [
            html.P(
                f"[{channel}] {message}",
                style={"color": channel_colors.get(channel, "#000"), "margin": "0", "paddingBottom": "2px"}
            )
            for _, ts, channel, message in entries_with_idx[:400]
        ]
        
        # Return wrapper div that will be placed in message-logs container
        return html.Div(children=children)

    @app.callback(
        Output("text-queue-status", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_command_queue_display(n):
        status = status_thread.get_status()
        if status is None or (
            status["queue_size"] == 0 and status["queued_item"] == "None"
        ):
            return "SRT Inactive"
        return "SRT in Use!"

    @app.callback(
        Output("command-display", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_command_display(n):
        status = status_thread.get_status()
        if status is None:
            return ""
        current_cmd = status["queued_item"]
        queue_size = status["queue_size"]
        status_string = f"""
        ##### Command Queue Status
         - Running Command: {current_cmd}
         - {queue_size} More Commands Waiting in the Queue
        """
        return status_string

    @app.callback(
        [
            Output("manual-command-input", "value"),
            Output("manual-command-status", "children"),
        ],
        [Input("manual-command-input", "n_submit")],
        [State("manual-command-input", "value"), State("auth-session", "data")],
        prevent_initial_call=True,
    )
    def handle_manual_command(n_submit, cmd_text, session_data):
        if not (session_data and session_data.get("authenticated")):
            return "", "Unauthorized: Please log in first"
        if not n_submit:
            raise PreventUpdate
        cmd = (cmd_text or "").strip()
        if not cmd:
            return "", "Command cannot be empty"
        command_thread.add_to_queue(cmd)
        return "", f"Queued: {cmd}"

    @app.callback(
        Output("recordings-list", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_output(n):
        """Save uploaded files and regenerate the file list."""

        files = [
            file.name
            for file in Path(config["SAVE_DIRECTORY"]).expanduser().glob("*")
            if file.is_file()
        ]
        folders = [
            file.name
            for file in Path(config["SAVE_DIRECTORY"]).expanduser().glob("*")
            if file.is_dir()
        ]
        if len(files) == 0:
            return [html.Li("No files yet!")]
        else:
            if config["DASHBOARD_DOWNLOADS"]:
                return [
                    html.Li(html.A(filename, href=f"/download/{urlquote(filename)}"))
                    for filename in files
                ] + [html.Li(html.A(foldername)) for foldername in folders]
            else:
                return [html.Li(html.A(filename)) for filename in (files + folders)]
