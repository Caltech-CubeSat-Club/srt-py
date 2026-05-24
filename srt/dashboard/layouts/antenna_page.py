"""antenna_page.py

Antenna diagnostics page for the SRT dashboard.

Displays:
  - Rolling time-series plots of amplifier commanded and actual currents
    (one trace per amp per channel, updated on the dashboard interval).
  - Read-only display of the LPR loop parameters loaded into the servo
    controller, annotated with their physical meaning and the parameter
    position in the LPR command string.

LPR parameter order (from cmdLPR source, positions 1-indexed):
  1  pAzKo       Az position loop outer P gain
  2  pElKo       El position loop outer P gain
  3  pAzKv       Az velocity observer gain
  4  pElKv       El velocity observer gain
  5  pAze        Az position error deadband (deg)
  6  pEle        El position error deadband (deg)
  7  pAzAmax     Az acceleration limit (antenna deg/s²)
  8  pElAmax     El acceleration limit (antenna deg/s²)
  9  pAzVmax     Az velocity ceiling (antenna deg/s)  ← raise to allow faster slews
  10 pElVmax     El velocity ceiling (antenna deg/s)
  11 pAzImax     Az position integrator anti-windup clamp
  12 pElImax     El position integrator anti-windup clamp
  13 pAzKpp      Az position loop P gain
  14 pElKpp      El position loop P gain
  15 pAzKpi      Az position loop I gain
  16 pElKpi      El position loop I gain
  17 pAzKvp      Az velocity loop P gain  ← reduce to fix overshoot
  18 pElKvp      El velocity loop P gain
  19 pAzKvi      Az velocity loop I gain
  20 pElKvi      El velocity loop I gain
  21 pAzKff      Az feedforward gain
  22 pElKff      El feedforward gain
  23 pAzTmax     Az torque upper limit (DAC counts, max 2047)
  24 pElTmax     El torque upper limit
  25 pAzTmin     Az torque lower limit (negative)
  26 pElTmin     El torque lower limit (negative)
  27 pAzTsgn     Az torque sign correction
  28 pElTsgn     El torque sign correction
  29 pAzTbias    Az torque bias (Y/Z amp differential, DAC counts)
  30 pAzEcr      Az antenna encoder counts/rev
  31 pElEcr      El antenna encoder counts/rev
  32 pAzMEcr     Az motor encoder counts/rev
  33 pElMEcr     El motor encoder counts/rev
  34 pAzEpo      Az encoder phase offset (deg) — must be correct for calibration
  35 pElEpo      El encoder phase offset (deg) — must be correct for calibration
  36 pAzAr       Az axis ratio (motor deg/s per antenna deg/s)
  37 pElAr       El axis ratio

The page expects the status dict to contain a "lpr_params" key holding either:
  - A list of 37 floats in LPR command order, OR
  - A dict keyed by parameter name.
If absent the boxes show "—".

Similarly, amp current history is expected under "amp_current_history":
  - A list of dicts: [{"time": float(unix), "2A01": {"commanded": int, "actual": int}, ...}, ...]
If absent the plots show empty.
"""

from collections import deque
from time import time

try:
    from dash import dcc, html
except ImportError:
    import dash_core_components as dcc
    import dash_html_components as html

import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output
import plotly.graph_objects as go

from .graphs import generate_pointing_error_graph
from .navbar import generate_navbar


# ---------------------------------------------------------------------------
# LPR parameter metadata
# LPR_PARAMS is ordered by LPR command position (0-indexed = position-1).
# ---------------------------------------------------------------------------

LPR_PARAMS = [
    # (key, display_name, unit/description)
    ("pAzKo",   "pAzKo",   "Az outer position P gain"),
    ("pElKo",   "pElKo",   "El outer position P gain"),
    ("pAzKv",   "pAzKv",   "Az velocity observer gain"),
    ("pElKv",   "pElKv",   "El velocity observer gain"),
    ("pAze",    "pAze",    "Az position error deadband (deg)"),
    ("pEle",    "pEle",    "El position error deadband (deg)"),
    ("pAzAmax", "pAzAmax", "Az acceleration limit (antenna °/s²)"),
    ("pElAmax", "pElAmax", "El acceleration limit (antenna °/s²)"),
    ("pAzVmax", "pAzVmax", "Az velocity ceiling (antenna °/s) ▲ raise for faster slews"),
    ("pElVmax", "pElVmax", "El velocity ceiling (antenna °/s)"),
    ("pAzImax", "pAzImax", "Az pos-integrator anti-windup clamp"),
    ("pElImax", "pElImax", "El pos-integrator anti-windup clamp"),
    ("pAzKpp",  "pAzKpp",  "Az position loop P gain"),
    ("pElKpp",  "pElKpp",  "El position loop P gain"),
    ("pAzKpi",  "pAzKpi",  "Az position loop I gain"),
    ("pElKpi",  "pElKpi",  "El position loop I gain"),
    ("pAzKvp",  "pAzKvp",  "Az velocity loop P gain ▼ reduce to fix overshoot"),
    ("pElKvp",  "pElKvp",  "El velocity loop P gain"),
    ("pAzKvi",  "pAzKvi",  "Az velocity loop I gain"),
    ("pElKvi",  "pElKvi",  "El velocity loop I gain"),
    ("pAzKff",  "pAzKff",  "Az feedforward gain"),
    ("pElKff",  "pElKff",  "El feedforward gain"),
    ("pAzTmax", "pAzTmax", "Az torque upper limit (DAC counts, max ≈2047)"),
    ("pElTmax", "pElTmax", "El torque upper limit (DAC counts)"),
    ("pAzTmin", "pAzTmin", "Az torque lower limit (negative DAC counts)"),
    ("pElTmin", "pElTmin", "El torque lower limit (negative DAC counts)"),
    ("pAzTsgn", "pAzTsgn", "Az torque sign correction"),
    ("pElTsgn", "pElTsgn", "El torque sign correction"),
    ("pAzTbias","pAzTbias","Az torque bias Y/Z differential (DAC counts)"),
    ("pAzEcr",  "pAzEcr",  "Az antenna encoder counts/rev"),
    ("pElEcr",  "pElEcr",  "El antenna encoder counts/rev"),
    ("pAzMEcr", "pAzMEcr", "Az motor encoder counts/rev"),
    ("pElMEcr", "pElMEcr", "El motor encoder counts/rev"),
    ("pAzEpo",  "pAzEpo",  "Az encoder phase offset (deg) — critical for calibration"),
    ("pElEpo",  "pElEpo",  "El encoder phase offset (deg) — critical for calibration"),
    ("pAzAr",   "pAzAr",   "Az axis ratio (motor °/s per antenna °/s)"),
    ("pElAr",   "pElAr",   "El axis ratio (motor °/s per antenna °/s)"),
]

AMP_IDS = ("2A01", "2A02")
AMP_LABELS = {
    "2A01": "2A01 (Az)",
    "2A02": "2A02 (El)",
    # "2A03": "2A03 (Az-Z)",
}

AMP_HISTORY_WINDOW_SEC = 5 * 60

# Colours per amp
_AMP_COLORS = {
    "2A01": ("#0288d1", "#01579b"),   # (commanded, actual)
    "2A02": ("#388e3c", "#1b5e20"),
    # "2A03": ("#ef6c00", "#bf360c"),
}


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def generate_layout():
    """Returns the antenna diagnostics page layout."""

    lpr_rows = []
    for i, (key, name, description) in enumerate(LPR_PARAMS):
        pos = i + 1
        is_highlight = any(
            tag in description for tag in ["▲", "▼", "critical"]
        )
        lpr_rows.append(
            dbc.Row(
                [
                    dbc.Col(
                        html.Small(
                            str(pos),
                            style={"color": "#555", "fontFamily": "monospace"},
                        ),
                        width=1,
                    ),
                    dbc.Col(
                        html.Code(
                            name,
                            style={
                                "fontFamily": "monospace",
                                "fontSize": "1.2rem",
                                "fontWeight": "bold", 
                            },
                        ),
                        width=3,
                    ),
                    dbc.Col(
                        html.Div(
                            id=f"lpr-{key}",
                            style={
                                "fontFamily": "monospace",
                                "fontSize": "1.2rem",
                                "color": "#000",
                                "textAlign": "center",
                                "minWidth": "70px",
                                "fontWeight": "bold",
                            },
                        ),
                        width=2,
                    ),
                    dbc.Col(
                        html.Small(
                            description,
                            style={
                                "color": "#f00" if is_highlight else "#000",
                                "fontSize": "1.2rem",
                            },
                        ),
                        width=6,
                    ),
                ],
                className="mb-0 py-0",
                style={
                    "borderBottom": "1px solid #1e2230",
                    "background": "#f8f9fa" if i % 2 == 0 else "#ffffff",
                },
            )
        )

    lpr_panel = dbc.Card(
        [
            dbc.CardHeader(
                dbc.Row([
                    dbc.Col(html.H5("LPR Parameters (read-only)", className="mb-0"), width=9),
                    dbc.Col(
                        html.Small(
                            "Loaded from last LPR command",
                            id="lpr-source-note",
                            style={"color": "#666", "float": "right"},
                        ),
                        width=3,
                    ),
                ]),
                style={"borderBottom": "1px solid #dee2e6"},
            ),
            dbc.CardBody(
                [
                    dbc.Row(
                        [
                            dbc.Col(html.Small("#", style={"color": "#444"}), width=1),
                            dbc.Col(html.Small("Parameter", style={"color": "#444"}), width=3),
                            dbc.Col(html.Small("Value", style={"color": "#444", "textAlign": "right"}), width=2),
                            dbc.Col(html.Small("Description", style={"color": "#444"}), width=6),
                        ],
                        className="mb-1",
                    ),
                    html.Div(lpr_rows),
                ],
                style={"padding": "8px 12px"},
            ),
        ],
        className="mb-3",
        style={},
    )

    amp_panel = dbc.Card(
        [
            dbc.CardHeader(
                html.H5("Amplifier Currents", className="mb-0"),
                style={"borderBottom": "1px solid #dee2e6"},
            ),
            dbc.CardBody(
                [
                    dcc.Graph(
                        id="amp-current-graph",
                        style={"height": "340px"},
                        config={"displayModeBar": False},
                    ),
                    html.Small(
                        "Commanded (dashed) and actual (solid) currents per amplifier. "
                        "Units are raw DAC counts (±2047 ≈ ±10 V). ",
                        style={"color": "#556", "display": "block", "marginTop": "4px"},
                    ),
                ],
                style={"padding": "10px 12px"},
            ),
        ],
        className="mb-3",
        style={},
    )

    pointing_error_panel = dbc.Card(
        [
            dbc.CardHeader(
                html.H5("Pointing Error", className="mb-0"),
                style={"borderBottom": "1px solid #dee2e6"},
            ),
            dbc.CardBody(
                [
                    dcc.Graph(
                        id="az-el-elevation",
                        style={"height": "400px"},
                        config={"displayModeBar": False},
                    ),
                ],
                style={"padding": "10px 12px"},
            ),
        ],
        className="mb-3",
        style={},
    )

    layout = html.Div(
        [
            generate_navbar(
                {
                    "View": [
                        dbc.DropdownMenuItem("Monitor Page", href="/monitor-page"),
                        dbc.DropdownMenuItem("System Page", href="/system-page"),
                    ]
                },
                title="Antenna Diagnostics",
            ),
            html.Div(
                [
                    dbc.Row(
                        [
                            dbc.Col([amp_panel, pointing_error_panel], width=12, lg=5),
                            dbc.Col(lpr_panel, width=12, lg=7),
                        ],
                        style={"margin": "10px 4px"},
                    )
                ]
            ),
        ],
        style={"minHeight": "100vh"},
    )
    return layout


# ---------------------------------------------------------------------------
# Helpers for building graph and reading params
# ---------------------------------------------------------------------------

def _lpr_values_from_status(status):
    """Extract LPR param values from status dict.

    Returns a dict keyed by pAzKo etc., values as floats or None.
    Handles both list (positional) and dict (named) forms.
    """
    if status is None:
        return {}
    raw = status.get("lpr_params", None)
    if raw is None:
        # Fall back to motor_status sub-key
        motor_status = status.get("motor_status", {})
        raw = motor_status.get("lpr_params", None)
    if raw is None:
        return {}
    if isinstance(raw, list):
        result = {}
        for i, (key, _, _) in enumerate(LPR_PARAMS):
            result[key] = raw[i] if i < len(raw) else None
        return result
    if isinstance(raw, dict):
        return raw
    return {}


def _build_amp_current_graph(history):
    """Build a Plotly figure from amp current history.

    history: list of dicts with keys "time", "2A01", "2A02", "2A03"
             each amp value is {"commanded": int, "actual": int}
    """
    fig = go.Figure()

    if not history:
        fig.update_layout(
            plot_bgcolor="white",
            paper_bgcolor="white",
            font=dict(color="#333"),
            xaxis=dict(title="Seconds", gridcolor="#e5e5e5"),
            yaxis=dict(title="Current (DAC counts)", gridcolor="#e5e5e5"),
            margin=dict(l=40, r=10, t=10, b=40),
        )
        return fig

    valid_times = [
        entry.get("time")
        for entry in history
        if isinstance(entry, dict)
        and isinstance(entry.get("time"), (int, float))
    ]
    if valid_times:
        latest_time = max(valid_times)
        times = [
            (entry["time"] - latest_time)
            if isinstance(entry, dict)
            and isinstance(entry.get("time"), (int, float))
            else None
            for entry in history
        ]
    else:
        last_index = max(len(history) - 1, 0)
        times = [idx - last_index for idx in range(len(history))]

    for amp_id in AMP_IDS:
        cmd_color, act_color = _AMP_COLORS[amp_id]
        label = AMP_LABELS[amp_id]

        commanded = [
            e.get(amp_id, {}).get("commanded", None) for e in history
        ]
        actual = [
            e.get(amp_id, {}).get("actual", None) for e in history
        ]

        # Replace sentinel -999999 with None so Plotly gaps them
        commanded = [v if v is not None and v != -999999 else None for v in commanded]
        actual    = [v if v is not None and v != -999999 else None for v in actual]

        fig.add_trace(go.Scatter(
            x=times, y=commanded,
            name=f"{label} cmd",
            mode="lines",
            line=dict(color=cmd_color, width=1.5, dash="dot"),
            connectgaps=False,
        ))
        fig.add_trace(go.Scatter(
            x=times, y=actual,
            name=f"{label} actual",
            mode="lines",
            line=dict(color=act_color, width=1.5),
            connectgaps=False,
        ))

    fig.update_layout(
        plot_bgcolor="white",
        paper_bgcolor="white",
        font=dict(color="#333", size=11),
        xaxis=dict(
            title="Seconds",
            gridcolor="#e5e5e5",
            zerolinecolor="#ccc",
        ),
        yaxis=dict(
            title="Current (DAC counts)",
            gridcolor="#e5e5e5",
            zerolinecolor="#ccc",
            zeroline=True,
            zerolinewidth=1,
        ),
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.01,
            xanchor="right",
            x=1,
            font=dict(size=10),
            bgcolor="rgba(0,0,0,0)",
        ),
        margin=dict(l=45, r=10, t=30, b=40),
        uirevision="amp-currents",
    )
    return fig


def _trim_amp_history(history, window_seconds=AMP_HISTORY_WINDOW_SEC):
    if not history:
        return []
    times = [
        entry.get("time")
        for entry in history
        if isinstance(entry, dict)
    ]
    times = [t for t in times if isinstance(t, (int, float))]
    if not times:
        return history
    latest = max(times)
    cutoff = latest - window_seconds
    return [
        entry for entry in history
        if isinstance(entry, dict)
        and isinstance(entry.get("time"), (int, float))
        and entry["time"] >= cutoff
    ]


# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------

def register_callbacks(app, config, status_thread):
    """Register callbacks for the antenna diagnostics page."""

    # Rolling buffer for amp current history (last 300 samples ≈ 5 min at 1 Hz)
    _amp_history = deque(maxlen=300)

    @app.callback(
        Output("amp-current-graph", "figure"),
        [Input("interval-component", "n_intervals")],
    )
    def update_amp_current_graph(n):
        status = status_thread.get_status()
        if status is not None:
            motor_status = status.get("motor_status", status)
            rotor_diagnostics: dict[str, any] = motor_status.get("rotor_diagnostics", {})
            # rotor_diagnostics keys = [
                #     "mode", "CalSts",
                #     "AzBrkOn", "ElBrkOn", "EmStopOn",
                #     "azerr", "elerr", "amp_currents",
                #     "ElUpPreLim", "ElDnPreLim", "ElUpFinLim", "ElDnFinLim",
                #     "AzCwPreLim", "AzCcwPreLim", "AzCwFinLim", "AzCcwFinLim",
                #     "AzLT180", "SimMode", "safe_mode",
                # ]
            amp_currents = rotor_diagnostics.get("amp_currents", None)
            t = status.get("time", None)
            if amp_currents is not None and t is not None:
                _amp_history.append({"time": t, **amp_currents})

        # Also accept pre-built history from the daemon if it publishes one
        if status is not None and "amp_current_history" in status:
            windowed = _trim_amp_history(status["amp_current_history"])
            return _build_amp_current_graph(windowed)

        return _build_amp_current_graph(_trim_amp_history(list(_amp_history)))

    @app.callback(
        Output("az-el-elevation", "figure"),
        [Input("interval-component", "n_intervals")],
    )
    def update_az_el_time_graph(n):
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
                return generate_pointing_error_graph(error_hist, 5)
        return ""

    # LPR parameter value outputs — one per parameter
    lpr_output_ids = [f"lpr-{key}" for key, _, _ in LPR_PARAMS]

    @app.callback(
        [Output(oid, "children") for oid in lpr_output_ids],
        [Input("interval-component", "n_intervals")],
    )
    def update_lpr_params(n):
        status = status_thread.get_status()
        values = _lpr_values_from_status(status)
        result = []
        for key, _, _ in LPR_PARAMS:
            v = values.get(key, None)
            if v is None:
                result.append("—")
            else:
                # Format: integers without decimal, floats with up to 4 sig figs
                try:
                    fv = float(v)
                    if fv == int(fv) and abs(fv) < 1e6:
                        result.append(str(int(fv)))
                    else:
                        result.append(f"{fv:.4g}")
                except (TypeError, ValueError):
                    result.append(str(v))
        return result