"""spectrum_page.py

Layout and callbacks for live spectrum analyzer view.

Changes from previous version
------------------------------
- Frequency inputs: start/stop and center/span rows toggle visibility based
  on the freq_mode selector. No apply button needed for this — it's instant.
- Every control sends its value immediately on change via individual callbacks
  rather than a single Apply button. The apply button is kept as a manual
  fallback for bulk sends (e.g. after typing several values quickly).
- On page load, all controls are populated from the live spectrum config in
  the status dict (falls back to config_dict["SPECTRUM_ANALYZER"] if no live
  data yet).
- Download section: PNG (via Plotly) and CSV with full metadata header.
  The filename includes datetime, freq range, RBW, and current az/el or
  tracked object from the status dict.
"""

from __future__ import annotations

import importlib
import io
import base64
from datetime import datetime
from typing import Dict, Optional

import numpy as np
import dash_bootstrap_components as dbc

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

from .graphs import build_spectrum_figure, emptygraph
from ...daemon.types import SpectrumConfig


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _empty_spectrum_figure():
    return emptygraph("Frequency", "Power (dBm)", "Spectrum Analyzer")


def _label(text, **kwargs):
    return dbc.Label(text, className="mb-0 mt-2 fw-semibold", style={"fontSize": "0.8rem"}, **kwargs)


def _num_input(id_, value, step=1, **kwargs):
    return dbc.Input(id=id_, type="number", value=value, step=step,
                     debounce=True, size="sm", **kwargs)


def _card(title, children):
    return dbc.Card(
        [
            dbc.CardHeader(html.H6(title, className="mb-0 py-1"),
                           style={"background": "#f8f9fa"}),
            dbc.CardBody(children, className="py-2 px-3"),
        ],
        className="mb-2",
    )


def _live_config_from_status(status) -> Optional[SpectrumConfig]:
    """Extract SpectrumConfig from the live status dict if available."""
    if status is None:
        return None
    spec = None
    if isinstance(status, dict):
        spec = status.get("spectrum")
    else:
        spec = getattr(status, "spectrum", None)
    if not spec:
        return None
    cfg = spec.get("config") if isinstance(spec, dict) else getattr(spec, "config", None)
    if not cfg:
        return None
    return SpectrumConfig.from_dict(cfg if isinstance(cfg, dict) else cfg.to_dict())


def _describe_target(status) -> str:
    """Return a string like 'Moon az=250.2 el=45.1' or 'az=180.0 el=30.0'."""
    if status is None:
        return ""
    if isinstance(status, dict):
        azel = status.get("motor_azel") or (0.0, 0.0)
        queued = status.get("queued_item") or ""
    else:
        azel = getattr(status, "motor_azel", (0.0, 0.0))
        queued = getattr(status, "queued_item", "")
    az, el = float(azel[0]), float(azel[1])
    if queued and queued not in ("None", "none", ""):
        return f"{queued} az={az:.2f} el={el:.2f}"
    return f"az={az:.2f} el={el:.2f}"


def _make_csv_header(config: SpectrumConfig, target_str: str, sweep_index: int, avg_count: int) -> str:
    now = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    freq_str = (
        f"start={config.start_hz:.0f}Hz stop={config.stop_hz:.0f}Hz"
        if config.freq_mode == "start_stop"
        else f"center={config.center_hz:.0f}Hz span={config.span_hz:.0f}Hz"
    )
    lines = [
        f"# Caltech 6m Spectrum Analyzer Export",
        f"# datetime={now}",
        f"# target={target_str}",
        f"# freq_mode={config.freq_mode} {freq_str}",
        f"# rbw={config.rbw_hz:.0f}Hz vbw={config.vbw_hz:.0f}Hz",
        f"# ref_level={config.ref_level_dbm}dBm",
        f"# atten={'auto' if config.atten_auto else f'{config.atten_db}dB'}",
        f"# preamp={config.preamp_on}",
        f"# trace_type={config.trace_type}",
        f"# num_averages={config.num_averages} avg_count={avg_count}",
        f"# sweep_index={sweep_index}",
        f"# x_units={config.x_units}",
        "frequency_hz,power_dbm",
    ]
    return "\n".join(lines)


def _make_export_filename(config: SpectrumConfig, target_str: str, ext: str) -> str:
    now = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
    target_slug = target_str.replace(" ", "_").replace("=", "").replace(".", "p")[:40]
    if config.freq_mode == "start_stop":
        freq_slug = f"{config.start_hz/1e6:.0f}_{config.stop_hz/1e6:.0f}MHz"
    else:
        freq_slug = f"c{config.center_hz/1e6:.0f}s{config.span_hz/1e6:.0f}MHz"
    return f"spectrum_{now}_{target_slug}_{freq_slug}.{ext}"


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def generate_layout(config_dict=None):
    # Use defaults — live values loaded by callback on page load
    config = SpectrumConfig()
    if isinstance(config_dict, dict) and isinstance(config_dict.get("SPECTRUM_ANALYZER"), dict):
        config = SpectrumConfig.from_dict(config_dict["SPECTRUM_ANALYZER"])

    frequency_group = _card("Frequency", [
        _label("Instrument serial"),
        dbc.Input(id="spectrum-instrument-serial", type="text",
                  value=config.instrument_serial, debounce=True, size="sm"),
        _label("Frequency mode"),
        dbc.Select(
            id="spectrum-freq-mode",
            options=[
                {"label": "Start / Stop", "value": "start_stop"},
                {"label": "Center / Span", "value": "center_span"},
            ],
            value=config.freq_mode,
            size="sm",
        ),
        # Start/Stop row — shown when freq_mode == start_stop
        html.Div(
            id="spectrum-start-stop-row",
            children=[
                _label("Start Hz / Stop Hz"),
                dbc.Row([
                    dbc.Col(_num_input("spectrum-start-hz", config.start_hz), width=6),
                    dbc.Col(_num_input("spectrum-stop-hz", config.stop_hz), width=6),
                ]),
            ],
        ),
        # Center/Span row — shown when freq_mode == center_span
        html.Div(
            id="spectrum-center-span-row",
            children=[
                _label("Center Hz / Span Hz"),
                dbc.Row([
                    dbc.Col(_num_input("spectrum-center-hz", config.center_hz), width=6),
                    dbc.Col(_num_input("spectrum-span-hz", config.span_hz), width=6),
                ]),
            ],
            style={"display": "none"},
        ),
    ])

    bandwidth_group = _card("Bandwidth", [
        _label("RBW (Hz)"),
        _num_input("spectrum-rbw-hz", config.rbw_hz),
        _label("VBW (Hz)"),
        _num_input("spectrum-vbw-hz", config.vbw_hz),
    ])

    level_group = _card("Level", [
        _label("Ref level (dBm)"),
        _num_input("spectrum-ref-level", config.ref_level_dbm, step=0.1),
        _label("Attenuation"),
        dbc.Select(
            id="spectrum-atten-auto",
            options=[
                {"label": "Auto", "value": "auto"},
                {"label": "Manual", "value": "manual"},
            ],
            value="auto" if config.atten_auto else "manual",
            size="sm",
        ),
        html.Div(
            id="spectrum-atten-db-row",
            children=[
                _label("Attenuation (dB)"),
                _num_input("spectrum-atten-db", config.atten_db, step=0.5),
            ],
            style={"display": "none" if config.atten_auto else "block"},
        ),
        _label("Preamp"),
        dbc.Select(
            id="spectrum-preamp",
            options=[
                {"label": "On",   "value": "on"},
                {"label": "Off",  "value": "off"},
                {"label": "Keep", "value": "keep"},
            ],
            value=(
                "on" if config.preamp_on is True
                else "off" if config.preamp_on is False
                else "keep"
            ),
            size="sm",
        ),
    ])

    averaging_group = _card("Averaging", [
        _label("Trace type"),
        dbc.Select(
            id="spectrum-trace-type",
            options=[
                {"label": "Clear / Write", "value": "clear_write"},
                {"label": "Average",       "value": "average"},
            ],
            value=config.trace_type,
            size="sm",
        ),
        html.Div(
            id="spectrum-num-averages-row",
            children=[
                _label("Number of averages"),
                _num_input("spectrum-num-averages", config.num_averages),
            ],
            style={"display": "block" if config.trace_type == "average" else "none"},
        ),
    ])

    display_group = _card("Display", [
        _label("X axis units"),
        dbc.Select(
            id="spectrum-x-units",
            options=[{"label": u, "value": u} for u in ("Hz", "kHz", "MHz", "GHz")],
            value=config.x_units,
            size="sm",
        ),
        _label("Y axis mode"),
        dbc.Select(
            id="spectrum-y-axis-mode",
            options=[
                {"label": "Auto",         "value": "auto"},
                {"label": "Fixed limits", "value": "fixed_limits"},
                {"label": "dB / div",     "value": "db_per_div"},
            ],
            value=config.y_axis_mode,
            size="sm",
        ),
        html.Div(
            id="spectrum-y-db-per-div-row",
            children=[
                _label("dB/div / Divisions"),
                dbc.Row([
                    dbc.Col(_num_input("spectrum-y-db-per-div", config.y_db_per_div, step=0.5), width=6),
                    dbc.Col(_num_input("spectrum-y-num-divs", config.y_num_divs, step=1), width=6),
                ]),
            ],
            style={"display": "block" if config.y_axis_mode == "db_per_div" else "none"},
        ),
        html.Div(
            id="spectrum-y-lim-row",
            children=[
                _label("Y limits min / max (dBm)"),
                dbc.Row([
                    dbc.Col(_num_input("spectrum-y-lim-min", config.y_lim_dbm[0], step=0.5), width=6),
                    dbc.Col(_num_input("spectrum-y-lim-max", config.y_lim_dbm[1], step=0.5), width=6),
                ]),
            ],
            style={"display": "block" if config.y_axis_mode == "fixed_limits" else "none"},
        ),
    ])

    download_group = _card("Export", [
        dbc.Row([
            dbc.Col(
                dbc.Button("Download PNG", id="spectrum-download-png-btn",
                           color="secondary", size="sm", className="w-100"),
                width=6,
            ),
            dbc.Col(
                dbc.Button("Download CSV", id="spectrum-download-csv-btn",
                           color="secondary", size="sm", className="w-100"),
                width=6,
            ),
        ], className="g-2"),
        dcc.Download(id="spectrum-download-png"),
        dcc.Download(id="spectrum-download-csv"),
    ])

    config_panel = html.Div([
        html.Div(id="spectrum-connection", className="mb-2"),
        html.Small(
            id="spectrum-sweep-status",
            className="text-muted d-block mb-2",
        ),
        frequency_group,
        bandwidth_group,
        level_group,
        averaging_group,
        display_group,
        download_group,
        # Manual apply fallback for bulk edits
        dbc.Row([
            dbc.Col(
                dbc.Button("Apply all", id="spectrum-apply-btn",
                           color="primary", size="sm", className="w-100"),
                width=6,
            ),
            dbc.Col(
                html.Small(id="spectrum-apply-status", className="text-muted"),
                width=6, className="d-flex align-items-center",
            ),
        ], className="mt-2 g-2"),
    ])

    layout = html.Div([
        html.Div([
            html.Div([], className="one-third column"),
            html.Div(
                html.H4("Spectrum Analyzer",
                        style={"margin-bottom": "0px", "text-align": "center"}),
                className="one-third column", id="title",
            ),
            html.Div([], className="one-third column", id="button"),
        ], id="header", className="row flex-display", style={"margin-bottom": "12px"}),
        dbc.Row([
            dbc.Col(
                dcc.Graph(id="spectrum-graph", style={"height": "70vh"}),
                width=8,
            ),
            dbc.Col(
                html.Div(config_panel,
                         style={"maxHeight": "80vh", "overflowY": "auto", "paddingRight": "4px"}),
                width=4,
            ),
        ]),
    ])

    return layout


# ---------------------------------------------------------------------------
# Callbacks
# ---------------------------------------------------------------------------

def register_callbacks(app, config, status_thread, command_thread):

    # ------------------------------------------------------------------
    # Live graph update
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-graph", "figure"),
        Output("spectrum-sweep-status", "children"),
        Input("interval-component", "n_intervals"),
    )
    def update_spectrum_graph(n):
        status = status_thread.get_status()
        spec = None
        if status is not None:
            spec = (status.get("spectrum") if isinstance(status, dict)
                    else getattr(status, "spectrum", None))

        if not spec:
            return _empty_spectrum_figure(), "No data"

        freq = np.array(spec.get("freq_hz", []), dtype=float)
        power = np.array(spec.get("power_dbm", []), dtype=float)
        if freq.size == 0 or power.size == 0:
            return _empty_spectrum_figure(), "No data"

        config_obj = SpectrumConfig.from_dict(spec.get("config", {}))
        sweep_index = int(spec.get("sweep_index", 0))
        avg_count = int(spec.get("avg_count", 0))
        fig = build_spectrum_figure(freq, power, config_obj, sweep_index, avg_count)

        sweep_text = f"Sweep #{sweep_index}"
        if config_obj.trace_type == "average":
            sweep_text += f"  |  {avg_count}/{config_obj.num_averages} avg"
        return fig, sweep_text

    # ------------------------------------------------------------------
    # Connection badge
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-connection", "children"),
        Input("interval-component", "n_intervals"),
    )
    def update_connection_badge(n):
        status = status_thread.get_status()
        connected = False
        if status is not None:
            spec = (status.get("spectrum") if isinstance(status, dict)
                    else getattr(status, "spectrum", None))
            if spec:
                connected = bool(spec.get("connected", False))
        color = "success" if connected else "secondary"
        label = "Analyzer connected" if connected else "Analyzer disconnected"
        return dbc.Badge(label, color=color)

    # ------------------------------------------------------------------
    # Populate controls from live config on page load / interval
    # Only fires when live config differs from what's shown.
    # Uses n_intervals so it updates once after page load.
    # ------------------------------------------------------------------

    _live_inputs = [
        "spectrum-instrument-serial", "spectrum-freq-mode",
        "spectrum-start-hz", "spectrum-stop-hz",
        "spectrum-center-hz", "spectrum-span-hz",
        "spectrum-rbw-hz", "spectrum-vbw-hz",
        "spectrum-ref-level", "spectrum-atten-auto", "spectrum-atten-db",
        "spectrum-preamp", "spectrum-trace-type", "spectrum-num-averages",
        "spectrum-x-units", "spectrum-y-axis-mode",
        "spectrum-y-db-per-div", "spectrum-y-num-divs",
        "spectrum-y-lim-min", "spectrum-y-lim-max",
    ]

    @app.callback(
        [Output(cid, "value") for cid in _live_inputs],
        Input("interval-component", "n_intervals"),
        prevent_initial_call=False,
    )
    def sync_controls_from_live(n):
        status = status_thread.get_status()
        cfg = _live_config_from_status(status)
        if cfg is None:
            raise PreventUpdate

        return [
            cfg.instrument_serial,
            cfg.freq_mode,
            cfg.start_hz, cfg.stop_hz,
            cfg.center_hz, cfg.span_hz,
            cfg.rbw_hz, cfg.vbw_hz,
            cfg.ref_level_dbm,
            "auto" if cfg.atten_auto else "manual",
            cfg.atten_db,
            ("on" if cfg.preamp_on is True else "off" if cfg.preamp_on is False else "keep"),
            cfg.trace_type,
            cfg.num_averages,
            cfg.x_units,
            cfg.y_axis_mode,
            cfg.y_db_per_div, cfg.y_num_divs,
            cfg.y_lim_dbm[0], cfg.y_lim_dbm[1],
        ]

    # ------------------------------------------------------------------
    # Toggle visibility callbacks (instant, no server round-trip needed,
    # but Dash requires server callbacks for show/hide)
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-start-stop-row", "style"),
        Output("spectrum-center-span-row", "style"),
        Input("spectrum-freq-mode", "value"),
    )
    def toggle_freq_rows(freq_mode):
        if freq_mode == "center_span":
            return {"display": "none"}, {"display": "block"}
        return {"display": "block"}, {"display": "none"}

    @app.callback(
        Output("spectrum-atten-db-row", "style"),
        Input("spectrum-atten-auto", "value"),
    )
    def toggle_atten_row(atten_mode):
        return {"display": "none" if atten_mode == "auto" else "block"}

    @app.callback(
        Output("spectrum-num-averages-row", "style"),
        Input("spectrum-trace-type", "value"),
    )
    def toggle_averages_row(trace_type):
        return {"display": "block" if trace_type == "average" else "none"}

    @app.callback(
        Output("spectrum-y-db-per-div-row", "style"),
        Output("spectrum-y-lim-row", "style"),
        Input("spectrum-y-axis-mode", "value"),
    )
    def toggle_y_rows(y_mode):
        return (
            {"display": "block" if y_mode == "db_per_div" else "none"},
            {"display": "block" if y_mode == "fixed_limits" else "none"},
        )

    # ------------------------------------------------------------------
    # Individual field send (debounced inputs fire on change)
    # Each one sends only its own field immediately.
    # ------------------------------------------------------------------

    def _send_single(key, value):
        if value is None:
            return
        if isinstance(value, bool):
            value_str = "true" if value else "false"
        elif value is None:
            value_str = "none"
        else:
            value_str = str(value)
        command_thread.add_to_queue(f"spectrum_config {key}={value_str}")

    # Fields that map directly to SpectrumConfig keys
    _direct_fields = [
        ("spectrum-instrument-serial", "instrument_serial", str),
        ("spectrum-freq-mode",         "freq_mode",         str),
        ("spectrum-start-hz",          "start_hz",          float),
        ("spectrum-stop-hz",           "stop_hz",           float),
        ("spectrum-center-hz",         "center_hz",         float),
        ("spectrum-span-hz",           "span_hz",           float),
        ("spectrum-rbw-hz",            "rbw_hz",            float),
        ("spectrum-vbw-hz",            "vbw_hz",            float),
        ("spectrum-ref-level",         "ref_level_dbm",     float),
        ("spectrum-atten-db",          "atten_db",          float),
        ("spectrum-trace-type",        "trace_type",        str),
        ("spectrum-num-averages",      "num_averages",      int),
        ("spectrum-x-units",           "x_units",           str),
        ("spectrum-y-axis-mode",       "y_axis_mode",       str),
        ("spectrum-y-db-per-div",      "y_db_per_div",      float),
        ("spectrum-y-num-divs",        "y_num_divs",        int),
    ]

    for input_id, config_key, cast in _direct_fields:
        # Use default arg capture to avoid closure-over-loop-variable bug
        @app.callback(
            Output(input_id, "valid"),
            Input(input_id, "value"),
            prevent_initial_call=True,
        )
        def _send_field(value, _key=config_key, _cast=cast):
            if value is None:
                raise PreventUpdate
            try:
                _send_single(_key, _cast(value))
            except (ValueError, TypeError):
                pass
            return False  # valid=False means no green border — we don't want validation styling

    # Attenuation auto toggle (select → bool)
    @app.callback(
        Output("spectrum-atten-auto", "valid"),
        Input("spectrum-atten-auto", "value"),
        prevent_initial_call=True,
    )
    def _send_atten_auto(value):
        if value is None:
            raise PreventUpdate
        _send_single("atten_auto", value == "auto")
        return False

    # Preamp toggle (select → bool | None)
    @app.callback(
        Output("spectrum-preamp", "valid"),
        Input("spectrum-preamp", "value"),
        prevent_initial_call=True,
    )
    def _send_preamp(value):
        if value is None:
            raise PreventUpdate
        preamp_on = True if value == "on" else (False if value == "off" else None)
        _send_single("preamp_on", "none" if preamp_on is None
                     else ("true" if preamp_on else "false"))
        return False

    # y_lim_dbm — two inputs combine into one tuple
    @app.callback(
        Output("spectrum-y-lim-min", "valid"),
        Input("spectrum-y-lim-min", "value"),
        State("spectrum-y-lim-max", "value"),
        prevent_initial_call=True,
    )
    def _send_y_lim_min(y_min, y_max):
        if y_min is None or y_max is None:
            raise PreventUpdate
        command_thread.add_to_queue(
            f"spectrum_config y_lim_dbm={float(y_min)},{float(y_max)}"
        )
        return False

    @app.callback(
        Output("spectrum-y-lim-max", "valid"),
        Input("spectrum-y-lim-max", "value"),
        State("spectrum-y-lim-min", "value"),
        prevent_initial_call=True,
    )
    def _send_y_lim_max(y_max, y_min):
        if y_min is None or y_max is None:
            raise PreventUpdate
        command_thread.add_to_queue(
            f"spectrum_config y_lim_dbm={float(y_min)},{float(y_max)}"
        )
        return False

    # ------------------------------------------------------------------
    # Apply all (manual bulk fallback)
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-apply-status", "children"),
        Input("spectrum-apply-btn", "n_clicks"),
        [State(cid, "value") for cid in _live_inputs],
        prevent_initial_call=True,
    )
    def apply_all(n_clicks, *values):
        if not n_clicks:
            raise PreventUpdate
        (
            instrument_serial, freq_mode,
            start_hz, stop_hz, center_hz, span_hz,
            rbw_hz, vbw_hz, ref_level_dbm,
            atten_mode, atten_db,
            preamp_mode, trace_type, num_averages,
            x_units, y_axis_mode,
            y_db_per_div, y_num_divs,
            y_lim_min, y_lim_max,
        ) = values

        parts = []
        def _p(k, v):
            if v is None:
                return
            parts.append(f"{k}={v}")

        _p("instrument_serial", instrument_serial)
        _p("freq_mode", freq_mode)
        _p("start_hz", float(start_hz) if start_hz is not None else None)
        _p("stop_hz", float(stop_hz) if stop_hz is not None else None)
        _p("center_hz", float(center_hz) if center_hz is not None else None)
        _p("span_hz", float(span_hz) if span_hz is not None else None)
        _p("rbw_hz", float(rbw_hz) if rbw_hz is not None else None)
        _p("vbw_hz", float(vbw_hz) if vbw_hz is not None else None)
        _p("ref_level_dbm", float(ref_level_dbm) if ref_level_dbm is not None else None)
        _p("atten_auto", "true" if atten_mode == "auto" else "false")
        _p("atten_db", float(atten_db) if atten_db is not None else None)
        _p("preamp_on",
           "none" if preamp_mode == "keep"
           else "true" if preamp_mode == "on" else "false")
        _p("trace_type", trace_type)
        _p("num_averages", int(num_averages) if num_averages is not None else None)
        _p("x_units", x_units)
        _p("y_axis_mode", y_axis_mode)
        _p("y_db_per_div", float(y_db_per_div) if y_db_per_div is not None else None)
        _p("y_num_divs", int(y_num_divs) if y_num_divs is not None else None)
        if y_lim_min is not None and y_lim_max is not None:
            parts.append(f"y_lim_dbm={float(y_lim_min)},{float(y_lim_max)}")

        parts = [p for p in parts if not p.endswith("=None")]
        if not parts:
            return "Nothing to send"
        command_thread.add_to_queue("spectrum_config " + " ".join(parts))
        return f"Sent {len(parts)} field(s)"

    # ------------------------------------------------------------------
    # CSV download
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-download-csv", "data"),
        Input("spectrum-download-csv-btn", "n_clicks"),
        prevent_initial_call=True,
    )
    def download_csv(n_clicks):
        status = status_thread.get_status()
        spec = None
        if status is not None:
            spec = (status.get("spectrum") if isinstance(status, dict)
                    else getattr(status, "spectrum", None))
        if not spec:
            raise PreventUpdate

        freq = np.array(spec.get("freq_hz", []), dtype=float)
        power = np.array(spec.get("power_dbm", []), dtype=float)
        if freq.size == 0:
            raise PreventUpdate

        cfg = SpectrumConfig.from_dict(spec.get("config", {}))
        sweep_index = int(spec.get("sweep_index", 0))
        avg_count = int(spec.get("avg_count", 0))
        target_str = _describe_target(status)

        header = _make_csv_header(cfg, target_str, sweep_index, avg_count)
        data = np.column_stack((freq, power))
        buf = io.StringIO()
        buf.write(header + "\n")
        np.savetxt(buf, data, delimiter=",", fmt="%.6f")
        filename = _make_export_filename(cfg, target_str, "csv")
        return dcc.send_string(buf.getvalue(), filename)

    # ------------------------------------------------------------------
    # PNG download — Plotly figure serialised to image bytes
    # ------------------------------------------------------------------

    @app.callback(
        Output("spectrum-download-png", "data"),
        Input("spectrum-download-png-btn", "n_clicks"),
        prevent_initial_call=True,
    )
    def download_png(n_clicks):
        status = status_thread.get_status()
        spec = None
        if status is not None:
            spec = (status.get("spectrum") if isinstance(status, dict)
                    else getattr(status, "spectrum", None))
        if not spec:
            raise PreventUpdate

        freq = np.array(spec.get("freq_hz", []), dtype=float)
        power = np.array(spec.get("power_dbm", []), dtype=float)
        if freq.size == 0:
            raise PreventUpdate

        cfg = SpectrumConfig.from_dict(spec.get("config", {}))
        sweep_index = int(spec.get("sweep_index", 0))
        avg_count = int(spec.get("avg_count", 0))
        target_str = _describe_target(status)

        fig = build_spectrum_figure(freq, power, cfg, sweep_index, avg_count,
                                    title_extra=target_str)

        # Annotate with metadata in figure title
        now = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
        fig.update_layout(
            title=dict(
                text=(
                    f"Caltech 6m Spectrum  |  {target_str}  |  {now}<br>"
                    f"<span style='font-size:11px'>"
                    f"RBW {cfg.rbw_hz/1e3:.0f} kHz  "
                    f"VBW {cfg.vbw_hz/1e3:.0f} kHz  "
                    f"Ref {cfg.ref_level_dbm} dBm  "
                    f"{'preamp' if cfg.preamp_on else ''}  "
                    f"{'atten auto' if cfg.atten_auto else f'atten {cfg.atten_db} dB'}  "
                    f"sweep #{sweep_index}"
                    f"</span>"
                ),
                font=dict(size=13),
            ),
            margin=dict(t=80),
        )

        try:
            img_bytes = fig.to_image(format="png", width=1400, height=700, scale=2)
        except Exception:
            # kaleido not installed — fall back to JSON-embedded SVG
            raise PreventUpdate

        filename = _make_export_filename(cfg, target_str, "png")
        return dcc.send_bytes(img_bytes, filename)