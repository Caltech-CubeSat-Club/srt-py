"""spectrum_page.py

Layout and callbacks for live spectrum analyzer view.
"""

from __future__ import annotations

import importlib
from typing import Dict

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

from .graphs import build_spectrum_figure, empty_spectrum_figure
from ...daemon.types import SpectrumConfig


def _card(title: str, children):
    return dbc.Card(
        [
            dbc.CardHeader(html.H6(title, className="mb-0")),
            dbc.CardBody(children),
        ],
        className="mb-2",
    )


def generate_layout(config_dict=None):
    if isinstance(config_dict, dict) and isinstance(config_dict.get("SPECTRUM_ANALYZER"), dict):
        config = SpectrumConfig.from_dict(config_dict.get("SPECTRUM_ANALYZER"))
    else:
        config = SpectrumConfig()

    frequency_group = _card(
        "Frequency",
        [
            dbc.Label("Instrument serial"),
            dbc.Input(
                id="spectrum-instrument-serial",
                type="text",
                value=config.instrument_serial,
            ),
            dbc.Label("Frequency mode", className="mt-2"),
            dbc.Select(
                id="spectrum-freq-mode",
                options=[
                    {"label": "Start/Stop", "value": "start_stop"},
                    {"label": "Center/Span", "value": "center_span"},
                ],
                value=config.freq_mode,
            ),
            dbc.Row(
                [
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-start-hz",
                            type="number",
                            value=config.start_hz,
                            step=1,
                        ),
                        width=6,
                    ),
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-stop-hz",
                            type="number",
                            value=config.stop_hz,
                            step=1,
                        ),
                        width=6,
                    ),
                ],
                className="mt-2",
            ),
            dbc.Row(
                [
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-center-hz",
                            type="number",
                            value=config.center_hz,
                            step=1,
                        ),
                        width=6,
                    ),
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-span-hz",
                            type="number",
                            value=config.span_hz,
                            step=1,
                        ),
                        width=6,
                    ),
                ],
                className="mt-2",
            ),
            html.Div(
                [
                    html.Small("Start / Stop (Hz)", className="text-muted"),
                    html.Small("Center / Span (Hz)", className="text-muted ms-3"),
                ],
                className="mt-1",
            ),
        ],
    )

    bandwidth_group = _card(
        "Bandwidth",
        [
            dbc.Label("RBW (Hz)"),
            dbc.Input(
                id="spectrum-rbw-hz",
                type="number",
                value=config.rbw_hz,
                step=1,
            ),
            dbc.Label("VBW (Hz)", className="mt-2"),
            dbc.Input(
                id="spectrum-vbw-hz",
                type="number",
                value=config.vbw_hz,
                step=1,
            ),
        ],
    )

    level_group = _card(
        "Level",
        [
            dbc.Label("Ref level (dBm)"),
            dbc.Input(
                id="spectrum-ref-level",
                type="number",
                value=config.ref_level_dbm,
                step=0.1,
            ),
            dbc.Label("Attenuation", className="mt-2"),
            dbc.Select(
                id="spectrum-atten-auto",
                options=[
                    {"label": "Auto", "value": "auto"},
                    {"label": "Manual", "value": "manual"},
                ],
                value="auto" if config.atten_auto else "manual",
            ),
            dbc.Input(
                id="spectrum-atten-db",
                type="number",
                value=config.atten_db,
                step=0.5,
                className="mt-2",
            ),
            dbc.Label("Preamp", className="mt-2"),
            dbc.Select(
                id="spectrum-preamp",
                options=[
                    {"label": "On", "value": "on"},
                    {"label": "Off", "value": "off"},
                    {"label": "Keep", "value": "keep"},
                ],
                value="on" if config.preamp_on is True else ("off" if config.preamp_on is False else "keep"),
            ),
        ],
    )

    averaging_group = _card(
        "Averaging",
        [
            dbc.Label("Trace type"),
            dbc.Select(
                id="spectrum-trace-type",
                options=[
                    {"label": "Clear/Write", "value": "clear_write"},
                    {"label": "Average", "value": "average"},
                ],
                value=config.trace_type,
            ),
            dbc.Label("Num averages", className="mt-2"),
            dbc.Input(
                id="spectrum-num-averages",
                type="number",
                value=config.num_averages,
                step=1,
            ),
        ],
    )

    display_group = _card(
        "Display",
        [
            dbc.Label("X units"),
            dbc.Select(
                id="spectrum-x-units",
                options=[
                    {"label": "Hz", "value": "Hz"},
                    {"label": "kHz", "value": "kHz"},
                    {"label": "MHz", "value": "MHz"},
                    {"label": "GHz", "value": "GHz"},
                ],
                value=config.x_units,
            ),
            dbc.Label("Y axis mode", className="mt-2"),
            dbc.Select(
                id="spectrum-y-axis-mode",
                options=[
                    {"label": "Auto", "value": "auto"},
                    {"label": "Fixed limits", "value": "fixed_limits"},
                    {"label": "dB per div", "value": "db_per_div"},
                ],
                value=config.y_axis_mode,
            ),
            dbc.Row(
                [
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-y-db-per-div",
                            type="number",
                            value=config.y_db_per_div,
                            step=0.5,
                        ),
                        width=6,
                    ),
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-y-num-divs",
                            type="number",
                            value=config.y_num_divs,
                            step=1,
                        ),
                        width=6,
                    ),
                ],
                className="mt-2",
            ),
            dbc.Row(
                [
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-y-lim-min",
                            type="number",
                            value=config.y_lim_dbm[0],
                            step=0.5,
                        ),
                        width=6,
                    ),
                    dbc.Col(
                        dbc.Input(
                            id="spectrum-y-lim-max",
                            type="number",
                            value=config.y_lim_dbm[1],
                            step=0.5,
                        ),
                        width=6,
                    ),
                ],
                className="mt-2",
            ),
            html.Div(
                [
                    html.Small("dB/div, divisions", className="text-muted"),
                    html.Small("Y limits (dBm)", className="text-muted ms-3"),
                ],
                className="mt-1",
            ),
        ],
    )

    config_panel = html.Div(
        [
            html.Div(id="spectrum-connection", className="mb-2"),
            frequency_group,
            bandwidth_group,
            level_group,
            averaging_group,
            display_group,
            dbc.Button("Apply", id="spectrum-apply-btn", color="primary", className="me-2"),
            html.Span(id="spectrum-apply-status", className="text-muted"),
        ]
    )

    layout = html.Div(
        [
            html.Div(
                [
                    html.Div([], className="one-third column"),
                    html.Div(
                        [
                            html.H4(
                                "Spectrum Analyzer",
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
            dbc.Row(
                [
                    dbc.Col(dcc.Graph(id="spectrum-graph"), width=8),
                    dbc.Col(config_panel, width=4),
                ]
            ),
        ]
    )

    return layout


def register_callbacks(app, config, status_thread, command_thread):
    @app.callback(
        Output("spectrum-graph", "figure"),
        [Input("interval-component", "n_intervals")],
    )
    def update_spectrum_graph(n):
        status = status_thread.get_status()
        if status is None or not getattr(status, "spectrum", None):
            return empty_spectrum_figure()

        spec = status.spectrum
        freq = np.array(spec.get("freq_hz", []), dtype=float)
        power = np.array(spec.get("power_dbm", []), dtype=float)
        if freq.size == 0 or power.size == 0:
            return empty_spectrum_figure()

        config_obj = SpectrumConfig.from_dict(spec.get("config", {}))
        sweep_index = int(spec.get("sweep_index", 0))
        avg_count = int(spec.get("avg_count", 0))

        return build_spectrum_figure(freq, power, config_obj, sweep_index, avg_count)

    @app.callback(
        Output("spectrum-connection", "children"),
        [Input("interval-component", "n_intervals")],
    )
    def update_connection_badge(n):
        status = status_thread.get_status()
        connected = False
        if status is not None and getattr(status, "spectrum", None):
            connected = bool(status.spectrum.get("connected", False))

        color = "success" if connected else "secondary"
        label = "Connected" if connected else "Disconnected"
        return dbc.Badge(label, color=color, className="me-2")

    @app.callback(
        Output("spectrum-apply-status", "children"),
        [Input("spectrum-apply-btn", "n_clicks")],
        [
            State("spectrum-instrument-serial", "value"),
            State("spectrum-freq-mode", "value"),
            State("spectrum-start-hz", "value"),
            State("spectrum-stop-hz", "value"),
            State("spectrum-center-hz", "value"),
            State("spectrum-span-hz", "value"),
            State("spectrum-rbw-hz", "value"),
            State("spectrum-vbw-hz", "value"),
            State("spectrum-ref-level", "value"),
            State("spectrum-atten-auto", "value"),
            State("spectrum-atten-db", "value"),
            State("spectrum-preamp", "value"),
            State("spectrum-trace-type", "value"),
            State("spectrum-num-averages", "value"),
            State("spectrum-x-units", "value"),
            State("spectrum-y-axis-mode", "value"),
            State("spectrum-y-db-per-div", "value"),
            State("spectrum-y-num-divs", "value"),
            State("spectrum-y-lim-min", "value"),
            State("spectrum-y-lim-max", "value"),
        ],
        prevent_initial_call=True,
    )
    def apply_spectrum_config(
        n_clicks,
        instrument_serial,
        freq_mode,
        start_hz,
        stop_hz,
        center_hz,
        span_hz,
        rbw_hz,
        vbw_hz,
        ref_level_dbm,
        atten_mode,
        atten_db,
        preamp_mode,
        trace_type,
        num_averages,
        x_units,
        y_axis_mode,
        y_db_per_div,
        y_num_divs,
        y_lim_min,
        y_lim_max,
    ):
        status = status_thread.get_status()
        current = SpectrumConfig()
        if status is not None and getattr(status, "spectrum", None):
            current = SpectrumConfig.from_dict(status.spectrum.get("config", {}))

        updates: Dict[str, Optional[object]] = {}

        if instrument_serial and instrument_serial != current.instrument_serial:
            updates["instrument_serial"] = instrument_serial
        if freq_mode and freq_mode != current.freq_mode:
            updates["freq_mode"] = freq_mode
        if start_hz is not None and float(start_hz) != current.start_hz:
            updates["start_hz"] = float(start_hz)
        if stop_hz is not None and float(stop_hz) != current.stop_hz:
            updates["stop_hz"] = float(stop_hz)
        if center_hz is not None and float(center_hz) != current.center_hz:
            updates["center_hz"] = float(center_hz)
        if span_hz is not None and float(span_hz) != current.span_hz:
            updates["span_hz"] = float(span_hz)
        if rbw_hz is not None and float(rbw_hz) != current.rbw_hz:
            updates["rbw_hz"] = float(rbw_hz)
        if vbw_hz is not None and float(vbw_hz) != current.vbw_hz:
            updates["vbw_hz"] = float(vbw_hz)
        if ref_level_dbm is not None and float(ref_level_dbm) != current.ref_level_dbm:
            updates["ref_level_dbm"] = float(ref_level_dbm)

        atten_auto = atten_mode == "auto"
        if atten_auto != current.atten_auto:
            updates["atten_auto"] = atten_auto
        if atten_db is not None and float(atten_db) != current.atten_db:
            updates["atten_db"] = float(atten_db)

        preamp_on = None
        if preamp_mode == "on":
            preamp_on = True
        elif preamp_mode == "off":
            preamp_on = False
        if preamp_on != current.preamp_on:
            updates["preamp_on"] = preamp_on

        if trace_type and trace_type != current.trace_type:
            updates["trace_type"] = trace_type
        if num_averages is not None and int(num_averages) != current.num_averages:
            updates["num_averages"] = int(num_averages)

        if x_units and x_units != current.x_units:
            updates["x_units"] = x_units
        if y_axis_mode and y_axis_mode != current.y_axis_mode:
            updates["y_axis_mode"] = y_axis_mode
        if y_db_per_div is not None and float(y_db_per_div) != current.y_db_per_div:
            updates["y_db_per_div"] = float(y_db_per_div)
        if y_num_divs is not None and int(y_num_divs) != current.y_num_divs:
            updates["y_num_divs"] = int(y_num_divs)

        if y_lim_min is not None and y_lim_max is not None:
            y_lim = (float(y_lim_min), float(y_lim_max))
            if y_lim != current.y_lim_dbm:
                updates["y_lim_dbm"] = y_lim

        if not updates:
            return "No changes"

        parts = []
        for key, value in updates.items():
            if isinstance(value, tuple):
                value_str = f"{value[0]},{value[1]}"
            elif value is None:
                value_str = "none"
            elif isinstance(value, bool):
                value_str = "true" if value else "false"
            else:
                value_str = str(value)
            parts.append(f"{key}={value_str}")

        command_thread.add_to_queue("spectrum_config " + " ".join(parts))
        return f"Sent {len(updates)} update(s)"
