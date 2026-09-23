"""
Generates TypeScript interfaces from the Pydantic models in
srt/daemon/telescope_types.py and command_types.py. This is the ONLY place
TS types for shared data shapes should come from — never hand-write a
matching interface in the frontend.

Pipeline:
  Pydantic BaseModel or annotation
    -> _schema_for()              (Pydantic's built-in JSON Schema export)
    -> _convert_prefix_items()    (fix tuple typing — see docstring below)
    -> write combined schema.json
    -> shell out to `json-schema-to-typescript` to emit the .d.ts file
"""

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from pydantic import BaseModel, TypeAdapter

# Make daemon/ importable regardless of cwd
SRT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SRT_ROOT / "daemon"))

from srt.daemon.telescope_types import (
    LprParams,
    RotorState,
    SpectrumConfig,
    SpectrumFrame,
    DaemonStatus,
    DaemonConfig
)

from srt.daemon.command_types import (
    TelescopeCommand,
    ObservationPlan
)

OUTPUT_DIR = SRT_ROOT / "svelte-frontend" / "src" / "lib" / "generated"
SCHEMA_TMP_DIR = SRT_ROOT / ".schema-tmp"

REF_TEMPLATE = "#/definitions/{model}"

# (exported name, type). Names are explicit because TelescopeCommand is an
# Annotated discriminated union, not a class: its `__name__` is "Annotated".
MODELS: list[tuple[str, Any]] = [
    ("LprParams", LprParams),
    ("RotorState", RotorState),
    ("SpectrumConfig", SpectrumConfig),
    ("SpectrumFrame", SpectrumFrame),
    ("DaemonStatus", DaemonStatus),
    ("DaemonConfig", DaemonConfig),
    ("TelescopeCommand", TelescopeCommand),
    ("ObservationPlan", ObservationPlan),
]


def _schema_for(tp: Any) -> dict:
    """BaseModels expose model_json_schema(); bare annotations don't, so
    those go through TypeAdapter. Same output either way."""
    if isinstance(tp, type) and issubclass(tp, BaseModel):
        return tp.model_json_schema(ref_template=REF_TEMPLATE)
    return TypeAdapter(tp).json_schema(ref_template=REF_TEMPLATE)


def _convert_prefix_items(node):
    """Recursively rewrites JSON Schema 2020-12 `prefixItems` (what
    Pydantic v2 emits for typed tuples, e.g. tuple[float, float]) into
    the older draft-07 `items`-as-array form.

    json-schema-to-typescript does not understand prefixItems — it
    reads minItems/maxItems correctly (so it knows there are exactly N
    positions) but has no logic to look inside prefixItems for each
    position's type, so every tuple field was coming out as
    [unknown, unknown] instead of e.g. [number, number]. Confirmed via
    a minimal repro against the installed version; this is a real,
    still-open upstream gap (bcherny/json-schema-to-typescript#543),
    not something wrong with the Pydantic-generated schema — the type
    information is present, just under a keyword this tool ignores.

    Rewriting to the older `items: [schema, ...]` form (still fully
    supported, confirmed via the same repro) fixes every tuple field
    in the schema at once — object_locs, object_time_locs, az_limits,
    stow_loc, pointing_error_history, error_logs, etc. — not just one
    field, since this walks the whole schema tree recursively.
    """
    if isinstance(node, dict):
        if "prefixItems" in node:
            node["items"] = [_convert_prefix_items(item) for item in node.pop("prefixItems")]
        for v in node.values():
            _convert_prefix_items(v)
    elif isinstance(node, list):
        for item in node:
            _convert_prefix_items(item)
    return node


def _inline_property_types(definitions: dict) -> dict:
    """Strip `title` below each definition root so properties inline.

    json-schema-to-typescript mints a named alias for every schema carrying
    a title, and Pydantic titles every single field — which is where
    Command1..Command12, Azimuth1, ObjectId1 and the rest come from. With
    titles gone, fields emit as `command?: "point_at_azel"` and
    `azimuth: number` directly, and field descriptions become JSDoc on the
    property itself.

    Each root's title is restored afterward so its interface keeps its name.
    """
    def strip(node):
        if isinstance(node, dict):
            node.pop("title", None)
            for v in node.values():
                strip(v)
        elif isinstance(node, list):
            for v in node:
                strip(v)

    for name, schema in definitions.items():
        strip(schema)
        schema["title"] = name
    return definitions


def build_combined_schema() -> dict:
    """
    Combine all models into one JSON Schema document with each model
    as a named definition, so json-schema-to-typescript emits one
    .d.ts file with all interfaces instead of N separate files.
    """
    definitions: dict = {}

    def add(name: str, schema: dict) -> None:
        """Definitions are keyed by bare class name, so two models sharing
        one silently overwrite each other. Fail instead."""
        prev = definitions.get(name)
        if prev is not None and prev != schema:
            sys.exit(
                f"Duplicate definition {name!r} with differing schemas.\n"
                f"Two Pydantic models share this class name — rename one, or "
                f"the generated TS type will be whichever was merged last."
            )
        definitions[name] = schema

    for name, tp in MODELS:
        schema = _schema_for(tp)
        # Pydantic nests sub-model defs under "$defs" — merge those in too.
        # (TelescopeCommand contributes every member command this way.)
        for sub_name, sub_schema in schema.pop("$defs", {}).items():
            add(sub_name, sub_schema)
        add(name, schema)

    combined = {
        "title": "GeneratedTypes",
        "definitions": _inline_property_types(definitions),
        "type": "object",
        "properties": {
            name: {"$ref": f"#/definitions/{name}"} for name, _ in MODELS
        },
    }

    # Fix tuple typing before handing off to json-schema-to-typescript —
    # see _convert_prefix_items docstring.
    return _convert_prefix_items(combined)


def main():
    SCHEMA_TMP_DIR.mkdir(exist_ok=True)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    schema = build_combined_schema()
    schema_path = SCHEMA_TMP_DIR / "combined_schema.json"
    schema_path.write_text(json.dumps(schema, indent=2))

    output_path = OUTPUT_DIR / "types.ts"

    result = subprocess.run(
        [
            "pnpx",
            "json-schema-to-typescript",
            str(schema_path),
            "--bannerComment",
            (
                "/* eslint-disable */\n"
                "/**\n"
                " * AUTO-GENERATED by srt/tools/generate_ts_types.py\n"
                " * Source of truth: daemon/telescope_types.py\n"
                " * Do not edit this file directly — run the generator instead.\n"
                " */"
            ),
            "--no-additionalProperties",
        ],
        capture_output=True,
        text=True,
        cwd=SRT_ROOT / "svelte-frontend",
    )

    if result.returncode != 0:
        print("Type generation failed:", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(1)

    output_path.write_text(result.stdout)
    print(f"Generated {output_path.relative_to(SRT_ROOT)}")


if __name__ == "__main__":
    main()
