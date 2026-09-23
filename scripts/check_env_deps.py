"""
Checks that every third-party module imported by the code is declared in
environment.yml. Catches the case where an import works locally only
because some other package happened to pull it in transitively.

    python scripts/check_env_deps.py

Exits non-zero if anything under srt/ is missing. Imports that only appear
under scripts/ are reported as warnings, since those are analysis tools and
are not expected to run in the srt-dev env.

Stdlib only, on purpose — a dependency checker shouldn't have dependencies.
"""

import ast
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SKIP_DIRS = {"node_modules", "__pycache__", ".git", "build", ".schema-tmp", "old_docs"}

# import name -> conda package name, where they differ
ALIAS = {
    "yaml": "pyyaml",
    "serial": "pyserial",
    "zmq": "pyzmq",
    "jwt": "pyjwt",
    "dotenv": "python-dotenv",
    "PIL": "pillow",
    "cv2": "opencv",
    "mpl_toolkits": "matplotlib",
}

# Declared but never imported by name, and legitimately so.
NOT_IMPORTED_BY_NAME = {
    "uvloop",        # uvicorn picks it up at runtime if installed
    "pip", "python", "conda-forge", "-e",
}


def imported_packages() -> dict[str, set[Path]]:
    """Third-party top-level imports -> the files importing them."""
    found: dict[str, set[Path]] = {}
    for path in REPO.rglob("*.py"):
        if any(part in SKIP_DIRS for part in path.parts):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="ignore"))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and not node.level:
                names = [node.module or ""]
            else:
                continue  # relative import = internal
            for name in names:
                top = name.split(".")[0]
                if not top or top in sys.stdlib_module_names or top == "srt":
                    continue
                found.setdefault(ALIAS.get(top, top), set()).add(path)
    return found


def declared_packages() -> set[str]:
    """environment.yml dependency names, sans version pins."""
    text = (REPO / "environment.yml").read_text()
    names = re.findall(r"^\s+-\s+([A-Za-z0-9_.\[\]-]+)", text, re.M)
    return {n.split("=")[0].split("[")[0].lower() for n in names}


def main() -> int:
    declared = declared_packages()
    found = imported_packages()

    errors, warnings = [], []
    for pkg, files in sorted(found.items()):
        if pkg.lower() in declared:
            continue
        rel = sorted(f.relative_to(REPO) for f in files)
        (warnings if all(str(f).startswith("scripts/") for f in rel) else errors).append((pkg, rel))

    for pkg, files in warnings:
        print(f"warning: {pkg} imported but not declared (scripts only: {files[0]})")

    for pkg, files in errors:
        where = f"{files[0]}" + (f" +{len(files) - 1} more" if len(files) > 1 else "")
        print(f"ERROR: {pkg} imported by {where} but missing from environment.yml")

    unused = declared - {p.lower() for p in found} - NOT_IMPORTED_BY_NAME
    if unused:
        print(f"note: declared but never imported: {', '.join(sorted(unused))}")

    if errors:
        print(f"\n{len(errors)} missing dependency/ies. Add them to environment.yml.")
        return 1
    print("environment.yml covers every import under srt/.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
