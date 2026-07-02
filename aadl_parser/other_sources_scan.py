# -*- coding: utf-8 -*-
"""Mark ``.c`` / ``.h`` not listed on subprogram ``source_text`` package as ``other_codes``.

1. Enumerate every ``*.c`` / ``*.h`` under ``input_dir``.
2. Collect ``package`` from each ``subprogram_properties`` entry whose ``name`` is ``source_text``.
3. ``other_codes`` = list of dicts ``{"code_name": relative/path, "code": file text}`` for uninvolved sources.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Set

_OTHERS_SKIP_DIR_NAMES = frozenset(
    {
        ".git",
        "build",
        "install",
        "log",
        "__pycache__",
        ".cursor",
        "node_modules",
    }
)


def _list_all_c_h_files(input_dir: str) -> List[str]:
    """Step 1: all ``.c`` / ``.h`` paths under ``input_dir`` (absolute)."""
    root = os.path.abspath(input_dir)
    out: List[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [
            d for d in dirnames if d not in _OTHERS_SKIP_DIR_NAMES and not d.startswith(".")
        ]
        for fn in filenames:
            if fn.endswith((".c", ".h")):
                out.append(os.path.join(dirpath, fn))
    out.sort()
    return out


def _collect_source_text_package_names(systems: List[Dict[str, Any]]) -> Set[str]:
    """Step 2: ``package`` field on ``source_text`` rows inside ``subprogram_properties`` (lowercased basenames)."""
    names: Set[str] = set()

    def walk(obj: Any) -> None:
        if isinstance(obj, dict):
            sp = obj.get("subprogram_properties")
            if isinstance(sp, list):
                for p in sp:
                    if not isinstance(p, dict):
                        continue
                    if (p.get("name") or "").strip().lower() != "source_text":
                        continue
                    pkg = (p.get("package") or "").strip()
                    if not pkg or pkg.lower() == "default":
                        continue
                    if pkg.endswith((".c", ".h")):
                        names.add(os.path.basename(pkg).lower())
            for v in obj.values():
                walk(v)
        elif isinstance(obj, list):
            for x in obj:
                walk(x)

    for s in systems:
        walk(s)
    return names


def _build_other_codes_list(input_dir: str, referenced_basenames: Set[str]) -> List[Dict[str, str]]:
    root = os.path.abspath(input_dir)
    rows: List[Dict[str, str]] = []
    for ap in _list_all_c_h_files(input_dir):
        base = os.path.basename(ap).lower()
        if base in referenced_basenames:
            continue
        code_name = os.path.relpath(ap, root).replace(os.sep, "/")
        try:
            with open(ap, "r", encoding="utf-8", errors="replace") as f:
                text = f.read()
        except OSError:
            continue
        rows.append({"code_name": code_name, "code": text})
    rows.sort(key=lambda r: r["code_name"])
    return rows


def attach_unreferenced_c_h_as_others(
    input_dir: str,
    systems: List[Dict[str, Any]],
) -> None:
    """Attach the same ``other_codes`` list to every system dict in ``systems``."""
    if not systems or not input_dir or not os.path.isdir(input_dir):
        return
    referenced = _collect_source_text_package_names(systems)
    codes = _build_other_codes_list(input_dir, referenced)
    for s in systems:
        s["other_codes"] = codes
