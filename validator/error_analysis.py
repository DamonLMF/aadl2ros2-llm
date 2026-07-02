#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Normalize ROS2 node.log into a structured JSON object.

Pipeline:
  1. Collect raw error blocks ([ERROR] lines) from the log.
  1b. If `=== colcon build failed (exit N) ===` (from test_agent.py) is present, treat lines above
      it as the build log and extract `CompileError` rows when gcc/clang `file:line: error:` lines match.
      If none match, emit one `CompileError` with the full build log in `exception_message` for downstream LLM.
  2. Deduplicate them.
  3. Pass the raw blocks to an LLM and ask it to output a JSON array
     of error dicts with: node / component / error_type / exception_message / root_cause_analysis.
  4. Scan [WARN] lines: group by component (logger suffix after the last '.').
  5. Return warnings_by_component (timing WARNs), and raw_blocks.
"""

import json
import os
import re
import logging
from typing import Any, Dict, List, Optional, Tuple
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

logger = logging.getLogger(__name__)

ROS_ERROR_RE = re.compile(
    r'^(?:\[[^\]]+\]\s+)?(?:\x1b\[[0-9;]*m)*\[ERROR\](?:\x1b\[[0-9;]*m)*\s+\[\d+\.\d+\]\s+\[([^\]]+)\]:\s+(.+)$'
)
ROS_WARN_RE = re.compile(
    r'^(?:\[[^\]]+\]\s+)?(?:\x1b\[[0-9;]*m)*\[WARN\](?:\x1b\[[0-9;]*m)*\s+\[(\d+\.\d+)\]\s+\[([^\]]+)\]:\s+(.+)$'
)

# templates/llm_component.cpp.j2 — RCLCPP_WARN bodies (thread CPU vs wall-clock Deadline)
_J2_THREAD_COMPUTE_WARN_SUBSTR = "execution time exceeded thread computer time"
_J2_WALL_DEADLINE_WARN_SUBSTR = "deadline exceeded wall clock time"

# colcon / gcc / clang / CMake / make — typical content when node.log captures `colcon build` stderr
_GCC_ERROR_WITH_COL_RE = re.compile(
    r'^(?P<file>.+?\.(?:cpp|c|cc|cxx|h|hpp|hh)):(?P<line>\d+):(?P<col>\d+):\s*error:\s*(?P<msg>.+)$'
)
# file:line: fatal error:  OR  file:line:col: fatal error:  (gcc/clang)
_GCC_FATAL_RE = re.compile(
    r'^(?P<file>.+?\.(?:cpp|c|cc|cxx|h|hpp|hh)):(?P<line>\d+)(?::\d+)?:\s*fatal error:\s*(?P<msg>.+)$'
)
_COMPONENT_FROM_PATH_RE = re.compile(r'[/\\]components[/\\]([^/\\]+)\.(?:cpp|c|cc|cxx)\b', re.IGNORECASE)
# tests/controller_soft_test_node.cpp -> node name controller_soft (for repair routing)
_TEST_NODE_FROM_PATH_RE = re.compile(
    r'[/\\]tests[/\\]([^/\\]+)_test_node\.(?:cpp|c|cc|cxx)\b', re.IGNORECASE
)
# src/controller_soft_node.cpp -> node name controller_soft
_MAIN_NODE_FROM_PATH_RE = re.compile(
    r'[/\\]src[/\\]([^/\\]+)_node\.(?:cpp|c|cc|cxx)\b', re.IGNORECASE
)
# Keep in sync with test_agent.py (append after failed colcon build).
_COLCON_BUILD_FAILED_MARKER_RE = re.compile(r"^=== colcon build failed \(exit \d+\) ===\s*$")
# exception_message format: "path:line: msg" (from _extract_compile_error_dicts)
_COMPILE_ERR_MSG_PATH_RE = re.compile(
    r'^(?P<file>.+?\.(?:cpp|c|cc|cxx|h|hpp|hh)):(?P<line>\d+):\s*',
)


def _logger_to_component_key(logger: str) -> str:
    """Map rclpy logger name (e.g. controller_soft.motions) -> component key (motions)."""
    s = (logger or "").strip().lower()
    if "." in s:
        return s.rsplit(".", 1)[-1]
    return s


def _parse_exec_deadline_ms(msg: str) -> Tuple[Optional[float], Optional[float]]:
    """Return (first_ms, second_ms) from one WARN body; supports rclpy and generic formats (legacy)."""
    if not msg:
        return None, None
    s = msg.lower()
    # "… 6.77 ms > 5.0 ms" / "… 6.77ms > 5.0ms"
    m = re.search(r"([\d.]+)\s*ms\s*>\s*([\d.]+)\s*ms", s)
    if m:
        return float(m.group(1)), float(m.group(2))
    # C++: "… 0.103 > 0.100 ms"
    m = re.search(r"([\d.]+)\s*>\s*([\d.]+)\s*ms\b", s)
    if m:
        return float(m.group(1)), float(m.group(2))
    # "execution took … ms … deadline was … ms"
    m = re.search(r"execution took\s*([\d.]+)\s*ms.*deadline was\s*([\d.]+)\s*ms", s)
    if m:
        return float(m.group(1)), float(m.group(2))
    # "exceeded deadline: … microseconds"
    m = re.search(r"exceeded deadline:\s*([\d.]+)\s*microseconds?\b", s)
    if m:
        return float(m.group(1)) / 1000.0, None
    return None, None


def _timing_pair_ms_gt(body: str) -> Optional[Tuple[float, float]]:
    """Parse `a > b ms` from a WARN body (same shape as templates/llm_component.cpp.j2)."""
    if not body:
        return None
    m = re.search(r"([\d.]+)\s*>\s*([\d.]+)\s*ms\b", body.strip().lower())
    if m:
        return float(m.group(1)), float(m.group(2))
    return None


def _collect_warnings_by_component(lines: List[str]) -> Dict[str, List[Dict[str, Any]]]:
    """Group [WARN] timing lines by component; separate compute-time vs wall-clock deadline summaries."""
    by_comp: Dict[str, List[str]] = {}
    for line in lines:
        m = ROS_WARN_RE.match(line.strip())
        if not m:
            continue
        logger, body = m.group(2), m.group(3).strip()
        by_comp.setdefault(_logger_to_component_key(logger), []).append(body)

    out: Dict[str, List[Dict[str, Any]]] = {}
    for comp, bodies in by_comp.items():
        compute_pairs: List[Tuple[float, float]] = []
        deadline_pairs: List[Tuple[float, float]] = []
        other_pairs: List[Tuple[float, float]] = []
        partial_lines = 0
        for body in bodies:
            s = body.strip().lower()
            pair = _timing_pair_ms_gt(body)
            if pair and _J2_THREAD_COMPUTE_WARN_SUBSTR in s:
                compute_pairs.append(pair)
            elif pair and _J2_WALL_DEADLINE_WARN_SUBSTR in s:
                deadline_pairs.append(pair)
            else:
                e, d = _parse_exec_deadline_ms(body)
                if e is not None and d is not None:
                    other_pairs.append((e, d))
                elif e is not None or d is not None:
                    partial_lines += 1

        if not compute_pairs and not deadline_pairs and not other_pairs and not partial_lines:
            continue

        messages: List[Dict[str, Any]] = []

        def _avg_pair(pairs: List[Tuple[float, float]]) -> Tuple[float, float]:
            am = sum(p[0] for p in pairs) / len(pairs)
            al = sum(p[1] for p in pairs) / len(pairs)
            return am, al

        if compute_pairs:
            am, al = _avg_pair(compute_pairs)
            measured = [p[0] for p in compute_pairs]
            messages.append(
                {
                    "kind": "compute_time",
                    "warning_number": len(compute_pairs),
                    "recommended_param_ms": max(measured),
                    "message": (
                        f"Execution time exceeded thread computer time: {am:.2f} > {al:.2f} ms (avg:{am:.2f}ms)"
                    ),
                }
            )
        if deadline_pairs:
            am, al = _avg_pair(deadline_pairs)
            measured = [p[0] for p in deadline_pairs]
            messages.append(
                {
                    "kind": "deadline",
                    "warning_number": len(deadline_pairs),
                    "recommended_param_ms": max(measured),
                    "message": (
                        f"Deadline exceeded wall clock time: {am:.2f} > {al:.2f} ms (avg:{am:.2f}ms)"
                    ),
                }
            )

        out[comp] = messages
    return out


def patch_timing_overruns(
    ros_arch_path: str,
    output_dir: str,
    warnings_by_component: Dict[str, List[Dict[str, Any]]],
) -> Tuple[List[str], List[str]]:
    """Raise timing limits in ros_arch + component .cpp from warnings_by_component (no log read).

    Patch only when: (max > limit AND count > 5) OR count > 10.
    Returns (patched).
    """
    if not os.path.exists(ros_arch_path):
        return [], []
    compute_pairs: Dict[str, List[Tuple[float, float]]] = {}
    deadline_pairs: Dict[str, List[Tuple[float, float]]] = {}
    counts: Dict[str, int] = {}  # compute_time warning count per comp
    for comp, items in (warnings_by_component or {}).items():
        for item in items or []:
            if not isinstance(item, dict):
                continue
            measured = item.get("recommended_param_ms")
            limit = None
            if measured is None:
                pair = _timing_pair_ms_gt((item.get("message") or "").strip())
                if not pair:
                    continue
                measured, limit = pair
            else:
                pair = _timing_pair_ms_gt((item.get("message") or "").strip())
                if pair:
                    _, limit = pair
                else:
                    limit = float(measured)
            pair = (float(measured), float(limit))
            k = item.get("kind")
            if k == "compute_time":
                counts[comp] = counts.get(comp, 0) + int(item.get("warning_number") or 1)
                compute_pairs.setdefault(comp, []).append(pair)
            elif k == "deadline":
                deadline_pairs.setdefault(comp, []).append(pair)

    # max/p95 measured where measured > limit; apply patch policy
    overruns: Dict[str, Dict[str, float]] = {}
    for comp in set(compute_pairs) | set(deadline_pairs):
        cnt = counts.get(comp, 0)
        cp = compute_pairs.get(comp, [])
        adj: Dict[str, float] = {}
        if cp and any(a > b for a, b in cp):
            max_m = max(a for a, _ in cp)
            lim = cp[0][1]
            if (max_m > lim and cnt > 5) or cnt > 10:
                adj["compute_max_ms"] = max_m
        if adj:
            overruns[comp] = adj

    if not overruns:
        return []
    with open(ros_arch_path, "r", encoding="utf-8") as f:
        arch = json.load(f)
    patched: List[str] = []
    for pkg in arch.get("ROSPackages") or []:
        pkg_name = pkg.get("name", "")
        for node in pkg.get("nodes") or []:
            for comp in node.get("components") or []:
                name = (comp.get("name") or "").strip()
                if name.lower() not in overruns:
                    continue
                cpp_path = os.path.join(output_dir, pkg_name, "src", "components", f"{name}.cpp")
                if not os.path.exists(cpp_path):
                    continue
                with open(cpp_path, "r", encoding="utf-8") as f:
                    src = f.read()
                adj = overruns[name.lower()]
                if "compute_max_ms" in adj:
                    new_v = f"{adj['compute_max_ms']:.6g}"
                    src = re.sub(
                        r'(const\s+double\s+compute_max_ms\s*=\s*)[\d.]+(\s*;)',
                        lambda m2: m2.group(1) + new_v + m2.group(2),
                        src,
                    )
                # if "deadline_ms" in adj:
                #     new_v = f"{adj['deadline_ms']:.6g}"
                #     src = re.sub(
                #         r'(const\s+double\s+deadline_ms\s*=\s*)[\d.]+(\s*;)',
                #         lambda m2: m2.group(1) + new_v + m2.group(2),
                #         src,
                #     )
                with open(cpp_path, "w", encoding="utf-8") as f:
                    f.write(src)
                patched.append(name)

    return patched


def _collect_raw_error_blocks(lines: List[str]) -> List[str]:
    """Scan lines and return a deduped list of raw error text blocks."""
    blocks: List[str] = []
    seen = set()
    i = 0
    while i < len(lines):
        line = lines[i]
        # --- Inline [ERROR] line ---
        if ROS_ERROR_RE.match(line):
            raw = line.strip()
            m = ROS_ERROR_RE.match(raw)
            # Deduplicate by (logger, message) to avoid timestamp-driven duplicates
            dedup_key = (m.group(1), m.group(2)) if m else raw
            if dedup_key not in seen:
                seen.add(dedup_key)
                blocks.append(raw)
            i += 1
        else:
            i += 1
    return blocks


def _compile_log_lines_before_marker(lines: List[str]) -> Optional[List[str]]:
    """Lines strictly above the last `=== colcon build failed (exit N) ===` marker, or None."""
    for i in range(len(lines) - 1, -1, -1):
        if _COLCON_BUILD_FAILED_MARKER_RE.match((lines[i] or "").strip()):
            return lines[:i]
    return None


def _component_from_source_path(file_path: str) -> Optional[str]:
    m = _COMPONENT_FROM_PATH_RE.search(file_path.replace("\\", "/"))
    return m.group(1) if m else None


def _node_from_test_source_path(file_path: str) -> Optional[str]:
    m = _TEST_NODE_FROM_PATH_RE.search(file_path.replace("\\", "/"))
    return m.group(1) if m else None


def _node_from_main_node_source_path(file_path: str) -> Optional[str]:
    m = _MAIN_NODE_FROM_PATH_RE.search(file_path.replace("\\", "/"))
    return m.group(1) if m else None


def _norm_compile_path_for_dedup(path: str) -> str:
    """Stable key for merging duplicate gcc diagnostics (same issue, different lines)."""
    return os.path.normpath((path or "").replace("\\", "/"))


def _format_merged_compile_exception(file_path: str, line_nums: List[int], gcc_msg: str) -> str:
    """Single exception_message for one or more source lines with the same diagnostic text."""
    lines = sorted(set(line_nums))
    if len(lines) == 1:
        return f"{file_path}:{lines[0]}: {gcc_msg}"
    lo, hi = lines[0], lines[-1]
    if lines == list(range(lo, hi + 1)):
        return f"{file_path}:{lo}-{hi}: {gcc_msg}"
    return f"{file_path}: lines {','.join(str(x) for x in lines)}: {gcc_msg}"


def _compile_error_routing(file_path: str) -> Tuple[Optional[str], Optional[str]]:
    """
    Return (node, component) for repair routing. Component wins if path is under components/.
    Otherwise map tests/*_test_node.cpp or src/*_node.cpp to node name.
    """
    comp = _component_from_source_path(file_path)
    if comp:
        return None, comp
    node = _node_from_test_source_path(file_path)
    if node:
        # ROS executable / repair target: <base>_test_node (only rewrite tests/*.cpp for this)
        return f"{node}_test_node", None
    node = _node_from_main_node_source_path(file_path)
    if node:
        return node, None
    return None, None


def _apply_compile_error_routing_to_errors(errors: List[Dict[str, Any]]) -> None:
    """
    After LLM enrichment, set node/component for CompileError from the gcc file path in
    exception_message so repair routing matches deterministic rules (tests/, src/, components/).
    """
    for e in errors:
        if e.get("error_type") != "CompileError":
            continue
        msg = (e.get("exception_message") or "").strip()
        m = _COMPILE_ERR_MSG_PATH_RE.match(msg)
        if not m:
            continue
        file_path = m.group("file").strip()
        node_r, comp_r = _compile_error_routing(file_path)
        if node_r or comp_r:
            e["node"] = node_r
            e["component"] = comp_r


def _extract_compile_error_dicts(lines: List[str]) -> List[Dict[str, Any]]:
    """
    Build CompileError rows from gcc/clang output.
    Deduplicate: same normalized file path + same diagnostic text → one row (line range or list).
    """
    raw_rows: List[Tuple[str, str, str]] = []
    seen_exact: set = set()
    for line in lines:
        s = line.rstrip("\n")
        m = _GCC_ERROR_WITH_COL_RE.match(s.strip()) or _GCC_FATAL_RE.match(s.strip())
        if not m:
            continue
        file_path = m.group("file").strip()
        ln = m.group("line")
        msg = (m.groupdict().get("msg") or "").strip()
        exc = f"{file_path}:{ln}: {msg}"
        if exc in seen_exact:
            continue
        seen_exact.add(exc)
        raw_rows.append((file_path, ln, msg))

    buckets: Dict[Tuple[str, str], Dict[str, Any]] = {}
    order: List[Tuple[str, str]] = []
    for file_path, ln, msg in raw_rows:
        key = (_norm_compile_path_for_dedup(file_path), msg)
        if key not in buckets:
            buckets[key] = {"file_path": file_path, "lines": [], "msg": msg}
            order.append(key)
        buckets[key]["lines"].append(int(ln))

    out: List[Dict[str, Any]] = []
    for key in order:
        b = buckets[key]
        fp = b["file_path"]
        exc = _format_merged_compile_exception(fp, b["lines"], b["msg"])
        node_r, comp_r = _compile_error_routing(fp)
        out.append({
            "node": node_r,
            "component": comp_r,
            "error_type": "CompileError",
            "exception_message": exc,
            "root_cause_analysis": None,
        })
    return out


def _try_parse_json(text: str) -> Optional[Any]:
    if not text:
        return None
    t = text.strip()
    if t.startswith("```"):
        t = re.sub(r"^```[a-zA-Z0-9_-]*\s*", "", t)
        t = re.sub(r"\s*```$", "", t)
    t = t.strip()
    try:
        return json.loads(t)
    except Exception:
        pass
    try:
        start = min(i for i in [t.find("["), t.find("{")] if i != -1)
        end = max(t.rfind("]"), t.rfind("}"))
        if end > start >= 0:
            return json.loads(t[start: end + 1])
    except Exception:
        pass
    return None


def _scan_nodes_components_and_test_nodes(xml_path: str):
    """Scan AADL JSON for process (node) names, thread (component) names, and derived ROS test executable names."""
    node_names = set()
    thread_names = set()
    with open(xml_path, "r", encoding="utf-8") as f:
        aadl_model = json.load(f)
    for system in aadl_model:
        if system['category'] != 'system':
            continue
        for subcomponent in system['subcomponents']:
            category = subcomponent['category']
            if category == 'process':
                node_name = subcomponent['name'].lower()
                node_names.add(node_name)
                for subcomponent in subcomponent['subcomponents']:
                    category = subcomponent['category']
                    if category == 'thread':
                        thread_name = subcomponent['name'].lower()
                        thread_names.add(thread_name)
    return sorted(node_names), sorted(thread_names)

def _parse_raw_block(block: str) -> Dict[str, Any]:
    """Parse a single raw error block into a basic error dict using regex (no LLM)."""
    error: Dict[str, Any] = {
        "node": None,
        "component": None,
        "error_type": None,
        "exception_message": None,
        "root_cause_analysis": None,
    }
    stripped = block.strip()

    m = ROS_ERROR_RE.match(stripped)
    if m:
        logger_name = m.group(1)
        message = m.group(2).strip()
        error["error_type"] = "RuntimeError"
        error["exception_message"] = message
        parts = logger_name.strip().lower().split(".")
        if len(parts) >= 2:
            error["node"] = parts[0]
            error["component"] = parts[-1]
        elif len(parts) == 1:
            error["node"] = parts[0]

    return error


def llm_add_root_cause_analysis(
    errors: List[Dict[str, Any]],
    api_key: str,
    xml_path: str,
    raw_blocks: Optional[List[str]] = None,
) -> Optional[List[Dict[str, Any]]]:
    """Call LLM once after ALL errors are merged to fill node/component and add root_cause_analysis."""
    try:
        from ros_generator_utils import ROSGeneratorUtils
        utils = ROSGeneratorUtils(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        raw_text = "\n\n---\n\n".join(raw_blocks or [])
        node_names, component_names = _scan_nodes_components_and_test_nodes(xml_path)
        errors_json = json.dumps(errors, ensure_ascii=False, indent=2)
        prompt = (
    "You are diagnosing a ROS2 multi-agent log. Think step by step internally, but DO NOT output your reasoning.\n"
    "Output ONLY a JSON array (no markdown, no prose).\n\n"
    "Internal chain-of-thought workflow (keep private):\n"
    "1) Read one row from PARTIALLY PARSED ERRORS and align it with RAW ERROR BLOCKS.\n"
    "2) Infer routing target from path hints and known names.\n"
    "3) Validate that exactly one of node/component is null.\n"
    "4) Write concise root cause and one enforceable rule.\n"
    "5) Preserve schema and order constraints before final output.\n\n"
    "Output rules:\n"
    "- Array length must equal PARTIALLY PARSED ERRORS length; do not merge/drop/reorder rows.\n"
    "- Keep each `exception_message` unchanged at the same index.\n"
    "- For each row, set exactly one of `node` or `component` to null.\n"
    "- Use the file path in `exception_message` (gcc path before `:line:`) for routing.\n"
    "- Path hints: `src/<n>_node.cpp` -> node `<n>`; `components/<name>.(cpp|hpp)` -> component `<name>`.\n"
    "- Names must match the Known lists.\n"
    "- Add `root_cause_analysis` (1-2 sentences) and `enforced_rule` (one actionable rule) per row.\n\n"
    f"Known Nodes: {node_names}\n"
    f"Known Components: {component_names}\n\n"
    "# PARTIALLY PARSED ERRORS\n"
    f"{errors_json}\n\n"
    "# RAW ERROR BLOCKS\n"
    f"{raw_text}\n\n"
    "Each object must have exactly these 6 keys:\n"
    "[\"node\", \"component\", \"error_type\", \"exception_message\", \"root_cause_analysis\", \"enforced_rule\"]\n"
)
        resp = utils.call_langchain(
            prompt=prompt,
            api_key=api_key,
            component_name="node_log_diagnosis",
        )
        obj = _try_parse_json(resp)
        if isinstance(obj, list) and all(isinstance(x, dict) for x in obj):
            _apply_compile_error_routing_to_errors(obj)
            return obj
        return None
    except Exception as e:
        logger.warning(f"LLM root cause analysis failed: {e}")
        return None


def error_analysis(
    log_path: str,
    xml_path: str,
) -> Dict[str, Any]:
    """Parse node.log into basic error dicts (no LLM). Returns raw_blocks for downstream LLM enrichment."""
    empty: Dict[str, Any] = {
        "errors": [],
        "error_types": [],
        "warnings_by_component": {},
        "raw_blocks": [],
    }
    if not os.path.exists(log_path):
        return empty

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        lines = [ln.rstrip("\n") for ln in f]

    warnings_by_component = _collect_warnings_by_component(lines)
    raw_blocks = _collect_raw_error_blocks(lines)
    errors: List[Dict[str, Any]] = [_parse_raw_block(block) for block in raw_blocks]
    # Drop empty parses (e.g. stray blocks)
    errors = [e for e in errors if e.get("error_type") or e.get("exception_message")]

    compile_slice = _compile_log_lines_before_marker(lines)
    if compile_slice is not None:
        compile_errs = _extract_compile_error_dicts(compile_slice)
        if not compile_errs:
            compile_errs = [{
                "node": None,
                "component": None,
                "error_type": "CompileError",
                "exception_message": "\n".join(compile_slice),
                "root_cause_analysis": None,
            }]
        errors.extend(compile_errs)
        full_log_block = "FULL_COLCON_GCC_BUILD_LOG\n" + "\n".join(compile_slice)
        if full_log_block not in raw_blocks:
            raw_blocks.append(full_log_block)

    error_types = sorted({e.get("error_type") for e in errors if e.get("error_type")})

    return {
        "errors": errors,
        "error_types": error_types,
        "warnings_by_component": warnings_by_component,
        "raw_blocks": raw_blocks,
    }
