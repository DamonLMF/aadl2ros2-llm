#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate ROS 2 Jazzy C++ from a ROS architecture JSON (templates + LLM for control_loop bodies).

- Component .hpp: deterministic template
- Component .cpp: template shell; LLM fills only ``control_loop`` inner logic
- Node main / device node: Jinja template (no LLM)
- CMakeLists.txt / package.xml: Jinja (no LLM)

CLI: ``python3 coder_agent.py -r <ros_arch.json> -o <out> -k <api_key>``
"""
import argparse
import json
import logging
from collections import defaultdict
import multiprocessing as mp
import os
import re
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from typing import Any, Dict, List, Optional, Set, Tuple

import coder_template as tmpl
from ros_generator_utils import LLM_CALL_MAX_ATTEMPTS, LLM_CALL_RETRY_SLEEP_S, ROSGeneratorUtils

def _strip_c_comments(code: str) -> str:
    """Remove C/C++ block and line comments, then collapse blank lines."""
    if code.count("\n") < max(8, code.count(";") // 4):
        code = re.sub(r"(?<=[;{}])\s+", "\n", code)
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
    code = re.sub(r'//[^\n]*', '', code)
    code = re.sub(r'\n{3,}', '\n\n', code)
    return code.strip()


def _prepare_source_text(source_text: str, source_name: str, max_chars: int = 16000) -> str:
    """
    Shrink an oversized Source_Text for use in LLM prompts.

    Strategy:
    1. Strip comments (saves ~30 % for typical AADL source files).
    2. If still over budget, keep:
       a. The preamble (all #define / global variables before the first function definition).
       b. The entry-point function whose name matches source_name, and every helper
          function called by it (direct + one-level transitive), extracted by brace-matching.
    3. Hard-cap to max_chars as a final safety net.
    """
    if not source_text:
        return ""

    stripped = _strip_c_comments(source_text)
    if len(stripped) <= max_chars:
        return stripped

    if not source_name:
        return stripped[:max_chars]

    # --- locate the first function definition to split preamble vs. functions ---
    first_func_m = re.search(
        r'^[\w][\w\s\*]*\b\w+\s*\([^)]*\)\s*\{',
        stripped,
        re.MULTILINE,
    )
    preamble = stripped[: first_func_m.start()].strip() if first_func_m else ""

    # --- brace-matched extraction of a named function ---
    def _extract_func(text: str, fname: str) -> Optional[str]:
        pattern = re.compile(
            rf'(?:^|(?<=\n))[\w][\w\s\*]*\b{re.escape(fname)}\s*\([^{{]*\)\s*\{{',
            re.MULTILINE,
        )
        m = pattern.search(text)
        if not m:
            return None
        brace_pos = text.index('{', m.start())
        depth, i = 0, brace_pos
        while i < len(text):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    return text[m.start(): i + 1]
            i += 1
        return None

    entry = _extract_func(stripped, source_name)
    if not entry:
        return stripped[:max_chars]

    # --- collect names called inside the entry function (excluding stdlib) ---
    stdlib_skip = {
        'printf', 'fprintf', 'sprintf', 'scanf', 'malloc', 'free',
        'sqrt', 'cos', 'sin', 'atan2', 'fabs', 'abs', 'log', 'exp',
        'memset', 'memcpy', 'strlen', 'strcpy', 'strcat',
        'constrain', 'degrees',   # macros – no function body to extract
    }
    called_names = set(re.findall(r'\b([a-zA-Z_]\w*)\s*\(', entry)) - {source_name} - stdlib_skip

    helpers: List[str] = []
    for fname in sorted(called_names):
        h = _extract_func(stripped, fname)
        if h:
            helpers.append(h)

    result = preamble + '\n\n' + '\n\n'.join(helpers) + '\n\n' + entry
    result = re.sub(r'\n{3,}', '\n\n', result).strip()
    return result[:max_chars] if len(result) > max_chars else result


def _try_parse_json_object(text: str) -> Optional[Dict[str, Any]]:
    """Take first top-level JSON object from LLM output (strips optional markdown fences)."""
    if not text or not str(text).strip():
        return None
    s = str(text).strip()
    s = re.sub(r"^```(?:json)?\s*", "", s, flags=re.IGNORECASE | re.MULTILINE)
    s = re.sub(r"\s*```\s*$", "", s, flags=re.MULTILINE)
    s = s.strip()
    i = s.find("{")
    if i < 0:
        return None
    # Must use raw_decode: naive brace-matching breaks when string values contain `{` / `}`
    # (e.g. C++ code inside JSON "header" / "source").
    decoder = json.JSONDecoder()
    try:
        obj, _end = decoder.raw_decode(s[i:])
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None


def _cpp_model_namespace(package_name: str) -> str:
    """``sanitize(package_name) + '_model'`` for ``other_codes`` free functions."""
    base = re.sub(r"[^a-z0-9_]", "_", (package_name or "package").lower()).strip("_") or "package"
    if base[0].isdigit():
        base = "pkg_" + base
    return f"{base}_model"


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Zero token usage (no LLM or cached step)
TokenStats = Dict[str, int]
_TOKEN_STATS_ZERO: TokenStats = {
    "prompt_tokens": 0,
    "completion_tokens": 0,
    "total_tokens": 0,
}

_ADA_TO_CPP_TYPE: Dict[str, str] = {
    "boolean": "bool",
    "bool": "bool",
    "integer": "int",
    "int": "int",
    "natural": "unsigned",
    "positive": "unsigned",
    "float": "float",
    "long_float": "double",
    "double": "double",
}


def _shared_var_cpp_type(v: Dict[str, Any]) -> str:
    """Resolve C++ member type for SharedSimState (LLM ``cpp_type``, then Ada ``type``, else ``int``)."""
    ct = str(v.get("cpp_type") or "").strip()
    if ct:
        return ct
    t = str(v.get("type") or "").strip()
    if not t:
        return "int"
    tl = t.lower().replace(" ", "_")
    return _ADA_TO_CPP_TYPE.get(tl, "int")


def _shared_var_cpp_initializer(cpp_type: str, iv: Any) -> str:
    """Emit a single token suitable for brace-or-equal initialization in C++."""
    ct = cpp_type.strip()
    if re.search(r"\bbool\b", ct):
        if isinstance(iv, bool):
            return "true" if iv else "false"
        if iv is None:
            return "false"
        if isinstance(iv, (int, float)) and iv != 0:
            return "true"
        s = str(iv).strip().lower()
        return "true" if s in ("true", "1", "yes") else "false"
    if re.search(r"\b(long\s+double|double|float)\b", ct):
        is_float_only = bool(
            re.search(r"\bfloat\b", ct)
            and not re.search(r"\bdouble\b", ct)
            and not re.search(r"\blong\s+double\b", ct)
        )
        if iv is None:
            return "0.0f" if is_float_only else "0.0"
        try:
            x = float(iv)
        except (TypeError, ValueError):
            x = 0.0
        if is_float_only:
            if x == int(x) and abs(x) < 1e16:
                return f"{int(x)}.0f"
            return f"{x}f"
        if x == int(x) and abs(x) < 1e16:
            return f"{int(x)}.0"
        return repr(x)
    if iv is None:
        return "0"
    if isinstance(iv, bool):
        return "1" if iv else "0"
    try:
        return str(int(iv))
    except (TypeError, ValueError):
        try:
            return str(int(float(iv)))
        except (TypeError, ValueError):
            return "0"


def _normalize_shared_var_for_template(v: Dict[str, Any]) -> Optional[Dict[str, str]]:
    """Build ``name`` / ``cpp_type`` / ``initial_value`` for ``shared_sim_state.hpp.j2``."""
    name = str(v.get("name", "")).strip()
    if not name:
        return None
    cpp_type = _shared_var_cpp_type(v)
    init_tok = _shared_var_cpp_initializer(cpp_type, v.get("initial_value"))
    return {"name": name, "cpp_type": cpp_type, "initial_value": init_tok}


def _sm_dispatch_flags(state_machine: dict) -> tuple[bool, bool]:
    """Return (has_timeout_dispatch, has_non_timeout_dispatch)."""
    has_timeout = has_event = False
    for tr in (state_machine or {}).get("transitions") or []:
        c = str(tr.get("condition", "")).lower()
        if "dispatch" not in c:
            continue
        if "timeout" in c:
            has_timeout = True
        elif c.strip():
            has_event = True
    return has_timeout, has_event


class ROSCodeGenerator:
    """Generate ROS implementation code from architecture JSON using templates and an LLM."""

    def __init__(
        self,
        ros_file: str,
        output_dir: str,
        api_key: Optional[str] = None,
        error_context: str = "",
    ) -> None:
        """
        Args:
            ros_file: Path to the ROS architecture JSON.
            output_dir: Generated package tree root.
            api_key: LLM API key (empty if unset; some steps are skipped without it).
        """
        self.ros_file = ros_file
        self.output_dir = output_dir
        self.prompt_dir = os.path.join(output_dir, "prompts")
        os.makedirs(self.prompt_dir, exist_ok=True)
        self.api_key = api_key
        self.error_context = error_context.strip()

        self.ros_info = os.path.join(output_dir, "ros_info")
        os.makedirs(self.ros_info, exist_ok=True)
        self.node_log_file = os.path.join(self.ros_info, "node.log")
        self.template_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "templates"
        )

        with open(ros_file, "r", encoding="utf-8") as f:
            self.ros_architecture = json.load(f)
        os.makedirs(output_dir, exist_ok=True)

        self.utils = ROSGeneratorUtils(output_dir)
        self.utils.initialize_memory()
        # Paths relative to package dir; .c files compiled as CXX (see CMakeLists template).
        self._package_other_codes: List[str] = []
        self._package_other_codes_hpp_includes: List[str] = []

    def _append_error_context(self, prompt: str, model_ns: Optional[str] = None) -> str:
        """Append dynamic-test error context to prompt for targeted regeneration."""
        if not self.error_context:
            return prompt
        if model_ns:
            scope_rule = (
                f"- Bundled calls: `{model_ns}::<name>` as in headers (not `::` alone when names clash). "
                "No std-clashing typedefs/macros (e.g. `uint64_t`, `#define pow`); add `#include` instead of inventing `extern`."
            )
        else:
            scope_rule = (
                '- Global scope (hard): if the compiler reports "no matching function" or "was not declared in this scope" for a free function call, prefix ONLY that call with `::` to resolve it in the global namespace. Do not rename, refactor, or change any other call sites.'
            )
        prompt += f"""
        # BUG FIX CONTEXT FROM DYNAMIC TEST\n
        You are regenerating code to fix runtime errors. Follow these constraints strictly:
        - Apply MINIMAL CHANGE strategy: modify ONLY code directly related to the listed errors.
        - DO NOT refactor, rename, reorder, or rewrite unrelated logic.
        - DO NOT change public interfaces, topic names/types, callback signatures, class names, file structure, or unrelated behaviors.
        - Keep non-error code paths functionally identical.
        - If multiple fixes are possible, choose the smallest local patch that resolves the error.
        - Preserve existing topic/interface contracts unless explicitly contradicted by error context.
        - Logger rule (hard): if a component has `rclcpp::Logger logger_;`, initialize it in the constructor initializer list with `logger_(node_->get_logger())` or `logger_(node->get_logger())`. Never default-construct it and never assign `logger_ = ...` inside the constructor body.
        - External header rule (hard): never add `#include` directives for external headers that are not present in the generated ROS package. If a missing subprogram/helper caused the error, implement the smallest equivalent local C++ logic inside the regenerated component instead of inventing an external header or undeclared function dependency.
        - Globals (hard): declare or define a variable ONLY if the original source defines it at file scope (`float x = …`, `const float y = …`, `#define Z`). Symbols only used inside functions must not appear as `extern`.
        - Types (hard): emit `struct`/`class`/`typedef`/`enum` ONLY if the original source defines that type at file scope; if the source only references a type (e.g. `struct foo_t *p`), do not redefine `foo_t`.
        {scope_rule}
        - AADL/BA + logs (hard): `event port` — arrival is the event (reject fixes that force `data == "true"` unless the model defines booleans; `"timeout"` / numeric strings count). `on dispatch <input>` may reset without publish; timed dispatch with `<port>!` publishes once on that output. 
        Prefer recognizing arrivals via non-null `*_cache_` (check before `->`) rather than string equality hacks. Log `Received <port>: ...`, `Published <port>: ...`, and `State transition: %s -> %s` when used.
        Run-time error context:
        {self.error_context}
            """
        return prompt

    def _clean_code(self, code: Optional[str]) -> str:
        """Remove think tags, code fences and carriage returns from generated code."""
        if not code:
            return ""
        return (
            code.replace("</think>", "")
            .replace("```cpp", "")
            .replace("```cmake", "")
            .replace("```xml", "")
            .replace("```", "")
            .replace("\r", "")
            .strip()
        )

    @staticmethod
    def _format_shared_vars_block(parsed: Optional[Dict[str, Any]]) -> str:
        """Text appended to component prompts: only the shared-variable list from parsed LLM JSON."""
        if not isinstance(parsed, dict):
            return ""
        v = parsed.get("shared_state_variables")
        if v is None:
            return ""
        return json.dumps({"shared_state_variables": v}, ensure_ascii=False, indent=2)

    def _load_shared_state_prompt_block(self, path: str) -> str:
        """Read ``prompt_block`` from a previous ``*_shared_state_analysis.json`` if present."""
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            return (data.get("prompt_block") or "").strip()
        except OSError as e:
            logger.warning("Could not load shared state analysis from %s: %s", path, e)
            return ""
        except json.JSONDecodeError as e:
            logger.warning("Invalid JSON in shared state file %s: %s", path, e)
            return ""

    def _load_shared_vars(self, package_name: str) -> List[Dict[str, str]]:
        """Load identified shared variable names (exclude mutex/pthread) from analysis artifact."""
        path = os.path.join(
            self.prompt_dir, f"{package_name}_shared_state_analysis.json"
        )
        if not os.path.exists(path):
            return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            vars_list = (data.get("parsed") or {}).get("shared_state_variables") or []
            out: List[Dict[str, str]] = []
            for v in vars_list:
                if not isinstance(v, dict):
                    continue
                if "mutex" in v.get("name", "").lower():
                    continue
                if "pthread" in v.get("name", "").lower():
                    continue
                nv = _normalize_shared_var_for_template(v)
                if nv:
                    out.append(nv)
            return out
        except Exception as e:
            logger.warning("Could not load shared_vars from %s: %s", path, e)
            return []

    def _run_shared_code_llm(
        self, shared_block: Dict[str, Any], package_name: str, out_path: str
    ) -> Tuple[str, TokenStats]:
        """Call LLM once per package on ``shared_code.source_text``; return (prompt_block, token_stats)."""
        zero = _TOKEN_STATS_ZERO.copy()
        st = shared_block.get("source_text")
        if st is None:
            src = ""
        elif isinstance(st, list):
            src = "\n\n".join(str(x or "") for x in st)
        else:
            src = str(st)
        eng = f"""You should analyze shared source from an AADL model: several subprograms in one process share this code (globals, mutex, etc.).
Source text: {src}
Task: Read the code and list identifiers that are **shared mutable state** the whole node must use once.
For each variable also record its **initial value** as declared in the source code.
Output: **One JSON object only** (valid JSON, no markdown fences, no other text), exactly this form:
{{
  "shared_state_variables": [{{"name": "<identifier>", "initial_value": <integer>, "rationale": "<one short English sentence why it is shared state>"}}]
}}
"""
        su = ROSGeneratorUtils(self.output_dir)
        su.initialize_memory()
        response = su.call_langchain(
            eng,
            api_key=self.api_key,
            component_name=None,
            save_memory=False,
        )
        token_delta = _TOKEN_STATS_ZERO.copy()
        parsed = _try_parse_json_object(response) if response else None
        block = self._format_shared_vars_block(parsed)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(
                {"prompt_block": block, "raw_response": response, "parsed": parsed},
                f,
                ensure_ascii=False,
                indent=2,
            )
        logger.info("Wrote shared_code LLM analysis: %s", out_path)
        logger.info(
            "[shared_code] Package %s: shared state analysis tokens %s", package_name, token_delta
        )
        return block, token_delta

    def _package_shared_state_context(
        self, package_info: Dict[str, Any], package_name: str, incremental_only: bool
    ) -> Tuple[str, TokenStats]:
        """If the package has ``shared_code``, run (or from cache, load) one analysis for all its components."""
        sc = package_info.get("shared_code")
        if not sc or not isinstance(sc, dict):
            return "", _TOKEN_STATS_ZERO.copy()
        out_path = os.path.join(
            self.prompt_dir, f"{package_name}_shared_state_analysis.json"
        )
        if incremental_only and os.path.exists(out_path):
            loaded = self._load_shared_state_prompt_block(out_path)
            if loaded:
                return loaded, _TOKEN_STATS_ZERO.copy()
        return self._run_shared_code_llm(sc, package_name, out_path)

    def _build_component_state_logic_prompt(
        self,
        component: Dict[str, Any],
        header_contract: str = "",
        node_name: str = "",
        shared_state_context: str = "",
        package_name: str = "",
    ) -> str:
        """Prompt for LLM: only the inner body of control_loop() (state transitions / control law)."""
        name = component['name'].replace('"', '')
        model_ns = _cpp_model_namespace(package_name) if (package_name or "").strip() else ""
        callbacks = component.get('callbacks', [])
        outputs = component.get('outputs', [])
        state_machine = component.get('state_machine', {}) or {}
        node_base = (node_name or "").strip()
        ros_logger_prefix = f"{node_base}.{name}" if node_base else f"<ROS_NODE_NAME>.{name}"
        subprograms = [
            {
                **sub,
                'properties': {
                    **sub.get('properties', {}),
                    **({
                        k: _prepare_source_text(
                            sub['properties'].get(k, ''),
                            sub['properties'].get('Source_Name', ''),
                        )
                        for k in ('Source_Text', 'source_text')
                        if sub['properties'].get(k)
                    })
                },
            }
            for sub in (component.get('subprograms') or [])
        ]
        fc = tmpl.fsm_template_context(component)
        sm_names = tmpl.fsm_state_names(component)
        has_subscriptions = bool(callbacks)

        if fc["use_fsm"] and len(sm_names) >= 2:
            w = sm_names[0]
            fsm_shell_bullet = (
                f"- Template initializes `fsm_state_` to `\"{w}\"`."
                " Log every state change with exactly:"
                " `std::string old_state = fsm_state_; fsm_state_ = \"<new>\";`"
                " then `RCLCPP_INFO(logger_, \"State transition: %s -> %s\", old_state.c_str(), fsm_state_.c_str());`"
                " — runtime verification depends on the exact substring `State transition:`."
            )
        elif fc["use_fsm"]:
            w = sm_names[0]
            fsm_shell_bullet = (
                f"- Single-state FSM `\"{w}\"` (template-managed):"
                " do NOT read, write, or log `fsm_state_`; do NOT emit any `State transition` lines."
            )
        elif has_subscriptions:
            fsm_shell_bullet = (
                "- No template-level cache gate; `control_loop` runs every tick."
            )
        else:
            fsm_shell_bullet = (
                "- No input ports; `control_loop` runs unconditionally every tick."
            )

        cache_rule = (
            "Available members: each callback port cache `<port>_cache_` and state variables from the header."
            if has_subscriptions else
            "Available members: state variables from the header only (no subscriptions, no `*_cache_` members)."
        )

        by_lp: Dict[str, List[str]] = defaultdict(list)
        for _o in outputs:
            lp = str(_o.get("port", "")).strip().lower()
            if lp:
                by_lp[lp].append(f"pub_{tmpl.output_cpp_member(_o)}_")
        fanout_rule_block = "\n".join(
            "- **Output fan-out**: port `{lp}` → publish on ALL of {pubs}; log once using the logical port name.".format(
                lp=lp, pubs=", ".join(sorted(set(pubs)))
            )
            for lp, pubs in sorted(by_lp.items())
            if len(set(pubs)) > 1
        )
        if fanout_rule_block:
            fanout_rule_block += "\n"

        has_timeout, has_event = _sm_dispatch_flags(state_machine)
        if has_timeout:
            active_condition = (
                "runs on each timer tick; `on dispatch timeout` → unconditional `output!` every tick."
                + (" Other transitions: guard only the caches they read." if has_event else "")
            )
        elif has_subscriptions:
            active_condition = "runs on each timer tick; sample-and-hold on latest non-null caches."
        else:
            active_condition = "runs unconditionally on each timer tick (no subscriptions)."

        step3_timeout_bullet = (
            "  `on dispatch timeout`: fire every tick; do not guard `output!` on unrelated `*_cache_`.\n"
            if has_timeout else ""
        )

        bundled_call = ""
        if model_ns:
            bundled_call = (
                f"\n  Subprogram C in `subprograms_excerpt` is reference only: inline its logic inside `control_loop`; "
                f"do NOT add helper/wrapper/local functions. Do NOT call `*_aadl` unless that exact name is declared in a bundled `.hpp`. "
                f"When the C body calls model routines, use `{model_ns}::<name from bundled .hpp>` (e.g. `aircraft_dynamics`, not `aircraft_dynamics_aadl`). "
                f"Prefer `{model_ns}::` over bare `::` when the component class name collides with the free function."
            )

        _props = component.get('properties', {}) or {}
        _, _deadline_ms, _compute_max_ms = tmpl.parse_period_deadline_compute(_props)
        _timing_bullets = []
        if _compute_max_ms is not None:
            _timing_bullets.append(
                f"- Template already declares `compute_max_ms={_compute_max_ms:.3f}` and wraps "
                "your code with `clock_gettime(CLOCK_THREAD_CPUTIME_ID, ...)` / RCLCPP_WARN for overrun."
            )
        if _deadline_ms is not None:
            _timing_bullets.append(
                f"- Template already declares `deadline_ms={_deadline_ms:.3f}` and wraps "
                "your code with `clock_gettime(CLOCK_MONOTONIC, ...)` / RCLCPP_WARN for deadline overrun."
            )
        _timing_section = (
            "\n".join(_timing_bullets) + "\n"
            if _timing_bullets
            else "- No compute/deadline timing instrumentation in this component (thresholds absent or sub-ms).\n"
        )

        prompt = f"""
You are an expert ROS 2 Jazzy C++ developer (rclcpp). Your task: generate ONLY the inner statements of `void {name}::control_loop()` — {active_condition}

The surrounding .cpp template is fixed and already contains:
{_timing_section}{fsm_shell_bullet}
Output ONLY C++ statements — no function signature, no `#include`, no `main()`, no wrapper class.

Header contract (member names and types to use exactly):
{header_contract}

Model context:
callbacks={json.dumps(callbacks, ensure_ascii=False, separators=(',', ':'))}
outputs={json.dumps(outputs, ensure_ascii=False, separators=(',', ':'))}
state_machine={json.dumps(state_machine, ensure_ascii=False, separators=(',', ':'))}
subprograms_excerpt={json.dumps(subprograms, ensure_ascii=False, separators=(',', ':'))}
logger_prefix={ros_logger_prefix}

Before writing any code, reason through the following steps in order:

Step 1 — Identify ports.
  List each callback port (input cache `<port>_cache_`) and each output port (`pub_<port>_`).
  Note that any cache may be nullptr on early ticks; no cache is guaranteed non-null at startup.

Step 2 — Analyse state_machine transitions.
  For every transition, note: source state, target state, condition (empty = unconditional), actions.
  Unconditional transitions (empty condition) MUST execute every tick regardless of cache availability.
  If state_machine is absent, derive behavior from subprograms_excerpt instead.

Step 3 — Determine per-transition cache guards.
  For each transition, identify which caches are actually read in its actions.
  Guard only those reads with `if (<port>_cache_ != nullptr)` immediately before the dereference.
  Do NOT wrap the entire FSM or multiple unrelated states in a single `if (a && b && ...)` block.
  For std_msgs::msg::String boolean-like ports: derive truth from payload (`->data == "true"`), not from pointer alone.
{step3_timeout_bullet}
Step 4 — Write the C++ implementation.
  Use `{name}::` member names exactly as in the header contract; use `logger_`, `node_`, `pub_<name>_`; no `sub_*` inside the loop.{bundled_call}
  {cache_rule}
{fanout_rule_block}  If using state_machine: implement transitions as `if / else if` branches in the order from Step 2; do NOT collapse multi-state sequences.
  If using subprograms_excerpt (no state_machine): implement only this component's bound subprogram body inline (ignore other routines in the same excerpt); no new functions; no `*_aadl` calls unless declared in bundled headers.
  State change (only when fsm_state_ exists): capture old state → assign new state → immediately log `RCLCPP_INFO(logger_, "State transition: %s -> %s", old_state.c_str(), fsm_state_.c_str());`
  Publish: `std_msgs::msg::T m; m.data = x; pub_port_->publish(m);` then log `RCLCPP_INFO(logger_, "Published %s: %f", "<port>", static_cast<double>(m.data));`
  Multi-output log: `RCLCPP_INFO(logger_, "Published %s: %f, %s: %f", "port_a", va, "port_b", vb);`
  Never use `RCLCPP_DEBUG`; never include the substring `error` in any log message string.

Output ONLY the executable C++ from Step 4 — no markdown fences, no commentary outside `//` line comments.
"""
        if (shared_state_context or "").strip():
            names_md = ", ".join(
                f"`sim_state_->{v['name']}`"
                for v in (getattr(self, "_current_shared_vars", None) or [])
            )
            prompt += f"""
---
Shared state (mutex-protected struct `SharedSimState`, member `sim_state_`).
Lock: `std::lock_guard<std::mutex> guard(sim_state_->mtx);`
Fields (int): {names_md or "(see analysis below)"}
Rules for shared state access:
- These fields are the C++ equivalents of the C globals in the source (e.g. `simu.c`). They represent **mutable plant/actuator state** shared across all components in this process.
- **Read**: use `sim_state_->FIELD` wherever the original C subprogram reads the global.
- **Write-back is mandatory**: if the original C subprogram **assigns** to a global (e.g. `WaterLevel_Value = WaterLevel_Value - CmdPump_Value*4 + 2;`), your C++ code **MUST** write it back: `sim_state_->WaterLevel_Value = ...;`
- All reads **and** writes must be inside `std::lock_guard<std::mutex> guard(sim_state_->mtx);`
- Never declare a local copy that shadows the shared field; update `sim_state_->FIELD` in place.
Identified shared variables:
{shared_state_context}
"""
        return prompt

    def _indent_llm_control_body(self, code: str) -> str:
        """Normalize LLM output to 4-space indented lines inside control_loop."""
        code = self._clean_code(code).strip()
        if not code:
            return "    // TODO: implement control_loop state / transition logic\n"
        out_lines: List[str] = []
        for line in code.splitlines():
            if line.strip():
                out_lines.append("    " + line.strip())
            else:
                out_lines.append("")
        return "\n".join(out_lines) + "\n"

    @staticmethod
    def _component_has_behavior_for_comparison(component: Dict[str, Any]) -> bool:
        sm = component.get("state_machine") or {}
        if sm.get("transitions") or sm.get("states"):
            return True
        return bool(component.get("subprograms"))

    def _package_behavior_for_comparison(self, component: Dict[str, Any]) -> str:
        """ROS2 JSON fragment: BA state_machine and subprograms only (trimmed Source_Text)."""
        subprograms = [
            {
                **sub,
                "properties": {
                    **sub.get("properties", {}),
                    **{
                        k: _prepare_source_text(
                            sub["properties"].get(k, ""),
                            sub["properties"].get("Source_Name", ""),
                        )
                        for k in ("Source_Text", "source_text")
                        if sub.get("properties", {}).get(k)
                    },
                },
            }
            for sub in (component.get("subprograms") or [])
        ]
        blob = {
            "component_name": (component.get("name") or "").replace('"', ""),
            "state_machine": component.get("state_machine") or {},
            "subprograms": subprograms,
        }
        raw = json.dumps(blob, ensure_ascii=True, separators=(",", ":"))
        max_arch = 24000
        if len(raw) > max_arch:
            return raw[: max_arch // 2] + "\n/* ... truncated ... */\n" + raw[-(max_arch // 2) :]
        return raw

    @staticmethod
    def _truncate_for_comparison_cpp(text: str, max_len: int = 52000) -> str:
        text = (text or "").strip()
        if len(text) <= max_len:
            return text
        h = max_len // 2
        return text[:h] + "\n/* ... truncated ... */\n" + text[-h:]

    def _build_code_comparison_prompt(
        self, architecture_fragment: str, source_code: str
    ) -> str:
        """English-only instructions; model must emit a single JSON object."""
        return f"""# Role
You are a rigorous embedded-software formal verification expert.
Your task is to strictly compare the design intent from the AADL architecture model (Ground Truth) with the generated ROS2/C++ code (Generated Code), and extract code evidence for generation-quality metrics.

# Task
You are given two inputs:
1) Model Requirement Set: logic points extracted from AADL Behavior Annex and Subprograms.
2) Generated Code: generated C++ / ROS2 source code.

Chain of Thought
1. Find matches (TP - True Positive): for each model requirement, locate concrete implementation code in generated code.
2. Find omissions (FN - False Negative): requirements that exist in the model but are not implemented in generated code.
3. Find hallucinations (FP - False Positive): logic present in generated code but not defined by the model requirements, such as fabricated state transitions, unauthorized variable access, or redundant control logic.
   Note: standard ROS2 framework boilerplate (for example rclcpp init/spin/timer/publisher/subscriber scaffolding) is NOT FP.
4. Scope binding (strict): for this component, audit ONLY the routine bound by Source_Name (fallback: subprogram name). Do NOT create FN entries for other routines that appear in the same source file but are not the bound routine.
5. Label discipline (strict):
   - If the generated code performs an extra state update / side effect that is NOT required by the bound routine, label it as FP (hallucination). Do NOT label it as FN.
   - Do NOT label allowed implementation mapping/scaffolding as FP, especially: mutex/lock guards, and publishing modeled output ports via ROS2 publishers.

Relaxed evaluation policy:
- Prefer semantic equivalence over literal/textual equivalence.
- Count behaviorally equivalent implementations as TP even if structure differs (for example callback + periodic loop split, renamed temporaries, reordered non-observable statements).
- Do NOT count engineering scaffolding as FP: ROS2 boilerplate, mutex/lock guards, QoS setup, caches, type-conversion glue, defensive checks, timing/deadline instrumentation, and logging format differences with equivalent meaning.
- Do NOT mark ROS2 publish of modeled output ports as FP; treat publish calls as expected implementation mapping of model output assignments.
- For logs, match by intent, not exact text; do not mark FN for debug-only printf/Put_Line omitted or mapped to RCLCPP_INFO.
- Mark FP only when extra logic introduces model-external behavior that changes observable semantics (new unauthorized state transition, unauthorized data dependency, or output not derivable from model intent).
- When uncertain between TP and FP for non-functional scaffolding, prefer TP (or omit FP) and add a warning.
- Provide statement-level traceability: map each modeled assignment/condition/transition/action to concrete generated-code statements; if missing, mark `FN` with `code_snippet: "null"`.

Compute metrics exactly:
Recall = TP / (TP + FN)
Precision = TP / (TP + FP)

# Output Format Specification
Output one and only one valid JSON object.
Do not output markdown fences, explanations, or any extra text before/after JSON.

The JSON must strictly follow this schema:
{{
  "analysis_details": [
    {{
      "requirement_description": "<string: a specific model requirement, or a specific hallucinated generated-code behavior>",
      "status_label": "<enum: TP | FN | FP>",
      "code_snippet": "<string: concrete C++ snippet; for FN, use 'null'>",
      "justification": "<string: concise reason for this label, focused on semantic equivalence or hallucination cause>"
    }}
  ],
  "statistics": {{
    "total_TP": "<integer: total TP count>",
    "total_FN": "<integer: total FN count>",
    "total_FP": "<integer: total FP count>",
    "Recall": "<number: TP/(TP+FN)>",
    "Precision": "<number: TP/(TP+FP)>"
  }}
}}

code_snippet rule:
- code_snippet must be an exact substring copied from the provided Generated Code. Do not paraphrase and do not invent variable names or statements.

Model Requirement Set (from AADL BA + Subprogram):
{architecture_fragment}

Generated Code:
{self._truncate_for_comparison_cpp(source_code)}
"""

    def _run_code_comparison_after_component(
        self,
        component: Dict[str, Any],
        source_code: str,
        utils: Optional[ROSGeneratorUtils] = None,
    ) -> Dict[str, int]:
        """POST codegen LLM check; writes ros_info/Code_comparison/<component>.json. Returns token delta."""
        zero = _TOKEN_STATS_ZERO.copy()
        out_dir = os.path.join(self.ros_info, "code_comparison")
        os.makedirs(out_dir, exist_ok=True)
        cname = (component.get("name") or "").replace('"', "")
        out_path = os.path.join(out_dir, f"{cname}.json")
        if not self._component_has_behavior_for_comparison(component):
            return zero
        arch_fragment_prepared = self._package_behavior_for_comparison(component)
        prompt = self._build_code_comparison_prompt(
            arch_fragment_prepared, source_code
        )

        llm_utils = utils if utils is not None else self.utils
        raw = llm_utils.call_langchain(
            prompt=prompt,
            api_key=self.api_key,
            component_name=f"{cname}_code_comparison",
            save_memory=False,
        )
        delta = _TOKEN_STATS_ZERO.copy()
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(raw or "")
        logger.info("Wrote AADL vs C++ comparison: %s", out_path)
        return delta


    def _build_device_logic_prompt(
        self,
        node_info: Dict[str, Any],
        package_name: str,
        baseline_cpp: str,
    ) -> str:
        node_name = (node_info.get("name") or "").strip()
        return f"""
You are an expert ROS 2 Jazzy C++ (rclcpp) developer.
Task: Update the provided device node C++ baseline to implement behavior logic from model context.

Hard constraints (must follow):
- Keep topic names, message types, class name, node name, and file structure compatible with baseline.
- Keep includes unless a strictly necessary include is missing.
- Do NOT rename publishers/subscribers/timer/member fields already present in baseline.
- Keep QoS setup unchanged.
- Keep code compilable with modern C++ and rclcpp.
- Preserve existing baseline behavior when model context has no clear behavioral rule.
- Add runtime logs for analyzer compatibility:
  - On receive: `RCLCPP_INFO(this->get_logger(), "Received <port>: ...")`
  - On publish: `RCLCPP_INFO(this->get_logger(), "Published <port>: ...")`
  Use exact `Received` / `Published` prefixes.
- **State changes must appear in logs (hard requirement)**: If the device/Behavior Annex implies an FSM or discrete states, maintain an explicit state variable and **every time** the architectural state changes, immediately log with exactly:
  `RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", from_state.c_str(), to_state.c_str());`
  Do not update state without this log on the same path; do not log spurious transitions when the state is unchanged. If the model is a single-state self-loop only, do not fabricate multi-step `State transition` logs.

Model context:
package_name={package_name}
node_info={json.dumps(node_info, ensure_ascii=False, separators=(',', ':'))}

Baseline C++ (edit this, return final full file only):
{baseline_cpp}

Output requirements:
- Return ONE complete C++ source file only.
- No markdown fences, no explanations.
"""
    def _generate_device_node_with_llm(
        self,
        node_info: Dict[str, Any],
        package_name: str,
        baseline_cpp: str,
    ) -> Tuple[str, TokenStats]:
        """LLM refinement for device node when behavior attachment exists."""
        prompt = self._build_device_logic_prompt(node_info, package_name, baseline_cpp)
        prompt = self._append_error_context(
            prompt, model_ns=_cpp_model_namespace(package_name)
        )
        node_name = (node_info.get("name") or "").strip()
        prompt_path = os.path.join(self.prompt_dir, f"{node_name}_device_prompt.txt")
        try:
            with open(prompt_path, "w", encoding="utf-8") as f:
                f.write(prompt)
        except OSError as e:
            logger.warning("Failed to write device prompt %s: %s", prompt_path, e)

        refined = self.utils.call_langchain(
            prompt=prompt,
            api_key=self.api_key,
            component_name=node_name or None,
            save_memory=False,
        )
        token_stats = _TOKEN_STATS_ZERO.copy()
        return self._clean_code(refined or baseline_cpp), token_stats

    def _generate_single_component_hpp_cpp(
        self,
        component: Dict[str, Any],
        package_name: str,
        use_memory: bool = False,
        utils: Optional[ROSGeneratorUtils] = None,
        memory_context_name: Optional[str] = None,
        save_memory: bool = True,
        node_name: str = "",
        shared_state_context: str = "",
        shared_vars: Optional[List[Dict[str, str]]] = None,
    ) -> Tuple[Dict[str, Any], TokenStats]:
        """Generate one component: template .hpp, then template .cpp with LLM ``control_loop`` body."""
        local_utils = utils if utils is not None else self.utils
        name = component['name'].replace('"', '')
        other_includes = list(self._package_other_codes_hpp_includes)
        header_code = tmpl.render_component_header_hpp(
            self.template_dir, component,
            node_name=node_name,
            package_name=package_name,
            shared_vars=shared_vars,
            extra_includes=other_includes,
        )
        logic_prompt = self._build_component_state_logic_prompt(
            component,
            header_contract=header_code,
            node_name=node_name,
            shared_state_context=shared_state_context,
            package_name=package_name,
        )
        mns = _cpp_model_namespace(package_name)
        logic_prompt = self._append_error_context(logic_prompt, model_ns=mns)

        source_prompt_path = os.path.join(self.prompt_dir, f"{name}_prompt.txt")
        with open(source_prompt_path, 'w', encoding='utf-8') as f:
            f.write(logic_prompt)

        # hpp(template) -> LLM(control_loop logic only) -> full .cpp(template + LLM body)
        memory_key = memory_context_name if memory_context_name else name
        source_call_kwargs = dict(
            prompt=logic_prompt,
            api_key=self.api_key,
            component_name=memory_key,
        )
        if use_memory:
            source_call_kwargs["use_memory"] = True
            source_call_kwargs["save_memory"] = save_memory
        # Fallback path: when LLM is unavailable or returns empty, keep generating a
        # compilable component shell so node/code layout stays consistent.
        if not (self.api_key or "").strip():
            llm_control_body_raw = ""
            gen_delta = _TOKEN_STATS_ZERO.copy()
            logger.warning(
                "Component %s: API key missing, using template fallback control_loop body.",
                name,
            )
        else:
            llm_control_body_raw = local_utils.call_langchain(**source_call_kwargs)
            gen_delta = _TOKEN_STATS_ZERO.copy()
            if not (llm_control_body_raw or "").strip():
                logger.warning(
                    "Component %s: empty LLM control_loop body, using template fallback body.",
                    name,
                )
                llm_control_body_raw = ""
        control_loop_inner_body = self._clean_code(llm_control_body_raw).strip()
        inner = self._indent_llm_control_body(llm_control_body_raw)

        #  code comparison after component (for the control loop inner body)
        cmp_delta = _TOKEN_STATS_ZERO.copy()
        if component is not None:
            if (self.api_key or "").strip():
                try:
                    cmp_delta = self._run_code_comparison_after_component(
                        component, control_loop_inner_body, utils=local_utils
                    )
                except Exception as e:
                    logger.warning(
                        "Component %s: code comparison skipped after failure: %s",
                        name,
                        e,
                    )
            else:
                logger.warning(
                    "Component %s: API key missing, skipping code comparison step.",
                    name,
                )

        # merge control loop code with Component template, and generate .cpp file
        source_code = tmpl.render_llm_component_cpp(
            self.template_dir, component, package_name, node_name, inner,
            shared_vars=shared_vars,
            extra_includes=other_includes,
        )

        token_stats = gen_delta.copy()
        self._add_token_stats(token_stats, cmp_delta)

        cleaned_header = self._clean_code(header_code) if header_code else None
        cleaned_source = self._clean_code(source_code) if source_code else None
        result = {
            'component_name': name,
            'header_code': cleaned_header,
            'source_code': cleaned_source,
            'control_loop_inner_body': control_loop_inner_body,
            'header_length': len(cleaned_header) if cleaned_header else 0,
            'source_length': len(cleaned_source) if cleaned_source else 0,
        }
        return result, token_stats

    @staticmethod
    def _add_token_stats(target: Dict[str, int], delta: Dict[str, int]) -> None:
        for k in ("prompt_tokens", "completion_tokens", "total_tokens"):
            target[k] += delta.get(k, 0)

    @staticmethod
    def _record_item_tokens(
        bucket: Dict[str, TokenStats], key: str, stats: Dict[str, int]
    ) -> None:
        if stats.get("total_tokens", 0) > 0:
            bucket[key] = stats.copy()

    def _persist_component_generation(
        self,
        cname: str,
        comp_result: Dict[str, Any],
        comp_stats: Dict[str, int],
        include_components_dir: str,
        src_components_dir: str,
        global_token_stats: Dict[str, int],
        code_length_records: Dict[str, Any],
        component: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Write .hpp/.cpp, merge token stats. Returns True only if both artifacts exist."""
        self._add_token_stats(global_token_stats, comp_stats)
        header_code = comp_result.get("header_code")
        source_code = comp_result.get("source_code")
        hf = os.path.join(include_components_dir, f"{cname}.hpp")
        sf = os.path.join(src_components_dir, f"{cname}.cpp")
        if header_code:
            with open(hf, "w", encoding="utf-8") as f:
                f.write(header_code)
            logger.info("Successfully written component header file: %s", hf)
        else:
            logger.warning("Failed to generate header for component %s", cname)
        if source_code:
            with open(sf, "w", encoding="utf-8") as f:
                f.write(source_code)
            logger.info("Successfully written component source file: %s", sf)
        else:
            logger.warning("Failed to generate source for component %s", cname)
        if header_code and source_code:
            code_length_records["components"][cname] = comp_result.get("header_length", 0) + comp_result.get(
                "source_length", 0
            )
            self._record_item_tokens(
                code_length_records["item_tokens"], f"component:{cname}", comp_stats
            )
            return True
        return False

    def generate_node_code(
        self, node_info: Dict[str, Any], package_name: str,
        shared_vars: Optional[List[Dict[str, str]]] = None,
    ) -> Tuple[Optional[str], TokenStats, int]:
        """
        Generate main node `src/<node>_node.cpp` from Jinja template (no LLM).

        Args:
            node_info: Node information (name, components, executor).
            package_name: ROS package name

        Returns:
            tuple: (source code, token stats all zeros, length)
        """
        node_name = node_info.get("name", "")
        try:
            if tmpl.is_device_style_node(node_info):
                baseline_cpp = tmpl.render_device_node_cpp(
                    self.template_dir, node_info, package_name
                )
                state_machine = node_info.get("state_machine", {})
                if state_machine and (state_machine.get("transitions") or state_machine.get("states")):
                    node_generated_code, token_stats = self._generate_device_node_with_llm(
                        node_info=node_info,
                        package_name=package_name,
                        baseline_cpp=baseline_cpp,
                    )
                    tpl_name = "cpp_device_node.cpp.j2 + LLM(device behavior)"
                else:
                    node_generated_code = baseline_cpp
                    token_stats = _TOKEN_STATS_ZERO.copy()
                    tpl_name = "cpp_device_node.cpp.j2"
            else:
                node_generated_code = tmpl.render_node_main_cpp(
                    self.template_dir, node_info, package_name,
                    shared_vars=shared_vars,
                )
                token_stats = _TOKEN_STATS_ZERO.copy()
                tpl_name = "cpp_node_main.cpp.j2"
            node_generated_code = self._clean_code(node_generated_code)
            if token_stats.get("total_tokens", 0) > 0:
                logger.info(
                    "Node %r: generated from %s (tokens=%s)",
                    node_name,
                    tpl_name,
                    token_stats,
                )
            return node_generated_code, token_stats, len(node_generated_code)
        except Exception as e:
            logger.error("Error rendering node template: %s", e)
            return None, _TOKEN_STATS_ZERO.copy(), 0

    @staticmethod
    def _cmake_node_targets_from_ros_nodes(
        nodes: List[Dict[str, Any]],
        extra_c_sources: Optional[List[str]] = None,
    ) -> List[Dict[str, Any]]:
        """One add_executable per logical node: main cpp + sorted component sources."""
        targets: List[Dict[str, Any]] = []
        extra = list(extra_c_sources or [])
        for node in nodes:
            node_name = (node.get("name") or "").strip()
            if not node_name:
                continue
            sources = [f"src/{node_name}_node.cpp"]
            components = node.get("components") or []
            for comp in sorted(components, key=lambda c: c.get("name", "")):
                cname = (comp.get("name") or "").strip()
                if cname:
                    sources.append(f"src/components/{cname}.cpp")
            sources.extend(extra)
            targets.append({"executable": f"{node_name}_node", "sources": sources})
        return targets

    def generate_config_files(
        self,
        package_name: str,
        nodes: List[Dict[str, Any]],
        project_dir: str,
    ) -> Dict[str, Any]:
        """Write CMakeLists.txt and package.xml from Jinja2 templates (no LLM).

        Layout matches generator output: src/<node>_node.cpp, src/components/*.cpp.
        """
        cmakelists_dir = os.path.join(project_dir, "CMakeLists.txt")
        package_xml_dir = os.path.join(project_dir, "package.xml")

        node_targets = self._cmake_node_targets_from_ros_nodes(nodes, self._package_other_codes)

        find_packages = ["ament_cmake", "rclcpp", "std_msgs", "geometry_msgs", "sensor_msgs"]
        ament_deps = ["rclcpp", "std_msgs", "geometry_msgs", "sensor_msgs"]

        cmake_text = tmpl.render_jinja(
            self.template_dir,
            "cpp_package_CMakeLists.txt.j2",
            package_name=package_name,
            cmake_minimum_version="3.22",
            find_packages=find_packages,
            ament_dependencies=ament_deps,
            node_targets=node_targets,
            cxx_c_language_sources=self._package_other_codes,
            use_ament_lint=False,
        )

        package_xml_text = tmpl.render_jinja(
            self.template_dir,
            "cpp_package_package.xml.j2",
            package_name=package_name,
            package_version="0.0.0",
            description=f"ROS 2 Jazzy package {package_name} (generated)",
            maintainer_email="your_email@example.com",
            maintainer_name="Your Name",
            license="Apache License 2.0",
            ros_dependencies=ament_deps,
            use_ament_lint=True,
        )

        cmake_code_length = len(cmake_text)
        xml_code_length = len(package_xml_text)

        with open(cmakelists_dir, "w", encoding="utf-8") as f:
            f.write(cmake_text)
        logger.info("Wrote CMakeLists.txt from template: %s", cmakelists_dir)

        with open(package_xml_dir, "w", encoding="utf-8") as f:
            f.write(package_xml_text)
        logger.info("Wrote package.xml from template: %s", package_xml_dir)

        # No LLM calls for config files — token usage is zero for this step.
        return {
            "code_length": {"CMakeLists.txt": cmake_code_length, "package.xml": xml_code_length},
        }

    @staticmethod
    def _collect_other_codes_hpp_includes(
        package_info: Dict[str, Any],
        package_name: str,
        c_src: str = "",
        self_rel: str = "",
    ) -> List[str]:
        """Peer ``#include`` lines for ``other_codes`` → ``components/.../*.hpp``.

        When ``c_src`` is set, only headers referenced by ``#include`` in that C snippet.
        Otherwise (component templates) returns all peer headers except ``self_rel``.
        """
        pkg = (package_name or "package").lower()
        blk = package_info.get("other_codes")
        if not isinstance(blk, list) or not blk:
            return []
        merged: Dict[str, str] = {}
        for it in blk:
            if isinstance(it, dict) and it.get("code_name") is not None and it.get("code") is not None:
                merged[str(it["code_name"])] = str(it["code"])
        self_norm = str(self_rel).replace("\\", "/").strip()
        peer_by_key: Dict[str, str] = {}
        for rel_key in merged:
            rel = str(rel_key).replace("\\", "/").strip()
            if ".." in rel.split("/") or rel.startswith("/") or not rel.endswith((".c", ".h")):
                continue
            if self_norm and rel == self_norm:
                continue
            stem = os.path.splitext(rel)[0]
            line = f'#include "{pkg}/components/{stem}.hpp"'
            peer_by_key[rel] = line
            peer_by_key[os.path.basename(rel)] = line
            if "/" in rel:
                peer_by_key[os.path.basename(stem)] = line
        if not (c_src or "").strip():
            return sorted(set(peer_by_key.values()))
        found: List[str] = []
        for m in re.finditer(r'^\s*#\s*include\s+[<"]([^">]+)[">]', c_src, re.M):
            inc = m.group(1).replace("\\", "/").strip().lstrip("./")
            line = peer_by_key.get(inc) or peer_by_key.get(os.path.basename(inc))
            if line:
                found.append(line)
        return sorted(set(found))

    @staticmethod
    def _apply_other_codes_header_includes(hdr: str, include_lines: List[str]) -> str:
        """Drop LLM bundled ``components/*.hpp`` includes; insert deterministic peer includes after ``#pragma once``."""
        bundled = re.compile(r'^\s*#include\s+"[^"]*/components/[^"]+"\s*$')
        kept = [ln for ln in hdr.splitlines() if not bundled.match(ln)]
        out: List[str] = []
        inserted = False
        for ln in kept:
            out.append(ln)
            if not inserted and "#pragma once" in ln:
                out.extend(include_lines)
                inserted = True
        if not inserted:
            out = ["#pragma once", *include_lines, *out]
        body = "\n".join(out).strip()
        return body + "\n" if body else ""

    def _generate_package_other_codes_if_present(
        self,
        package_info: Dict[str, Any],
        include_components_dir: str,
        src_dir: str,
        package_name: str,
        global_token_stats: Dict[str, int],
        code_length_records: Dict[str, Any],
        incremental_components_only: bool,
        only_other_codes: Optional[set] = None,
    ) -> None:
        """LLM: ``other_codes`` C → C++; ``.c`` → ``.hpp`` + ``.cpp``, ``.h`` → ``.hpp`` only."""
        incremental_other_only = bool(only_other_codes)
        if (incremental_components_only and not incremental_other_only) or not (self.api_key or "").strip():
            return
        blk = package_info.get("other_codes")
        if not isinstance(blk, list) or not blk:
            return
        merged: Dict[str, str] = {}
        for it in blk:
            if isinstance(it, dict) and it.get("code_name") is not None and it.get("code") is not None:
                merged[str(it["code_name"])] = str(it["code"])
        if not merged:
            return
        model_ns = _cpp_model_namespace(package_name)
        max_chars = 14000
        for rel_key, c_src in sorted(merged.items()):
            if not c_src.strip():
                continue
            rel = str(rel_key).replace("\\", "/").strip()
            if only_other_codes and rel not in only_other_codes:
                continue
            if ".." in rel.split("/") or rel.startswith("/") or not rel.endswith((".c", ".h")):
                continue
            b = os.path.splitext(rel)[0]
            h_rel, c_rel = b + ".hpp", b + ".cpp"
            header_only = rel.endswith(".h")
            snippet = c_src.strip()
            if len(snippet) > max_chars:
                snippet = snippet[:max_chars] + "\n/* ... truncated ... */\n"
            pkg_h_rel = f"{package_name}/components/{h_rel}"
            struct_note = (
                " Do NOT `#include` any `*.h` in the header; peer `.hpp` deps are injected automatically."
                " Do NOT define `struct`/`class`/`typedef` in `.hpp` unless this C snippet defines that type at file scope;"
                " types only used (e.g. in parameters) come from peer headers—omit duplicate definitions."
            )
            if header_only:
                prompt = (
                    "Role: You are an expert C++17 developer and system integration engineer.\n"
                    "Task: Convert the provided C header snippet into a C++17 header only (declarations, no .cpp).\n"
                    f'JSON only: {{"header":"..."}}.\n'
                    '- JSON string values: use spaces for alignment only; no Tab characters (literal \\t breaks parsing).\n'
                    f'- "header": `{pkg_h_rel}` (`#pragma once` first).{struct_note} '
                    f'Free function declarations inside `namespace {model_ns} {{ ... }}`.\n'
                    "- Do NOT emit a separate source file; declarations only.\n"
                    "- Globals (hard): declare a variable in the header ONLY if this snippet defines it at file scope.\n"
                    "- Types (hard): emit only types defined at file scope in this snippet.\n"
                    f"C header ({rel!r}):\n---\n{snippet}\n---"
                )
            else:
                prompt = (
                    "Role: You are an expert C++17 developer and system integration engineer.\n"
                    "Task: Convert the provided C source code snippet into compliant C++17 code,"
                    f'JSON only: {{"header":"...","source":"..."}}. Convert the following C code to C++17.\n'
                    '- JSON string values: use spaces for alignment only; no Tab characters (literal \\t breaks parsing).\n'
                    f'- "header": `{pkg_h_rel}` (`#pragma once` first).{struct_note} '
                    f'Free function declarations inside `namespace {model_ns} {{ ... }}`.\n'
                    f'- "source": `{c_rel}`; first line `#include "{pkg_h_rel}"`; definitions inside `namespace {model_ns} {{ ... }}`.\n'
                    "- `printf`/macro formats: if the C pattern relies on `FMT` as `#define` glued to string literals, keep that `#define` or rewrite to one literal format string.\n"
                    "- Declarations must match definitions (same names).\n"
                    "- Globals (hard): declare or define a variable in `.hpp`/`.cpp` ONLY if this C snippet defines it at "
                    "file scope (`float x = …`, `const float y = …`, `#define Z`). Symbols only used inside functions "
                    "must not appear as `extern`/globals in the output.\n"
                    "- Types (hard): same rule for `struct`/`class`/`typedef`/`enum`—emit only types defined at file scope "
                    "in this C snippet; if the source only references a type (e.g. `struct foo_t *p`), do not redefine `foo_t`.\n"
                    "- Math (hard): `<math.h>` / `#include <math.h>` (incl. under `#ifdef MATH`) → `#include <cmath>`; "
                    "use `std::pow`, `std::cos`, `std::sin`, `std::atan`, `std::sqrt`. "
                    "No `#define cos` / `#define sin` (breaks `std::cos`). "
                    "No typedef `uint64_t`/`uint32_t`/... (collides with `<cstdint>`); use `std::uint64_t` or another name.\n"
                    "- C++ output must NOT use `__attribute__((weak))` or `__weak__`.\n"
                    f"C source ({rel!r}):\n---\n{snippet}\n---"
                )
            prompt = self._append_error_context(prompt, model_ns=model_ns)
            oc_prompt_path = os.path.join(
                self.prompt_dir, f"other_codes_{rel.replace('/', '_')}_prompt.txt"
            )
            with open(oc_prompt_path, "w", encoding="utf-8") as f:
                f.write(prompt)
            raw = self.utils.call_langchain(
                prompt=prompt,
                api_key=self.api_key,
                component_name=f"other_{rel.replace('/', '_')}",
                save_memory=False,
            )
            delta = _TOKEN_STATS_ZERO.copy()
            self._add_token_stats(global_token_stats, delta)
            self._record_item_tokens(
                code_length_records["item_tokens"], f"other_codes:{rel}", delta
            )
            parsed = _try_parse_json_object(raw or "")
            hdr, src = "", ""
            if isinstance(parsed, dict):
                hdr = self._clean_code((parsed.get("header") or "").strip())
                src = self._clean_code((parsed.get("source") or "").strip())
            if (not isinstance(parsed, dict)) or (not hdr) or (not header_only and not src):
                _raw_path = os.path.join(
                    self.prompt_dir, f"other_codes_{rel.replace('/', '_')}_llm_raw.txt"
                )
                with open(_raw_path, "w", encoding="utf-8") as f:
                    f.write(raw or "")
                _reason = (
                    "JSON parse failed"
                    if not isinstance(parsed, dict)
                    else ("empty header after clean" if header_only else "empty header or source after clean")
                )
                logger.warning(
                    "other_codes %s: %s, skipping; raw LLM output written to %s",
                    rel,
                    _reason,
                    _raw_path,
                )
                continue
            peer_includes = self._collect_other_codes_hpp_includes(
                package_info, package_name, c_src=c_src, self_rel=rel
            )
            hdr = self._apply_other_codes_header_includes(hdr, peer_includes)
            h_path = os.path.join(include_components_dir, *h_rel.split("/"))
            os.makedirs(os.path.dirname(h_path), exist_ok=True)
            with open(h_path, "w", encoding="utf-8") as f:
                f.write(hdr)
            if header_only:
                code_length_records.setdefault("other_codes", {})[rel] = {"header": len(hdr)}
                logger.info("other_codes %s -> %s (header only)", rel, h_path)
                continue
            if src.strip():
                src = src.strip() + "\n"
            c_path = os.path.join(src_dir, *c_rel.split("/"))
            os.makedirs(os.path.dirname(c_path), exist_ok=True)
            with open(c_path, "w", encoding="utf-8") as f:
                f.write(src)
            self._package_other_codes.append(f"src/{c_rel}".replace("\\", "/"))
            code_length_records.setdefault("other_codes", {})[rel] = {"header": len(hdr), "source": len(src)}
            logger.info("other_codes %s -> %s", rel, c_path)

    def _skip_package_after_permission_check(self, project_dir: str) -> bool:
        """Original behavior: skip the package when not writable and chmod/icacls reports success."""
        if os.access(project_dir, os.W_OK):
            return False
        return bool(self.utils.grant_file_permissions(project_dir))

    def _ensure_package_source_layout(self, project_dir: str, package_name_lower: str) -> Dict[str, str]:
        """Create include/src trees; return absolute paths used by codegen."""
        include_pkg_dir = os.path.join(project_dir, "include", package_name_lower)
        include_components_dir = os.path.join(include_pkg_dir, "components")
        src_dir = os.path.join(project_dir, "src")
        src_components_dir = os.path.join(src_dir, "components")
        for d in (include_components_dir, src_components_dir, src_dir):
            os.makedirs(d, exist_ok=True)
        return {
            "include_components_dir": include_components_dir,
            "src_components_dir": src_components_dir,
            "src_dir": src_dir,
        }

    @staticmethod
    def _components_for_node_pass(
        node_info: Dict[str, Any], only_components_set: Set[str]
    ) -> List[Dict[str, Any]]:
        """Components to generate for this pass: optionally filtered, then sorted by name."""
        raw = node_info.get("components", [])
        if only_components_set:
            raw = [c for c in raw if c.get("name", "") in only_components_set]
        if not raw:
            return []
        return sorted(raw, key=lambda c: c["name"])

    def _generate_node_components(
        self,
        components: List[Dict[str, Any]],
        package_name: str,
        node_name: str,
        include_components_dir: str,
        src_components_dir: str,
        global_token_stats: Dict[str, int],
        code_length_records: Dict[str, Any],
        shared_state_context: str = "",
        shared_vars: Optional[List[Dict[str, str]]] = None,
    ) -> bool:
        """First component in-process; remaining in a process pool. False = anchor failed, skip node shell."""
        first = components[0]
        anchor = (first.get("name") or "").replace('"', "")
        try:
            res, stats = _generate_single_component_with_retry(
                self,
                component=first,
                package_name=package_name,
                use_memory=True,
                utils=self.utils,
                memory_context_name=anchor,
                save_memory=True,
                node_name=node_name,
                shared_state_context=shared_state_context,
                shared_vars=shared_vars,
            )
        except Exception as e:
            logger.error("Failed to generate first component %s after retries: %s", anchor, e)
            return False
        if not self._persist_component_generation(
            anchor,
            res,
            stats,
            include_components_dir,
            src_components_dir,
            global_token_stats,
            code_length_records,
            component=first,
        ):
            return False

        rest = components[1:]
        if not rest:
            return True

        base = (self.ros_file, self.output_dir, self.api_key, self.error_context)
        hpp_includes = list(self._package_other_codes_hpp_includes)
        # Use "spawn" so workers do not inherit fork-copied locks (logging, HTTP clients, etc.),
        # which can deadlock after the CLI or other imports have initialized the interpreter.
        with ProcessPoolExecutor(max_workers=2, mp_context=mp.get_context("spawn")) as pool:
            futures = {
                pool.submit(
                    _parallel_single_component_worker,
                    *base,
                    comp,
                    package_name,
                    node_name,
                    anchor,
                    shared_state_context,
                    shared_vars or [],
                    hpp_includes,
                ): comp
                for comp in rest
            }
            failed: List[str] = []
            for fut in as_completed(futures):
                comp = futures[fut]
                cn = (comp.get("name") or "").replace('"', "")
                try:
                    res, stats = fut.result()
                except Exception as e:
                    logger.error(
                        "Failed to generate component %s after retries: %s", cn, e
                    )
                    failed.append(cn)
                    continue
                self._persist_component_generation(
                    cn,
                    res,
                    stats,
                    include_components_dir,
                    src_components_dir,
                    global_token_stats,
                    code_length_records,
                    component=comp,
                )
            if failed:
                logger.error(
                    "Node %s: components not generated: %s", node_name, ", ".join(failed)
                )
                return False
        return True

    def _write_template_main_node(
        self,
        node_info: Dict[str, Any],
        package_name: str,
        src_dir: str,
        global_token_stats: Dict[str, int],
        code_length_records: Dict[str, Any],
        shared_vars: Optional[List[Dict[str, str]]] = None,
    ) -> None:
        """Emit main node from template (no LLM); merge token stats (zeros)."""
        node_name = node_info.get("name", "")
        node_file = os.path.join(src_dir, f"{node_name}_node.cpp")

        parallel_token_stats: Dict[str, int] = {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}

        try:
            node_code, node_token_stats, node_code_length = self.generate_node_code(
                node_info, package_name, shared_vars=shared_vars,
            )
            if node_code:
                with open(node_file, "w", encoding="utf-8") as f:
                    f.write(node_code)
                logger.info("Successfully written to file: %s", node_file)
                code_length_records["nodes"][node_name] = node_code_length
            if node_token_stats.get("total_tokens", 0) > 0:
                self._record_item_tokens(
                    code_length_records["item_tokens"], f"node:{node_name}", node_token_stats
                )
            self._add_token_stats(parallel_token_stats, node_token_stats)
        except Exception as e:
            logger.error("Error generating node code: %s", e)

        if parallel_token_stats["total_tokens"] > 0:
            self._add_token_stats(global_token_stats, parallel_token_stats)
            logger.info(
                "[Node LLM] Input Tokens: %s, Output Tokens: %s, Total Tokens: %s",
                parallel_token_stats["prompt_tokens"],
                parallel_token_stats["completion_tokens"],
                parallel_token_stats["total_tokens"],
            )

    def _record_config_code_lengths(
        self, result: Dict[str, Any], code_length_records: Dict[str, Any]
    ) -> None:
        for file_name, code_length in result["code_length"].items():
            if file_name not in ("CMakeLists.txt", "package.xml"):
                continue
            if "config" not in code_length_records:
                code_length_records["config"] = {}
            code_length_records["config"][file_name] = code_length

    def generate_code(
        self,
        generate_config_files: bool = True,
        only_components: Optional[List[str]] = None,
        only_other_codes: Optional[List[str]] = None,
        only_nodes: Optional[List[str]] = None,
    ) -> None:
        """Generate all node code.

        Main node sources are always from templates (no LLM). Incremental repair
        is supported via ``only_components``, ``only_other_codes``, or ``only_nodes``.

        Args:
            generate_config_files: Whether to emit CMakeLists.txt and package.xml (False on incremental runs)
            only_components: If set, only regenerate these components (skip main node in that run)
            only_other_codes: If set, only regenerate these other_codes entries (code_name paths)
            only_nodes: If set, only regenerate these node files (device-style nodes only; skips components and config)
        """
        start_time = time.time()
        logger.info("Code generation start time: %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

        os.makedirs(self.output_dir, exist_ok=True)

        global_token_stats = {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}
        code_length_records: Dict[str, Any] = {"components": {}, "nodes": {}, "item_tokens": {}}

        ros_packages = self.utils._extract_ros_packages(self.ros_architecture)
        only_components_set = {c for c in (only_components or []) if c}
        only_other_codes_set = {
            str(c).replace("\\", "/") for c in (only_other_codes or []) if c
        }
        only_nodes_set = {n for n in (only_nodes or []) if n}
        incremental_components_only = bool(only_components_set)
        incremental_other_codes_only = bool(only_other_codes_set)
        incremental_nodes_only = bool(only_nodes_set)

        for package_name_raw, package_info in ros_packages.items():
            project_dir = os.path.join(self.output_dir, package_name_raw)
            os.makedirs(project_dir, exist_ok=True)
            package_name = package_name_raw.lower()

            if self._skip_package_after_permission_check(project_dir):
                continue

            layout = self._ensure_package_source_layout(project_dir, package_name)
            include_components_dir = layout["include_components_dir"]
            src_components_dir = layout["src_components_dir"]
            src_dir = layout["src_dir"]

            self._package_other_codes = []
            self._package_other_codes_hpp_includes = self._collect_other_codes_hpp_includes(
                package_info, package_name
            )
            self._generate_package_other_codes_if_present(
                package_info,
                include_components_dir,
                src_dir,
                package_name,
                global_token_stats,
                code_length_records,
                incremental_components_only,
                only_other_codes=only_other_codes_set or None,
            )

            if incremental_other_codes_only:
                continue

            nodes = package_info.get("nodes", [])
            if not nodes:
                continue

            shared_state_context, sc_stats = self._package_shared_state_context(
                package_info, package_name, incremental_components_only
            )
            self._add_token_stats(global_token_stats, sc_stats)
            self._record_item_tokens(
                code_length_records["item_tokens"], f"shared_state:{package_name}", sc_stats
            )
            shared_vars = self._load_shared_vars(package_name)
            self._current_shared_vars = shared_vars  # available to prompt builder

            # Write per-package shared state header (idempotent)
            if shared_vars:
                shared_hpp_path = os.path.join(
                    project_dir, "include", package_name, "shared_sim_state.hpp"
                )
                os.makedirs(os.path.dirname(shared_hpp_path), exist_ok=True)
                with open(shared_hpp_path, "w", encoding="utf-8") as f:
                    f.write(tmpl.render_shared_sim_state_hpp(
                        self.template_dir, package_name, shared_vars
                    ))
                logger.info("Wrote shared sim state header: %s", shared_hpp_path)

            for node_info in nodes:
                node_name = node_info.get("name", "")

                # only_nodes mode: only regenerate the specified device-style nodes, skip everything else.
                if incremental_nodes_only:
                    if node_name in only_nodes_set and tmpl.is_device_style_node(node_info):
                        self._write_template_main_node(
                            node_info,
                            package_name,
                            src_dir,
                            global_token_stats,
                            code_length_records,
                            shared_vars=shared_vars,
                        )
                    continue

                components = self._components_for_node_pass(node_info, only_components_set)
                if components:
                    ok = self._generate_node_components(
                        components,
                        package_name,
                        node_name,
                        include_components_dir,
                        src_components_dir,
                        global_token_stats,
                        code_length_records,
                        shared_state_context=shared_state_context,
                        shared_vars=shared_vars,
                    )
                    if not ok:
                        continue
                    if incremental_components_only:
                        continue

                    self._write_template_main_node(
                        node_info,
                        package_name,
                        src_dir,
                        global_token_stats,
                        code_length_records,
                        shared_vars=shared_vars,
                    )
                elif tmpl.is_device_style_node(node_info) and not incremental_components_only:
                    self._write_template_main_node(
                        node_info,
                        package_name,
                        src_dir,
                        global_token_stats,
                        code_length_records,
                        shared_vars=shared_vars,
                    )

            if generate_config_files and not incremental_nodes_only:
                result = self.generate_config_files(package_name, nodes, project_dir)
                self._record_config_code_lengths(result, code_length_records)

        token_stats_file = os.path.join(self.prompt_dir, "token_usage_stats.json")
        end_time = time.time()
        total_time = end_time - start_time
        logger.info("Code generation completion time: %s", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        logger.info(
            "[Total Time Consumption Stats] Total time: %.2f seconds (%.2f minutes)",
            total_time,
            total_time / 60,
        )

        final_stats = {
            "token_stats": global_token_stats,
            "token_usage": code_length_records.pop("item_tokens", {}),
            "code_lengths": code_length_records,
            "time_stats": {
                "start_time": datetime.fromtimestamp(start_time).strftime("%Y-%m-%d %H:%M:%S"),
                "end_time": datetime.fromtimestamp(end_time).strftime("%Y-%m-%d %H:%M:%S"),
                "total_time_seconds": total_time,
                "total_time_minutes": total_time / 60,
            },
        }

        try:
            with open(token_stats_file, "w", encoding="utf-8") as f:
                json.dump(final_stats, f, ensure_ascii=False, indent=2)
            logger.info("Successfully saved token statistics and code lengths to file: %s", token_stats_file)
        except Exception as e:
            logger.error("Error saving token statistics and code lengths: %s", e)


def _generate_single_component_with_retry(
    gen: "ROSCodeGenerator",
    *,
    component: Dict[str, Any],
    package_name: str,
    use_memory: bool,
    utils: ROSGeneratorUtils,
    memory_context_name: Optional[str] = None,
    save_memory: bool = True,
    node_name: str = "",
    shared_state_context: str = "",
    shared_vars: Optional[List[Dict[str, str]]] = None,
) -> Tuple[Dict[str, Any], TokenStats]:
    """Retry whole component codegen when LLM returns empty or invoke fails."""
    cname = (component.get("name") or "").replace('"', "")
    last_err: Optional[Exception] = None
    for attempt in range(1, LLM_CALL_MAX_ATTEMPTS + 1):
        try:
            return gen._generate_single_component_hpp_cpp(
                component=component,
                package_name=package_name,
                use_memory=use_memory,
                utils=utils,
                memory_context_name=memory_context_name,
                save_memory=save_memory,
                node_name=node_name,
                shared_state_context=shared_state_context,
                shared_vars=shared_vars,
            )
        except Exception as e:
            last_err = e
            logger.warning(
                "Component %s codegen failed (attempt %s/%s): %s",
                cname,
                attempt,
                LLM_CALL_MAX_ATTEMPTS,
                e,
            )
            if attempt < LLM_CALL_MAX_ATTEMPTS:
                time.sleep(LLM_CALL_RETRY_SLEEP_S)
    raise last_err  # type: ignore[misc]


def _parallel_single_component_worker(
    ros_file: str,
    output_dir: str,
    api_key: Optional[str],
    error_context: str,
    component: Dict[str, Any],
    package_name: str,
    node_name: str,
    memory_anchor: str,
    shared_state_context: str = "",
    shared_vars: Optional[List[Dict[str, str]]] = None,
    other_codes_hpp_includes: Optional[List[str]] = None,
) -> Tuple[Dict[str, Any], TokenStats]:
    """Run one component in a child process; conversation memory is keyed on ``memory_anchor``."""
    wname = (component.get("name") or "").replace('"', "")
    logger.info("Parallel worker started for component %s (memory anchor=%s)", wname, memory_anchor)
    gen = ROSCodeGenerator(ros_file, output_dir, api_key, error_context=error_context or "")
    gen._package_other_codes_hpp_includes = list(other_codes_hpp_includes or [])
    gen._current_shared_vars = shared_vars or []
    temp_utils = ROSGeneratorUtils(output_dir)
    return _generate_single_component_with_retry(
        gen,
        component=component,
        package_name=package_name,
        use_memory=True,
        utils=temp_utils,
        memory_context_name=memory_anchor,
        save_memory=True,
        node_name=node_name,
        shared_state_context=shared_state_context,
        shared_vars=shared_vars or [],
    )


def main():
    """Main function"""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Generate ROS implementation code using large language models based on ROS architecture')
    parser.add_argument('-r', '--ros', required=True, help='Path to ROS architecture file')
    parser.add_argument('-o', '--output', default='./ros_generated', help='Output directory')
    parser.add_argument('-k', '--key', help='Large language model API key', default='')
    parser.add_argument(
        '--only-components',
        default='',
        help='Comma-separated component names to regenerate (LLM body only); skips CMakeLists refresh',
    )
    parser.add_argument(
        '--only-other-codes',
        default='',
        help='Comma-separated other_codes code_name paths to regenerate (e.g. common/app2_code.c)',
    )
    parser.add_argument(
        '--only-nodes',
        default='',
        help='Comma-separated device-style node names to regenerate (e.g. lgs,dps); skips components and config',
    )
    parser.add_argument('--error_context', default='', help='Runtime error context appended to regeneration prompts')
    args = parser.parse_args()
    
    # Create code generator
    generator = ROSCodeGenerator(args.ros, args.output, args.key, error_context=args.error_context)
    
    # Generate code and documentation for all nodes
    only_components = [x.strip() for x in args.only_components.split(',') if x.strip()]
    only_other_codes = [x.strip() for x in args.only_other_codes.split(',') if x.strip()]
    only_nodes = [x.strip() for x in args.only_nodes.split(',') if x.strip()]
    incremental = bool(only_components or only_other_codes or only_nodes)
    generator.generate_code(
        generate_config_files=not incremental,
        only_components=only_components or None,
        only_other_codes=only_other_codes or None,
        only_nodes=only_nodes or None,
    )
    logger.info("Code generation completed, output directory: %s", args.output)

if __name__ == "__main__":
    main()