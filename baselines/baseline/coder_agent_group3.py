#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import logging
import os
import re
import sys
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
import coder_template as tmpl
from ros_generator_utils import ROSGeneratorUtils
from coder_agent import (
    ROSCodeGenerator as MainCodeGenerator,
    _cpp_model_namespace,
    _prepare_source_text,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)
_TOKEN_STATS_ZERO = {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}


def read_json_file(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_arch_packages(path: str) -> List[Dict[str, Any]]:
    arch = read_json_file(path)
    if isinstance(arch, dict) and isinstance(arch.get("ROSPackages"), list):
        return arch["ROSPackages"]
    if isinstance(arch, list):
        return arch
    raise ValueError("Architecture JSON must be a package list or contain ROSPackages")


def clean_llm_code(raw: Optional[str]) -> str:
    """Align with coder_agent.ROSCodeGenerator._clean_code."""
    if not raw:
        return ""
    return (
        raw.replace("</think>", "")
        .replace("```cpp", "")
        .replace("```cmake", "")
        .replace("```xml", "")
        .replace("```", "")
        .replace("\r", "")
        .strip()
    )


def _try_parse_json_object(text: Optional[str]) -> Optional[Dict[str, Any]]:
    if not text:
        return None
    cleaned = str(text).strip()
    try:
        parsed = json.loads(cleaned)
        return parsed if isinstance(parsed, dict) else None
    except Exception:
        pass
    m = re.search(r"\{[\s\S]*\}", cleaned)
    if not m:
        return None
    try:
        parsed = json.loads(m.group(0))
        return parsed if isinstance(parsed, dict) else None
    except Exception:
        return None


class ROSCodeGenerator:
    """RQ3 group3: template CMake/package.xml; LLM for components, nodes, devices, shared state, other_codes."""

    def __init__(
        self,
        arch_file: str,
        output_dir: str,
        api_key: Optional[str] = None,
        error_context: str = "",
    ):
        self.arch_file = arch_file
        self.output_dir = output_dir
        self.api_key = api_key
        self.error_context = (error_context or "").strip()
        self.prompt_dir = os.path.join(output_dir, "prompts")
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.prompt_dir, exist_ok=True)
        self.ros_info = os.path.join(output_dir, "ros_info")
        os.makedirs(self.ros_info, exist_ok=True)
        self.utils = ROSGeneratorUtils(self.output_dir)

        self.statistics = {
            "total_tokens": 0,
            "prompt_tokens": 0,
            "completion_tokens": 0,
            "component_stats": {},
            "node_stats": {},
            "config_stats": {},
            "generation_time": 0.0,
        }
        self.template_dir = os.path.join(PROJECT_ROOT, "templates")
        self._pkg_other_codes: Dict[str, List[str]] = {}
        self._pkg_other_codes_includes: Dict[str, List[str]] = {}

    # Bind coder_agent comparison helpers (state_machine + subprograms only).
    _package_behavior_for_comparison = MainCodeGenerator._package_behavior_for_comparison
    _build_code_comparison_prompt = MainCodeGenerator._build_code_comparison_prompt
    _component_has_behavior_for_comparison = MainCodeGenerator.__dict__[
        "_component_has_behavior_for_comparison"
    ]
    _truncate_for_comparison_cpp = MainCodeGenerator.__dict__["_truncate_for_comparison_cpp"]

    def _maybe_run_code_comparison(self, component: Dict[str, Any], cpp_code: str) -> None:
        if not cpp_code.strip():
            return
        cname = self._normalized_name(component.get("name"), "component")
        try:
            MainCodeGenerator._run_code_comparison_after_component(
                self, component, cpp_code, self.utils
            )
        except Exception as exc:
            logger.warning("Component %s: code comparison skipped after failure: %s", cname, exc)

    @staticmethod
    def _format_shared_vars_block(parsed: Optional[Dict[str, Any]]) -> str:
        if not isinstance(parsed, dict):
            return ""
        vars_block = parsed.get("shared_state_variables")
        if vars_block is None:
            return ""
        return json.dumps({"shared_state_variables": vars_block}, ensure_ascii=False, indent=2)

    def _write_shared_state_artifact(
        self,
        out_path: str,
        prompt_block: str,
        response: Optional[str],
        parsed: Optional[Dict[str, Any]],
    ) -> None:
        try:
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(
                    {"prompt_block": prompt_block, "raw_response": response, "parsed": parsed},
                    f,
                    ensure_ascii=False,
                    indent=2,
                )
            logger.info("Wrote shared_code LLM analysis: %s", out_path)
        except Exception as e:
            logger.error("Failed to write %s: %s", out_path, e)

    def _load_shared_state_prompt_block(self, path: str) -> str:
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

    def _run_shared_code_llm(
        self, shared_block: Dict[str, Any], package_name: str, out_path: str
    ) -> Tuple[str, Dict[str, int]]:
        src_raw = shared_block.get("source_text")
        if src_raw is None:
            src = ""
        elif isinstance(src_raw, list):
            src = "\n\n".join(str(x or "") for x in src_raw)
        else:
            src = str(src_raw)

        prompt = f"""You should analyze shared source from an AADL model: several subprograms in one process share this code (globals, mutex, etc.).
Source text: {src}
Task: Read the code and list identifiers that are **shared mutable state** the whole node must use once.
For each variable also record its **initial value** as declared in the source code.
Output: **One JSON object only** (valid JSON, no markdown fences, no other text), exactly this form:
{{
  "shared_state_variables": [{{"name": "<identifier>", "initial_value": <integer>, "rationale": "<one short English sentence why it is shared state>"}}]
}}
"""
        try:
            response = self.utils.call_langchain(
                prompt=prompt,
                api_key=self.api_key,
                component_name=f"{package_name}_shared_state_analysis",
                use_memory=False,
                load_memory=False,
                save_memory=False,
            )
            token_stats = {
                "prompt_tokens": self.utils.total_prompt_tokens,
                "completion_tokens": self.utils.total_completion_tokens,
                "total_tokens": self.utils.total_tokens,
            }
            parsed = _try_parse_json_object(response)
            block = self._format_shared_vars_block(parsed)
            self._write_shared_state_artifact(out_path, block, response, parsed)
            return block, token_stats
        except Exception as e:
            logger.error("shared_code analysis failed for %s: %s", package_name, e)
            return "", _TOKEN_STATS_ZERO.copy()

    def _package_shared_state_context(
        self, package_info: Dict[str, Any], package_name: str, incremental_only: bool
    ) -> Tuple[str, Dict[str, int]]:
        shared_code = package_info.get("shared_code")
        if not shared_code or not isinstance(shared_code, dict):
            return "", _TOKEN_STATS_ZERO.copy()
        out_path = os.path.join(self.prompt_dir, f"{package_name}_shared_state_analysis.json")
        if incremental_only and os.path.exists(out_path):
            loaded = self._load_shared_state_prompt_block(out_path)
            if loaded:
                return loaded, _TOKEN_STATS_ZERO.copy()
        return self._run_shared_code_llm(shared_code, package_name, out_path)

    def _append_error_context(self, prompt: str, model_ns: Optional[str] = None) -> str:
        if not self.error_context:
            return prompt
        if model_ns:
            scope_rule = (
                f"- Bundled calls: `{model_ns}::<name>` as in headers (not `::` alone when names clash). "
                "No std-clashing typedefs/macros (e.g. `uint64_t`, `#define pow`); add `#include` instead of inventing `extern`."
            )
        else:
            scope_rule = (
                '- Global scope (hard): if the compiler reports "no matching function" or "was not declared in this scope" '
                "for a free function call, prefix ONLY that call with `::` to resolve it in the global namespace. "
                "Do not rename, refactor, or change any other call sites."
            )
        prompt += f"""
# BUG FIX CONTEXT FROM DYNAMIC TEST
You are regenerating code to fix runtime errors. Follow these constraints strictly:
- Apply MINIMAL CHANGE strategy: modify ONLY code directly related to the listed errors.
- DO NOT refactor, rename, reorder, or rewrite unrelated logic.
- DO NOT change public interfaces, topic names/types, callback signatures, class names, file structure, or unrelated behaviors.
- Keep non-error code paths functionally identical.
- If multiple fixes are possible, choose the smallest local patch that resolves the error.
- Preserve existing topic/interface contracts unless explicitly contradicted by error context.
- Logger rule (hard): if a component has `rclcpp::Logger logger_;`, initialize it in the constructor initializer list with `logger_(node_->get_logger())` or `logger_(node->get_logger())`. Never default-construct it and never assign `logger_ = ...` inside the constructor body.
- External header rule (hard): never add `#include` directives for external headers that are not present in the generated ROS package. If a missing subprogram/helper caused the error, implement the smallest equivalent local C++ logic inside the regenerated component instead of inventing an external header or undeclared function dependency.
- Component header include path (hard): component headers live at `include/<package>/components/<name>.hpp`. In `.cpp` and node files use exactly `#include "<package>/components/<name>.hpp"`.
- QoS (Jazzy, hard): build from `rclcpp::QoS` using only valid chain methods — `.reliable()` or `.best_effort()`; for durability use `.transient_local()` or `.durability_volatile()`. Never `.volatile()`, `volatile_durability`, `durability_transient_local()`, or other non-existent `rclcpp::QoS` members. Map JSON `reliability`/`durability` from the architecture contract; do not invent QoS policies.
- Callback group on pub/sub (hard): do not pass `CallbackGroup::SharedPtr` as a bare argument to `create_publisher`/`create_subscription`. Use `rclcpp::PublisherOptions` / `rclcpp::SubscriptionOptions` and set `options.callback_group`.
- SharedSimState wiring (hard): when this package uses SharedSimState, every component constructor is `(node, cb_group, sim_state)` and the main node must pass `sim_state` to ALL components — not only those that read/write shared fields.
- Globals (hard): declare or define a variable ONLY if the original source defines it at file scope (`float x = …`, `const float y = …`, `#define Z`). Symbols only used inside functions must not appear as `extern`.
- Types (hard): emit `struct`/`class`/`typedef`/`enum` ONLY if the original source defines that type at file scope; if the source only references a type (e.g. `struct foo_t *p`), do not redefine `foo_t`.
{scope_rule}
- AADL/BA + logs (hard): `event port` — arrival is the event (reject fixes that force `data == "true"` unless the model defines booleans; `"timeout"` / numeric strings count). `on dispatch <input>` may reset without publish; timed dispatch with `<port>!` publishes once on that output.
Prefer recognizing arrivals via non-null `*_cache_` (check before `->`) rather than string equality hacks. Log entity prefix (hard): every `Received`/`Published`/`State transition` line MUST start with `[<node>.<component>]` (e.g. `[main.analyse] Received from_receiver: ...`) so runtime_analysis attributes events to the thread, not the process node logger.
- Class/name consistency (hard): component class name in `.hpp` and every `std::make_shared` reference in node `.cpp` must use the same lowercase spelling as architecture `name`; align header and node together — do not flip casing between repair rounds.
- Periodic publish (hard): when `outputs` is non-empty, `control_loop()` must publish on every output port with `Published <port>:` — empty or no-op `control_loop()` is invalid.
- Timer (hard): `node_->create_wall_timer(period, std::bind(&Class::control_loop, this), callback_group_)`; period from `properties.period` when present; pass `callback_group_` as the third argument directly.
Run-time error context:
{self.error_context}
"""
        return prompt

    @staticmethod
    def _normalized_name(name: Any, default: str) -> str:
        text = str(name or default).strip()
        return text.lower() if text else default

    @staticmethod
    def _parse_shared_vars_from_context(shared_state_context: str) -> List[Dict[str, Any]]:
        if not (shared_state_context or "").strip():
            return []
        try:
            data = json.loads(shared_state_context)
        except json.JSONDecodeError:
            return []
        vars_list = data.get("shared_state_variables") if isinstance(data, dict) else None
        return [v for v in (vars_list or []) if isinstance(v, dict) and v.get("name")]

    def _extract_packages(self, arch: Any) -> List[Dict[str, Any]]:
        # Control-group parser output is often a top-level list.
        if isinstance(arch, list):
            return arch
        if isinstance(arch, dict) and isinstance(arch.get("ROSPackages"), list):
            return arch["ROSPackages"]
        if isinstance(arch, dict) and isinstance(arch.get("systems"), list):
            return arch["systems"]
        raise ValueError(
            "Unsupported architecture JSON: expected top-level list, or dict key `ROSPackages` / `systems`."
        )

    def _call_llm(
        self,
        prompt_name: str,
        prompt: str,
        key: str,
        *,
        append_error_context: bool = True,
    ) -> Tuple[str, Dict[str, int]]:
        if append_error_context:
            prompt = self._append_error_context(prompt)
        prompt_file = os.path.join(self.prompt_dir, f"{prompt_name}.txt")
        try:
            with open(prompt_file, "w", encoding="utf-8") as f:
                f.write(prompt)
        except OSError as e:
            logger.error("Failed to write prompt %s: %s", prompt_file, e)

        try:
            code = self.utils.call_langchain(
                prompt=prompt,
                api_key=self.api_key,
                component_name=key,
                use_memory=False,
                load_memory=False,
                save_memory=False,
            )
            token_stats = {
                "prompt_tokens": self.utils.total_prompt_tokens,
                "completion_tokens": self.utils.total_completion_tokens,
                "total_tokens": self.utils.total_tokens,
            }
            self.statistics["prompt_tokens"] += token_stats["prompt_tokens"]
            self.statistics["completion_tokens"] += token_stats["completion_tokens"]
            self.statistics["total_tokens"] += token_stats["total_tokens"]
            return clean_llm_code(code), token_stats
        except Exception as e:
            logger.error("LLM generation failed for %s: %s", key, e)
            return "", _TOKEN_STATS_ZERO.copy()

    def _build_other_codes_prompt_block(self, package_name: str) -> str:
        """Build bundled other_codes context for component prompts (mirrors coder_agent bundled_call)."""
        includes = self._pkg_other_codes_includes.get(package_name, [])
        if not includes:
            return ""
        model_ns = _cpp_model_namespace(package_name)
        inc_block = "\n".join(includes)
        return (
            f"\n---\nBundled other_codes headers (converted C subprograms):\n{inc_block}\n"
            f"Subprogram C in Component JSON `subprograms` is reference only: inline its bound logic inside "
            f"`control_loop()`; do NOT add helper/wrapper/local functions. "
            f"Do NOT call `*_aadl` unless that exact name is declared in a bundled `.hpp` above. "
            f"When the C body calls model routines, use `{model_ns}::<name from bundled .hpp>` "
            f"(e.g. free functions in those headers, not `*_aadl`); prefer `{model_ns}::` over bare `::` "
            f"when the component class name collides with the free function.\n"
        )

    @staticmethod
    def _collect_other_code_cpp_sources(
        package_info: Dict[str, Any],
        project_dir: Optional[str] = None,
    ) -> List[str]:
        """All converted other_codes ``.cpp`` paths for CMake (``.c`` entries only)."""
        blk = package_info.get("other_codes")
        if not isinstance(blk, list):
            return []
        sources: List[str] = []
        for it in blk:
            if not isinstance(it, dict):
                continue
            rel_key = it.get("code_name")
            if rel_key is None:
                continue
            rel = str(rel_key).replace("\\", "/").strip()
            if not rel.endswith(".c"):
                continue
            stem = os.path.splitext(rel)[0]
            rel_cpp = f"src/{stem}.cpp".replace("\\", "/")
            if project_dir:
                if not os.path.isfile(os.path.join(project_dir, rel_cpp)):
                    continue
            sources.append(rel_cpp)
        return sorted(set(sources))

    def _build_other_codes_heuristic_prompt(
        self,
        package_name: str,
        rel: str,
        snippet: str,
        model_ns: str,
        pkg_h_rel: str,
        c_rel: str,
        header_only: bool = False,
    ) -> str:
        struct_note = (
            " Do NOT `#include` any `*.h` in the header; peer `.hpp` deps are injected automatically."
            " Do NOT define `struct`/`class`/`typedef` in `.hpp` unless this C snippet defines that type at file scope;"
            " types only used (e.g. in parameters) come from peer headers—omit duplicate definitions."
        )
        if header_only:
            return (
                "Role: You are an expert C++17 developer and system integration engineer.\n"
                "Task: Convert the provided C header snippet into a C++17 header only (declarations, no .cpp).\n"
                f'JSON only: {{"header":"..."}}.\n'
                "- JSON string values: use spaces for alignment only; no Tab characters (literal \\t breaks parsing).\n"
                f'- "header": `{pkg_h_rel}` (`#pragma once` first).{struct_note} '
                f"Free function declarations inside `namespace {model_ns} {{ ... }}`.\n"
                "- Do NOT emit a separate source file; declarations only.\n"
                "- Globals (hard): declare a variable in the header ONLY if this snippet defines it at file scope.\n"
                "- Types (hard): emit only types defined at file scope in this snippet.\n"
                f"C header ({rel!r}):\n---\n{snippet}\n---"
            )
        return (
            "Role: You are an expert C++17 developer and system integration engineer.\n"
            "Task: Convert the provided C source code snippet into compliant C++17 code,"
            f'JSON only: {{"header":"...","source":"..."}}. Convert the following C code to C++17.\n'
            "- JSON string values: use spaces for alignment only; no Tab characters (literal \\t breaks parsing).\n"
            f'- "header": `{pkg_h_rel}` (`#pragma once` first).{struct_note} '
            f"Free function declarations inside `namespace {model_ns} {{ ... }}`.\n"
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

    def _generate_package_other_codes_heuristic(
        self,
        package_info: Dict[str, Any],
        include_components_dir: str,
        src_dir: str,
        package_name: str,
        incremental_components_only: bool,
        other_code_sources: List[str],
        only_other_codes: Optional[set] = None,
    ) -> None:
        """Heuristic LLM: convert other_codes C snippets to C++ hpp/cpp."""
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
            prompt = self._build_other_codes_heuristic_prompt(
                package_name, rel, snippet, model_ns, pkg_h_rel, c_rel, header_only=header_only
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
                use_memory=False,
                load_memory=False,
                save_memory=False,
            )
            parsed = _try_parse_json_object(raw or "")
            hdr, src = "", ""
            if isinstance(parsed, dict):
                hdr = clean_llm_code((parsed.get("header") or "").strip())
                src = clean_llm_code((parsed.get("source") or "").strip())
            if (not isinstance(parsed, dict)) or (not hdr) or (not header_only and not src):
                _raw_path = os.path.join(
                    self.prompt_dir, f"other_codes_{rel.replace('/', '_')}_llm_raw.txt"
                )
                with open(_raw_path, "w", encoding="utf-8") as f:
                    f.write(raw or "")
                logger.warning(
                    "other_codes %s: parse/empty output, raw written to %s", rel, _raw_path
                )
                continue
            peer_includes = MainCodeGenerator._collect_other_codes_hpp_includes(
                package_info, package_name, c_src=c_src, self_rel=rel
            )
            hdr = MainCodeGenerator._apply_other_codes_header_includes(hdr, peer_includes)
            h_path = os.path.join(include_components_dir, *h_rel.split("/"))
            os.makedirs(os.path.dirname(h_path), exist_ok=True)
            with open(h_path, "w", encoding="utf-8") as f:
                f.write(hdr)
            if header_only:
                logger.info("other_codes %s -> %s (header only)", rel, h_path)
                continue
            if src.strip():
                src = src.strip() + "\n"
            c_path = os.path.join(src_dir, *c_rel.split("/"))
            os.makedirs(os.path.dirname(c_path), exist_ok=True)
            with open(c_path, "w", encoding="utf-8") as f:
                f.write(src)
            other_code_sources.append(f"src/{c_rel}".replace("\\", "/"))
            logger.info("other_codes %s -> %s", rel, c_path)

    def _gen_component_hpp(
        self,
        package_name: str,
        node_name: str,
        component: Dict[str, Any],
        shared_state_context: str = "",
        other_codes_context: str = "",
    ) -> str:
        cname = self._normalized_name(component.get("name"), "component")
        callbacks = component.get("callbacks", []) or []
        outputs = component.get("outputs", []) or []
        state_machine = component.get("state_machine", {}) or {}
        subprograms = [
            {
                **sub,
                "properties": {
                    **sub.get("properties", {}),
                    **{
                        k: _prepare_source_text(
                            sub["properties"].get(k, ""),
                            sub["properties"].get("Source_Name", "") or sub.get("name", ""),
                        )
                        for k in ("Source_Text", "source_text")
                        if sub.get("properties", {}).get(k)
                    },
                },
            }
            for sub in (component.get("subprograms") or [])
        ]
        fc = tmpl.fsm_template_context(component)
        sm_names = tmpl.fsm_state_names(component)
        if fc["use_fsm"] and len(sm_names) >= 2:
            fsm_decl = (
                f"- Multi-state FSM: declare `std::string fsm_state_` (initialize in .cpp to \"{sm_names[0]}\").\n"
            )
        elif fc["use_fsm"]:
            fsm_decl = (
                f"- Single-state FSM \"{sm_names[0]}\": do NOT declare or use `fsm_state_`.\n"
            )
        else:
            fsm_decl = "- No FSM: do not declare `fsm_state_`.\n"
        has_shared = bool(self._parse_shared_vars_from_context(shared_state_context))
        if has_shared:
            shared_step4 = (
                "4. Shared state — package-wide: declare `sim_state_` and a 3-arg constructor like every sibling. "
                "Per-component: access `sim_state_->FIELD` only if this component's bound subprogram uses those globals."
            )
            ctor_rule = (
                f"- Constructor (hard, package-wide): `{cname}(rclcpp::Node* node, rclcpp::CallbackGroup::SharedPtr cb_group, "
                f"std::shared_ptr<SharedSimState> sim_state);` declare `std::shared_ptr<SharedSimState> sim_state_;` on "
                "EVERY component on this node (even if this component never reads/writes shared fields).\n"
                f'- `#include "{package_name}/shared_sim_state.hpp"`; process-wide mutable state only via `SharedSimState` '
                "(one instance per node); do not redeclare package-level globals or duplicate sync primitives."
            )
        else:
            shared_step4 = "4. Shared state — none for this package; no `sim_state_`."
            ctor_rule = (
                f"- Constructor: `{cname}(rclcpp::Node* node, rclcpp::CallbackGroup::SharedPtr cb_group);` "
                "No `sim_state_`."
            )
        prompt = f"""
You are an expert ROS 2 Jazzy C++ developer (rclcpp).
Generate a complete component header (.hpp) from the architecture contract below.

Package: {package_name}
Node: {node_name}
Model context:
callbacks={json.dumps(callbacks, ensure_ascii=False, separators=(',', ':'))}
outputs={json.dumps(outputs, ensure_ascii=False, separators=(',', ':'))}
state_machine={json.dumps(state_machine, ensure_ascii=False, separators=(',', ':'))}
subprograms_excerpt={json.dumps(subprograms, ensure_ascii=False, separators=(',', ':'))}

Before writing code, reason through:
1. Inputs — map each `callbacks` entry to a subscription port and a cache member.
2. Outputs — map each `outputs` entry to a publisher member.
3. Behavior — what does `state_machine` or `subprograms` imply for `control_loop()`?
{shared_step4}

Design guidelines (use your judgment):
- Class name `{cname}`; keep identifiers lowercase; wrap the class in `namespace {package_name}` only (no nested `components` sub-namespace).
- Declarations only: no function bodies in this .hpp (implement everything in the matching .cpp).
- Use `rclcpp::Node* node_` and `rclcpp::Logger logger_`; do not embed or construct `rclcpp::Node`.
- Use rclcpp, std_msgs, C++17. Prefer std_msgs over custom rosidl types.
{ctor_rule}
- Member naming (match project template): `callback_group_`; per callback port `<port>_callback`, `sub_<port>_`, `<port>_cache_`; per output `pub_<publisher_member>_` where `publisher_member` is JSON `publisher_member` or `port`; `timer_`; declare `void control_loop();` only — do NOT declare `dispatch()` or other ad-hoc public methods.
{fsm_decl}
- Declare only types and members justified by the JSON; avoid inventing extra structs or enums.
- Topic names must match the `topic` field in Component JSON exactly; do not rename or invent topics.

Output only valid C++ header code, no markdown.
Shared state analysis (optional):
{shared_state_context or "{}"}
"""
        if other_codes_context:
            prompt += other_codes_context
        code, _ = self._call_llm(f"{cname}_hpp_prompt", prompt, f"{cname}_hpp")
        return code

    def _gen_component_cpp(
        self,
        package_name: str,
        node_name: str,
        component: Dict[str, Any],
        hpp_code: str,
        shared_state_context: str = "",
        other_codes_context: str = "",
    ) -> str:
        cname = self._normalized_name(component.get("name"), "component")
        callbacks = component.get("callbacks", []) or []
        outputs = component.get("outputs", []) or []
        state_machine = component.get("state_machine", {}) or {}
        subprograms = [
            {
                **sub,
                "properties": {
                    **sub.get("properties", {}),
                    **{
                        k: _prepare_source_text(
                            sub["properties"].get(k, ""),
                            sub["properties"].get("Source_Name", "") or sub.get("name", ""),
                        )
                        for k in ("Source_Text", "source_text")
                        if sub.get("properties", {}).get(k)
                    },
                },
            }
            for sub in (component.get("subprograms") or [])
        ]
        ros_logger_prefix = f"{node_name}.{cname}" if node_name else cname
        has_shared = bool(self._parse_shared_vars_from_context(shared_state_context))
        if has_shared:
            shared_step6 = (
                "6a. Shared state wiring (package-wide, hard): constructor MUST match the header's 3-arg signature "
                "and initialize `sim_state_(sim_state)` in the initializer list (even if this component never "
                "touches shared fields).\n"
                "6b. Shared state access (per-component): only if THIS component's bound subprogram reads or assigns "
                "a global mapped to SharedSimState, use `sim_state_->FIELD` under "
                "`std::lock_guard<std::mutex> guard(sim_state_->mtx);`; otherwise leave `sim_state_` unused."
            )
            ctor_hint = (
                "- Constructor: initialize `logger_(node->get_logger())`; store `callback_group_`; "
                "take `(node, cb_group, sim_state)` and set `sim_state_(sim_state)` in the initializer list; "
                "wire pub/sub/timer using `node_` and `callback_group_`."
            )
        else:
            shared_step6 = "6. Shared state — none for this package."
            ctor_hint = (
                "- Constructor: initialize `logger_(node->get_logger())`; store `callback_group_`; "
                "wire pub/sub/timer using `node_` and `callback_group_`."
            )
        prompt = f"""
You are an expert ROS 2 Jazzy C++ developer (rclcpp).
Implement the component source (.cpp) for class `{cname}`.

Package: {package_name}
Node: {node_name}
Model context:
callbacks={json.dumps(callbacks, ensure_ascii=False, separators=(',', ':'))}
outputs={json.dumps(outputs, ensure_ascii=False, separators=(',', ':'))}
state_machine={json.dumps(state_machine, ensure_ascii=False, separators=(',', ':'))}
subprograms_excerpt={json.dumps(subprograms, ensure_ascii=False, separators=(',', ':'))}
logger_prefix={ros_logger_prefix}

Header contract (binding — class name, namespace, members, and method signatures must match exactly):
{hpp_code}

Reason step by step before coding:
1. Read the header contract first; implement only what it declares — do not add, rename, or omit members or methods.
2. Ports — wire subscriptions/publishers from `callbacks`/`outputs` using topic, port, message_type, and QoS from model context.
3. Callbacks — update caches safely; guard nullptr before dereferencing.
4. control_loop — if `state_machine` present: implement transitions; else derive behavior from `subprograms_excerpt` (bound routine only).
5. Subprograms — treat C in `subprograms_excerpt` as reference only: inline only this component's bound subprogram body into `control_loop()`; ignore other routines in the same excerpt; no new helper/wrapper functions; no `*_aadl` unless declared in bundled headers below.
{shared_step6}
7. Logging (hard) — prefix every `Received`/`Published` line with `[{ros_logger_prefix}]` then the port name from JSON (e.g. `RCLCPP_INFO(logger_, "[{ros_logger_prefix}] Received <port>: ...")`). State changes: `[{ros_logger_prefix}] State transition: %s -> %s`. Without this prefix, runtime_analysis assigns logs to the process node instead of this thread. Never use `RCLCPP_DEBUG`; never include the substring `error` in log strings.

Implementation hints:
- Implement all methods declared in the header contract; do not add `dispatch()` or methods absent from the header.
{ctor_hint}
- Every `RCLCPP_INFO` for `Received`/`Published`/`State transition` must include the `[{ros_logger_prefix}]` prefix at the start of the format string.
- `#include "{package_name}/components/{cname}.hpp"`; match the header's namespace and class name exactly.
- Topic names in create_subscription/create_publisher must match the `topic` field in Component JSON exactly.
- QoS (Jazzy): use `.reliable()`/`.best_effort()` and `.transient_local()`/`.durability_volatile()`; never `.volatile()` or `.volatile_()`.
- Pub/sub callback group: use `rclcpp::SubscriptionOptions`/`PublisherOptions` with `options.callback_group`; never pass `callback_group_` as a bare extra argument to `create_subscription`/`create_publisher`.
- Timers: use `node_->create_wall_timer(...)`, not the free function `rclcpp::create_wall_timer`.
- Do not declare new class members in the .cpp that are absent from the header contract.

Output only valid C++ source code, no markdown.
Shared state analysis (optional):
{shared_state_context or "{}"}
"""
        shared_vars = self._parse_shared_vars_from_context(shared_state_context)
        if shared_vars:
            names_md = ", ".join(f"`sim_state_->{v['name']}`" for v in shared_vars)
            prompt += f"""
---
Shared state (mutex-protected struct `SharedSimState`, member `sim_state_`).
Lock: `std::lock_guard<std::mutex> guard(sim_state_->mtx);`
Fields (int): {names_md}
Rules for shared state access:
- **Wiring vs access**: all components store `sim_state_` when the package has shared state; the rules below apply only when THIS component's subprogram uses those fields.
- These fields are the C++ equivalents of the C globals in the source (e.g. `simu.c`). They represent **mutable plant/actuator state** shared across all components in this process.
- **Read**: use `sim_state_->FIELD` wherever the original C subprogram reads the global.
- **Write-back is mandatory**: if the original C subprogram **assigns** to a global, your C++ code **MUST** write it back via `sim_state_->FIELD = ...;`
- All reads **and** writes must be inside `std::lock_guard<std::mutex> guard(sim_state_->mtx);`
- Never declare a local copy that shadows the shared field; update `sim_state_->FIELD` in place.
Identified shared variables:
{shared_state_context}
"""
        if other_codes_context:
            prompt += other_codes_context
        code, _ = self._call_llm(f"{cname}_cpp_prompt", prompt, f"{cname}_cpp")
        return code

    def _gen_node_cpp(
        self,
        package_name: str,
        node: Dict[str, Any],
        shared_state_context: str = "",
    ) -> str:
        node_name = self._normalized_name(node.get("name"), "node")
        executor_json = json.dumps(node.get("executor") or {}, ensure_ascii=False)
        has_shared = bool(self._parse_shared_vars_from_context(shared_state_context))
        prompt = f"""
You are an expert ROS 2 Jazzy C++ developer (rclcpp).
Generate a process-style ROS 2 node that composes multiple component classes.

Package: {package_name}
Node JSON: {json.dumps(node, ensure_ascii=False)}
Executor (ROS 2 architecture — implement callback groups and executor type as specified):
{executor_json}

Think through:
1. How should `executor` (type, callback_groups, period_ms, component bindings) map to rclcpp callback groups and the chosen executor?
2. Which components belong to this process and how should they share one rclcpp::Node?
3. How should main() initialize ROS, spin, and shut down cleanly?

Guidelines:
- One node class named `{node_name}_node_cls` in the **global namespace** plus `int main(int argc, char** argv)`.
- Lowercase identifiers; include `"{package_name}/components/<component>.hpp"` for each component.
- Process node wrapper only: create callback groups, instantiate components, and spin — do NOT add publishers, subscribers, timers, or `dispatch()` on the node itself (all I/O lives inside component classes).
- Callback groups (hard): JSON types `MutuallyExclusiveCallbackGroup` / `ReentrantCallbackGroup` are architecture labels, NOT C++ classes. Create groups with `this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)` or `::Reentrant`. Never `std::make_shared<rclcpp::MutuallyExclusiveCallbackGroup>`.
- Executor (hard): map JSON `executor.type` — `MultiThreadedExecutor` → `rclcpp::executors::MultiThreadedExecutor`, `SingleThreadedExecutor` → `rclcpp::executors::SingleThreadedExecutor`.
- Match each component's constructor arity from its `"{package_name}/components/<name>.hpp"` header.
- Default (header has only `node` + `cb_group`): `comp_<name>_ = std::make_shared<{package_name}::<name>>(this, cbg_<name>_);`
- When shared-state analysis below is non-empty{" or component headers declare `sim_state_`" if has_shared else ""}: `#include "{package_name}/shared_sim_state.hpp"`; create one `auto sim_state = std::make_shared<{package_name}::SharedSimState>();` and pass `sim_state` as the third argument to **every** component on this node (same instance for all).
- Follow the Executor JSON above for which component gets which callback group; do not invent layout beyond the architecture.

Output only valid C++ source code, no markdown.
"""
        if shared_state_context.strip():
            prompt += f"""
Shared state analysis (for wiring sim_state):
{shared_state_context}
"""
        code, _ = self._call_llm(f"{node_name}_node_cpp_prompt", prompt, f"{node_name}_node_cpp")
        return code

    def _gen_device_node_cpp(self, package_name: str, node: Dict[str, Any]) -> str:
        node_name = self._normalized_name(node.get("name"), "node")
        executor_json = json.dumps(node.get("executor") or {}, ensure_ascii=False)
        prompt = f"""
You are an expert ROS 2 Jazzy C++ (rclcpp) developer.
Generate a device-style ROS 2 node: publishers/subscribers live on the node itself (no separate component classes).

Package: {package_name}
Node JSON: {json.dumps(node, ensure_ascii=False, separators=(',', ':'))}
Executor (ROS 2 architecture — implement callback groups and executor type as specified):
{executor_json}

Hard constraints (must follow):
- Naming (hard): one class `{node_name}_device`, exactly one `int main(int argc, char** argv)`; lowercase identifiers; keep topic names and message types from Node JSON.
- ROS node name (hard): constructor MUST call `rclcpp::Node("{node_name}")` using the Node JSON `name` field exactly.
- Callback groups: use `this->create_callback_group(rclcpp::CallbackGroupType::...)`; never invent `rclcpp::MutuallyExclusiveCallbackGroup` classes.
- Callback group on pub/sub (hard): use `rclcpp::PublisherOptions` / `rclcpp::SubscriptionOptions`, set `options.callback_group`; pass to `create_publisher(topic, qos, options)` or `create_subscription(topic, qos, callback, options)` — never pass `CallbackGroup` as a bare extra argument.
- Timer callback group: pass `CallbackGroup::SharedPtr` as the 3rd argument to `create_wall_timer(period, callback, group)`.
- QoS (Jazzy, hard): use `.reliable()`/`.best_effort()` and `.transient_local()`/`.durability_volatile()`; never `.volatile()` or `.volatile_()`.
- Add runtime logs for analyzer compatibility:
  - On receive: `RCLCPP_INFO(this->get_logger(), "Received <port>: ...")`
  - On publish: `RCLCPP_INFO(this->get_logger(), "Published <port>: ...")`
  Use exact `Received` / `Published` prefixes. Do not prefix logs with `[node.component]`. Never use `RCLCPP_DEBUG`; never include the substring `error` in log strings.
- **State changes must appear in logs (hard requirement)**: If Behavior Annex / `state_machine` implies discrete states, maintain an explicit state variable and **every time** the architectural state changes, immediately log with exactly:
  `RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", from_state.c_str(), to_state.c_str());`
  Do not update state without this log on the same path. Single-state self-loop only: do not fabricate multi-step `State transition` logs.
- If Node JSON has no `state_machine` and has `publishers`, use stub random publishing:
  - `#include <random>`; members: `std::mt19937 rng_`, `uniform_int_distribution` for int/bool, `uniform_real_distribution<double>` for float.
  - Per publisher `message_type`: Bool→random bool, Int32→dist_int_, String→`std::to_string(dist_int_)`, Float64→dist_float_, else Float32 from dist_float_.
  - `create_wall_timer` period: parse `properties.period` when present, otherwise `5ms`.
- If `state_machine` is present, implement BA/FSM logic instead of random stub publishing.

Output only valid C++ source code, no markdown.
"""
        code, _ = self._call_llm(f"{node_name}_device_node_cpp_prompt", prompt, f"{node_name}_device_node_cpp")
        return code

    def _gen_shared_sim_state_hpp(self, package_name: str, shared_state_context: str) -> str:
        prompt = f"""
You are an expert ROS 2 Jazzy C++ developer.
Design `{package_name}/shared_sim_state.hpp` from the shared-variable analysis below.

Shared state analysis:
{shared_state_context or "{}"}

Guidelines:
- `#pragma once`, `#include <mutex>`, wrap in `namespace {package_name} {{ ... }}`.
- `struct SharedSimState` with `mutable std::mutex mtx` as the first synchronization member.
- One `int` member per `shared_state_variables` entry; initialize from `initial_value` in the member initializer or in-class default.
- Do not add fields absent from the analysis; omit mutex/sync-only symbols from the analysis.

Output only valid C++ header code, no markdown.
"""
        if self.error_context:
            prompt += f"\n\nRepair context (this header only):\n{self.error_context}\n"
        code, _ = self._call_llm(
            f"{package_name}_shared_sim_state_hpp_prompt",
            prompt,
            f"{package_name}_shared_sim_state_hpp",
            append_error_context=False,
        )
        return code

    def _cmake_nodes_for_package(self, nodes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        cmake_nodes: List[Dict[str, Any]] = []
        for n in nodes:
            n_components = n.get("components", []) or n.get("subcomponents", [])
            n_has_ports = bool(n.get("ports") or [])
            if n_components or n_has_ports or tmpl.is_device_style_node(n):
                cmake_nodes.append(n)
        return cmake_nodes

    def _write_config_files_from_template(
        self,
        main_gen: MainCodeGenerator,
        package_name: str,
        nodes: List[Dict[str, Any]],
        project_dir: str,
        other_code_sources: List[str],
    ) -> None:
        """CMakeLists.txt and package.xml via Jinja templates (no LLM)."""
        main_gen._package_other_codes = sorted(set(other_code_sources))
        result = main_gen.generate_config_files(package_name, nodes, project_dir)
        lengths = result.get("code_length") or {}
        for name, length in lengths.items():
            self.statistics["config_stats"][name] = {"code_length": length}

    @staticmethod
    def _write(path: str, content: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)

    def _save_statistics(self) -> None:
        stats_file = os.path.join(self.output_dir, "generation_statistics.json")
        try:
            with open(stats_file, "w", encoding="utf-8") as f:
                json.dump(self.statistics, f, ensure_ascii=False, indent=2)
            logger.info("Saved statistics: %s", stats_file)
        except Exception as e:
            logger.error("Error saving statistics: %s", e)

    def generate_code(
        self,
        generate_config_files: bool = True,
        only_components: Optional[List[str]] = None,
        only_nodes: Optional[List[str]] = None,
        only_other_codes: Optional[List[str]] = None,
        only_shared_sim_state: bool = False,
        config_only: bool = False,
    ) -> None:
        start = time.time()
        try:
            arch = read_json_file(self.arch_file)
            packages = self._extract_packages(arch)
        except Exception as e:
            logger.error("Failed to load architecture file %s: %s", self.arch_file, e)
            return
        only_components_set = {c.strip().lower() for c in (only_components or []) if c and c.strip()}
        only_nodes_set = {n.strip().lower() for n in (only_nodes or []) if n and n.strip()}
        only_other_codes_set = {
            str(c).replace("\\", "/") for c in (only_other_codes or []) if c
        }
        incremental_components_only = bool(only_components_set)
        incremental_nodes_only = bool(only_nodes_set)
        incremental_other_codes_only = bool(only_other_codes_set)
        incremental_shared_sim_only = bool(only_shared_sim_state)

        for pkg in packages:
            try:
                package_name = str(pkg.get("name", "generated_pkg")).lower()
                nodes = pkg.get("nodes", [])
                if not nodes:
                    logger.info("Skip package %s: no nodes in ROS architecture", package_name)
                    continue
                project_dir = os.path.join(self.output_dir, package_name)
                include_dir = os.path.join(project_dir, "include", package_name, "components")
                src_comp_dir = os.path.join(project_dir, "src", "components")
                src_dir = os.path.join(project_dir, "src")
                for d in (include_dir, src_comp_dir, src_dir):
                    os.makedirs(d, exist_ok=True)

                main_gen = MainCodeGenerator(
                    self.arch_file,
                    self.output_dir,
                    self.api_key,
                    error_context=self.error_context if incremental_other_codes_only else "",
                )
                main_gen._package_other_codes = []
                main_gen._package_other_codes_hpp_includes = (
                    MainCodeGenerator._collect_other_codes_hpp_includes(pkg, package_name)
                )
                _dummy_stats: Dict[str, int] = {
                    "prompt_tokens": 0,
                    "completion_tokens": 0,
                    "total_tokens": 0,
                }
                _dummy_len: Dict[str, Any] = {"components": {}, "nodes": {}, "item_tokens": {}}
                self._generate_package_other_codes_heuristic(
                    pkg,
                    include_dir,
                    src_dir,
                    package_name,
                    incremental_components_only=incremental_components_only or incremental_nodes_only,
                    other_code_sources=main_gen._package_other_codes,
                    only_other_codes=only_other_codes_set or None,
                )
                self._pkg_other_codes[package_name] = list(main_gen._package_other_codes)
                self._pkg_other_codes_includes[package_name] = list(
                    main_gen._package_other_codes_hpp_includes
                )
                if incremental_other_codes_only:
                    cmake_nodes = self._cmake_nodes_for_package(nodes)
                    other_sources = self._collect_other_code_cpp_sources(pkg, project_dir)
                    if other_sources:
                        self._write_config_files_from_template(
                            main_gen,
                            package_name,
                            cmake_nodes,
                            project_dir,
                            other_sources,
                        )
                    continue

                logger.info("Generating package %s with %d nodes", package_name, len(nodes))
                shared_state_context, _ = self._package_shared_state_context(
                    pkg,
                    package_name,
                    incremental_components_only
                    or incremental_nodes_only
                    or incremental_shared_sim_only,
                )

                if incremental_shared_sim_only:
                    if self._parse_shared_vars_from_context(shared_state_context):
                        shared_hpp = self._gen_shared_sim_state_hpp(package_name, shared_state_context)
                        shared_hpp_path = os.path.join(
                            project_dir, "include", package_name, "shared_sim_state.hpp"
                        )
                        self._write(shared_hpp_path, shared_hpp)
                        self.statistics["config_stats"]["shared_sim_state.hpp"] = {
                            "code_length": len(shared_hpp),
                        }
                    continue

                if (
                    not config_only
                    and not incremental_other_codes_only
                    and not incremental_nodes_only
                    and not incremental_components_only
                    and self._parse_shared_vars_from_context(shared_state_context)
                ):
                    shared_hpp = self._gen_shared_sim_state_hpp(package_name, shared_state_context)
                    shared_hpp_path = os.path.join(
                        project_dir, "include", package_name, "shared_sim_state.hpp"
                    )
                    self._write(shared_hpp_path, shared_hpp)
                    self.statistics["config_stats"]["shared_sim_state.hpp"] = {
                        "code_length": len(shared_hpp),
                    }

                for node in nodes:
                    try:
                        node_name = self._normalized_name(node.get("name"), "node")
                        if config_only:
                            continue
                        if only_nodes_set and node_name not in only_nodes_set:
                            node_comps = node.get("components", []) or node.get("subcomponents", [])
                            has_repair_component = bool(only_components_set) and any(
                                self._normalized_name(c.get("name"), "component") in only_components_set
                                for c in node_comps
                                if isinstance(c, dict)
                            )
                            if not has_repair_component:
                                continue

                        node_for_prompt = dict(node)
                        node_for_prompt["name"] = node_name

                        if tmpl.is_device_style_node(node):
                            if incremental_components_only and not (
                                incremental_nodes_only and node_name in only_nodes_set
                            ):
                                continue
                            device_cpp = self._gen_device_node_cpp(package_name, node_for_prompt)
                            self._write(os.path.join(src_dir, f"{node_name}_node.cpp"), device_cpp)
                            self.statistics["node_stats"][node_name] = {
                                "code_length": len(device_cpp),
                                "kind": "device",
                            }
                            continue

                        components = node.get("components", []) or node.get("subcomponents", [])
                        if only_components_set:
                            components = [
                                c for c in components
                                if self._normalized_name(c.get("name"), "component") in only_components_set
                            ]
                        has_ports = bool(node.get("ports") or [])
                        if not components and not has_ports:
                            logger.info(
                                "Skip node %s in package %s: no subcomponents and no ports",
                                node_name,
                                package_name,
                            )
                            continue

                        # Stage 1 + 2: component hpp/cpp (LLM)
                        if incremental_components_only or not incremental_nodes_only:
                            oc_ctx = self._build_other_codes_prompt_block(package_name)
                            for comp in components:
                                cname = self._normalized_name(comp.get("name"), "component")
                                comp_for_prompt = dict(comp)
                                comp_for_prompt["name"] = cname
                                hpp = self._gen_component_hpp(
                                    package_name,
                                    node_name,
                                    comp_for_prompt,
                                    shared_state_context=shared_state_context,
                                    other_codes_context=oc_ctx,
                                )
                                cpp = self._gen_component_cpp(
                                    package_name,
                                    node_name,
                                    comp_for_prompt,
                                    hpp,
                                    shared_state_context=shared_state_context,
                                    other_codes_context=oc_ctx,
                                )
                                self._write(os.path.join(include_dir, f"{cname}.hpp"), hpp)
                                self._write(os.path.join(src_comp_dir, f"{cname}.cpp"), cpp)
                                self.statistics["component_stats"][cname] = {
                                    "hpp_length": len(hpp),
                                    "cpp_length": len(cpp),
                                }
                                self._maybe_run_code_comparison(comp_for_prompt, cpp)

                        # Stage 3: process node cpp (LLM)
                        if incremental_components_only and not (
                            incremental_nodes_only and node_name in only_nodes_set
                        ):
                            continue
                        raw_node_components = node.get("components", []) or node.get("subcomponents", [])
                        node_for_prompt["components"] = [
                            {**c, "name": self._normalized_name(c.get("name"), "component")}
                            if isinstance(c, dict) else c
                            for c in raw_node_components
                        ]
                        node_cpp = self._gen_node_cpp(
                            package_name, node_for_prompt, shared_state_context
                        )
                        self._write(os.path.join(src_dir, f"{node_name}_node.cpp"), node_cpp)
                        self.statistics["node_stats"][node_name] = {
                            "code_length": len(node_cpp),
                            "kind": "process",
                        }
                    except Exception as e:
                        logger.error("Failed to generate node in package %s: %s", package_name, e)
                        continue

                # Stage 4: CMakeLists + package.xml (templates, no LLM)
                if not generate_config_files:
                    continue
                other_sources = self._pkg_other_codes.get(package_name) or []
                if not other_sources:
                    other_sources = self._collect_other_code_cpp_sources(pkg, project_dir)
                self._write_config_files_from_template(
                    main_gen,
                    package_name,
                    self._cmake_nodes_for_package(nodes),
                    project_dir,
                    other_sources,
                )
            except Exception as e:
                logger.error("Failed to generate package: %s", e)
                continue

        self.statistics["generation_time"] = time.time() - start
        logger.info(
            "Generation finished at %s, elapsed %.2fs",
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            self.statistics["generation_time"],
        )

        token_stats_file = os.path.join(self.prompt_dir, "token_usage_stats.json")
        try:
            with open(token_stats_file, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "token_stats": {
                            "prompt_tokens": self.statistics["prompt_tokens"],
                            "completion_tokens": self.statistics["completion_tokens"],
                            "total_tokens": self.statistics["total_tokens"],
                        },
                        "time_stats": {"total_time_seconds": self.statistics["generation_time"]},
                    },
                    f,
                    ensure_ascii=False,
                    indent=2,
                )
            logger.info("Successfully saved token statistics to file: %s", token_stats_file)
        except Exception as e:
            logger.error("Error saving token statistics: %s", e)
        self._save_statistics()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RQ3 group3: template CMake/package.xml; LLM for components, nodes, devices, shared state, other_codes"
    )
    parser.add_argument("-r", "--aadl", required=True, help="ROS architecture JSON path")
    parser.add_argument("-o", "--output", default="./ros_generated", help="Output directory")
    parser.add_argument("-k", "--key", default=None, help="LLM API key")
    parser.add_argument(
        "--only-components",
        default="",
        help="Comma-separated component names to regenerate; skips node/config regeneration",
    )
    parser.add_argument(
        "--only-nodes",
        default="",
        help="Comma-separated node names to regenerate (node cpp + filtered components)",
    )
    parser.add_argument(
        "--error_context",
        default="",
        help="Runtime error context appended to regeneration prompts",
    )
    parser.add_argument(
        "--config-only",
        action="store_true",
        help="Regenerate only CMakeLists.txt and package.xml",
    )
    parser.add_argument(
        "--only-other-codes",
        default="",
        help="Comma-separated other_codes code_name paths to repair (e.g. common/app_code.c)",
    )
    parser.add_argument(
        "--only-shared-sim-state",
        action="store_true",
        help="Regenerate only shared_sim_state.hpp (no components/nodes)",
    )
    args = parser.parse_args()

    generator = ROSCodeGenerator(args.aadl, args.output, args.key, error_context=args.error_context)
    only_components = [x.strip() for x in args.only_components.split(",") if x.strip()]
    only_nodes = [x.strip() for x in args.only_nodes.split(",") if x.strip()]
    only_other_codes = [x.strip() for x in args.only_other_codes.split(",") if x.strip()]
    only_shared_sim_state = bool(args.only_shared_sim_state)
    incremental = bool(only_components or only_other_codes or only_nodes or only_shared_sim_state)
    generator.generate_code(
        generate_config_files=True if args.config_only else (not incremental),
        only_components=only_components or None,
        only_nodes=only_nodes or None,
        only_other_codes=only_other_codes or None,
        only_shared_sim_state=only_shared_sim_state,
        config_only=args.config_only,
    )
    logger.info("Code generation completed: %s", args.output)


if __name__ == "__main__":
    main()

