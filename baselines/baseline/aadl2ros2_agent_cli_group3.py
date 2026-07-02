#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AADL to ROS2 conversion CLI tool (RQ3 group3 — LLM end-to-end component generation + closed-loop repair).
usage: python aadl2ros2_agent_cli_group3.py -i ./example/fcc -f Flight_Controller.aadl -s Flight_Controller -o ./output -k your_api_key
"""

import json
import os
import re
import shutil
import sys

from rich.console import Console
from rich.panel import Panel
from rich.text import Text

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from planner.system_state import SystemState
from planner.nodes_group3 import (
    aadl_parser_tool,
    dynamic_tester_tool,
    architect_convert_tool,
    coder_agent,
)

def create_welcome_panel() -> Panel:
    """
    Create a welcome panel using rich library
    
    Returns:
        Rich Panel object
    """
    welcome_text = Text("AADL2ROS2 Agent CLI (RQ3 group3)", style="bold cyan")
    welcome_text.append("\n\n", style="reset")
    welcome_text.append("Welcome to the AADL to ROS2 conversion tool!", style="green")
    welcome_text.append("\n\n", style="reset")
    welcome_text.append("This tool converts AADL models to ROS2 code using a multi-agent system.", style="white")
    
    return Panel(welcome_text, title="[bold blue]AADL2ROS2 Agent[/bold blue]", border_style="blue")


def create_initial_state(args) -> SystemState:
    """
    Create initial SystemState from command-line arguments
    
    Args:
        args: Parsed command-line arguments
    Returns:
        Initial SystemState dictionary
    """
    out_dir = os.path.abspath(os.path.normpath(args.output_dir))
    xml_file_path = os.path.join(out_dir, f"{args.system}.json")
    ros2_arch_file = os.path.join(out_dir, f"{args.system}_ros.json")
    initial_state: SystemState = {
        "aadl_file_path": args.input_dir,
        "aadl_file_name": args.file_name,
        "output_dir": out_dir,
        "api_key": args.api_key,
        "system_name": args.system,
        "xml_file_path": xml_file_path,
        "ros2_arch_file": ros2_arch_file,
        "static_check_errors": [],
        "dynamic_test_errors": [],
        "error_types": [],
        "current_node": "initialization",
        "inject_virtual_io": bool(getattr(args, "inject_virtual_io", False)),
    }
    
    return initial_state


def snapshot_iteration_metrics(output_dir: str, iteration: int) -> None:
    """
    Save per-iteration artifacts for later metric summarization.
    Captures:
      - runtime_analysis_report.txt
      - ros_info/node.log
      - ros_info/code_comparison (or Code_comparison)
    """
    iter_root = os.path.join(output_dir, "ros_info", "iterations", f"iter_{iteration:02d}")
    os.makedirs(iter_root, exist_ok=True)

    report_src = os.path.join(output_dir, "runtime_analysis_report.txt")
    if os.path.exists(report_src):
        shutil.copy2(report_src, os.path.join(iter_root, "runtime_analysis_report.txt"))

    node_log_src = os.path.join(output_dir, "ros_info", "node.log")
    if os.path.exists(node_log_src):
        shutil.copy2(node_log_src, os.path.join(iter_root, "node.log"))

    code_cmp_src = os.path.join(output_dir, "ros_info", "code_comparison")
    if not os.path.exists(code_cmp_src):
        code_cmp_src = os.path.join(output_dir, "ros_info", "Code_comparison")
    if os.path.exists(code_cmp_src):
        dst = os.path.join(iter_root, "code_comparison")
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(code_cmp_src, dst)


_COMPILE_ERR_PATH_RE = re.compile(
    r"^(?P<file>.+?\.(?:cpp|c|cc|cxx|h|hpp|hh)):(?P<line>\d+)",
)


def error_signature(error: dict, other_code: str = "") -> str:
    target = other_code or error.get("component") or error.get("node") or "unknown"
    return "|".join([
        str(target),
        str(error.get("error_type") or "Unknown"),
        str(error.get("exception_message") or "").strip(),
    ])


def load_arch_other_code_names(ros2_arch_file: str) -> set:
    names: set = set()
    try:
        with open(ros2_arch_file, "r", encoding="utf-8") as f:
            arch = json.load(f)
    except (OSError, json.JSONDecodeError):
        return names
    for pkg in arch.get("ROSPackages", []) if isinstance(arch, dict) else []:
        for it in pkg.get("other_codes") or []:
            if isinstance(it, dict) and it.get("code_name"):
                names.add(str(it["code_name"]).replace("\\", "/"))
    return names


def _other_code_gcc_path_suffixes(code_name: str) -> tuple:
    """Suffixes of generated other_codes artifacts (same layout as coder_agent other_codes)."""
    stem = os.path.splitext(str(code_name).replace("\\", "/"))[0]
    return (f"/components/{stem}.hpp", f"/src/{stem}.cpp")


def _other_code_name_for_compile_path(fp: str, arch_names: set) -> str:
    """Match gcc file path to arch ``code_name`` via generated .hpp / .cpp suffixes."""
    if not arch_names:
        return ""
    norm = "/" + fp.replace("\\", "/").lstrip("/")
    hit = ""
    for name in sorted(arch_names, key=len, reverse=True):
        if any(norm.endswith(suf) for suf in _other_code_gcc_path_suffixes(name)):
            hit = name
            break
    return hit


def infer_other_codes_from_errors(errors: list, arch_names: set) -> list:
    if not arch_names:
        return []
    found: set = set()
    for e in errors:
        if not isinstance(e, dict):
            continue
        msg = (e.get("exception_message") or "").strip()
        m = _COMPILE_ERR_PATH_RE.match(msg)
        if not m:
            continue
        cn = _other_code_name_for_compile_path(m.group("file"), arch_names)
        if cn:
            found.add(cn)
    return sorted(found)


def _errors_for_other_code(errors: list, code_name: str) -> list:
    return [
        e for e in errors
        if isinstance(e, dict)
        and (m := _COMPILE_ERR_PATH_RE.match((e.get("exception_message") or "").strip()))
        and _other_code_name_for_compile_path(m.group("file"), {code_name}) == code_name
    ]


def has_repeated_behavior_error(errors: list, repair_attempts: dict) -> bool:
    return any(
        e.get("error_type") == "BehaviorError"
        and repair_attempts.get(error_signature(e), 0) > 0
        for e in errors
        if isinstance(e, dict)
    )


def expand_direct_neighbors(ros2_arch_file: str, components: list) -> list:
    targets = {str(c).lower() for c in components if c}
    if not targets:
        return []
    try:
        with open(ros2_arch_file, "r", encoding="utf-8") as f:
            arch = json.load(f)
    except (OSError, json.JSONDecodeError):
        return sorted(targets)

    related = set(targets)
    for pkg in arch.get("ROSPackages", []) if isinstance(arch, dict) else []:
        for node in pkg.get("nodes", []) or []:
            topic_to_components = {}
            for comp in node.get("components", []) or []:
                name = str(comp.get("name") or "").lower()
                if not name:
                    continue
                for endpoint in (comp.get("callbacks", []) or []) + (comp.get("outputs", []) or []):
                    topic = endpoint.get("topic") if isinstance(endpoint, dict) else None
                    if topic:
                        topic_to_components.setdefault(topic, set()).add(name)
            for comps in topic_to_components.values():
                if comps & targets:
                    related.update(comps)
    return sorted(related)


def _has_generated_main_nodes(output_dir: str) -> bool:
    """True if layout matches test_agent: <output>/<pkg>/src/*_node.cpp exists."""
    try:
        names = os.listdir(output_dir)
    except OSError:
        return False
    for name in names:
        src = os.path.join(output_dir, name, "src")
        if not os.path.isdir(src):
            continue
        for fn in os.listdir(src):
            if fn.endswith("_node.cpp"):
                return True
    return False


def main():
    """
    Main function
    """
    import argparse
    
    # Create argument parser
    parser = argparse.ArgumentParser(
        description="AADL to ROS2 conversion (RQ3 group3: LLM end-to-end codegen + closed-loop repair)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        Examples:
        python baselines/baseline/aadl2ros2_agent_cli_group3.py -i ./example/fcc -f Flight_Controller.aadl -s Flight_Controller
        -o ./output -k your_api_key
        """
    )
    
    # Add arguments matching aadl2code.py
    parser.add_argument("-i", "--input_dir", type=str, required=True, help="Directory containing AADL files (required)")
    parser.add_argument("-f", "--file_name", type=str, required=True, help="AADL model file name (required)")
    parser.add_argument("-s", "--system", type=str, required=True, help="Top-level AADL system name (required)")
    parser.add_argument("-o", "--output_dir", type=str, default="./output", help="Output directory for generated ROS2 code (default: ./output)")
    parser.add_argument("-k", "--api_key", type=str, default=None, help="LLM API key for code generation (optional, can also use DEEPSEEK_API_KEY env variable)")
    parser.add_argument("-iv", "--inject-virtual-io",action="store_true", help="During dynamic test only, start temporary virtual topic publishers for uncovered process inputs")
    args = parser.parse_args()
    console = Console()
    
    # Display welcome panel
    console.print(create_welcome_panel())
    
    # Create initial state
    initial_state = create_initial_state(args)
    
    # Run Phase 1: Model Ingestion
    updated_state = aadl_parser_tool(console,initial_state)

    # Run Phase 2: architect_agent
    updated_state = architect_convert_tool(console, updated_state)
    
    # Run Phase 3: coder_agent
    # updated_state = coder_agent(console, updated_state)
    if not _has_generated_main_nodes(updated_state["output_dir"]):
        console.print("[bold yellow]No *_node.cpp under output; skipping dynamic test phase.[/bold yellow]")
        return

    # Run Phase 4: repeat until no dynamic errors (or max iterations reached)
    repair_iter = 0
    max_repair_iters = 5
    repair_attempts = {}
    while True:
        repair_iter += 1
        console.print(f"[bold cyan]Dynamic test iteration {repair_iter}/{max_repair_iters}[/bold cyan]")
        # Run Phase 4: dynamic tester and runtime analysis
        updated_state = dynamic_tester_tool(console, updated_state)
        snapshot_iteration_metrics(updated_state["output_dir"], repair_iter)
        errors = updated_state.get("dynamic_test_errors") or []
        patched_timing = updated_state.get("patched_timing") or []

        # No errors and no timing changes → clean run, stop.
        if not errors and not patched_timing:
            break
        unresolved = [
            e for e in errors
            if isinstance(e, dict) and repair_attempts.get(error_signature(e), 0) >= 2
        ]
        if unresolved:
            console.print("[bold red]Unresolved errors after two repair attempts:[/bold red]")
            for e in unresolved:
                target = e.get("component") or e.get("node") or "unknown"
                console.print(
                    f"- {e.get('error_type')} {target}: {e.get('exception_message')}"
                )
            break
        if repair_iter >= max_repair_iters:
            console.print("[bold yellow]Reached max repair iterations, continue to next phase.[/bold yellow]")
            break
        # Timing thresholds were actually patched (count criteria met) → re-run to verify.
        if not errors and patched_timing:
            console.print("[bold cyan]Timing thresholds patched (count criteria met); re-running dynamic test.[/bold cyan]")
            continue

        # Read the global "Error Ledger (Iron Law)
        ledger_path = os.path.join(updated_state["output_dir"], "error_ledger.json")
        ledger_rules = ""
        if os.path.exists(ledger_path):
            with open(ledger_path, "r", encoding="utf-8") as f:
                try:
                    ledger_data = json.load(f)
                    rules = [f"{i+1}. {item.get('enforced_rule')}" for i, item in enumerate(ledger_data) if item.get('enforced_rule')]
                    if rules:
                        ledger_rules = "\n\n[CRITICAL STRICT RULES FROM PAST FAILURES]:\nYou MUST absolutely follow these rules, or the system will crash again:\n" + "\n".join(rules)
                except json.JSONDecodeError:
                    pass

        arch_other_names = load_arch_other_code_names(updated_state["ros2_arch_file"])
        only_other_codes = infer_other_codes_from_errors(errors, arch_other_names)
        shared_sim_hpp_errors = [
            e for e in errors
            if isinstance(e, dict)
            and "shared_sim_state.hpp" in str(e.get("exception_message") or "")
        ]
        only_components = sorted({e.get("component") for e in errors if e.get("component")})
        only_main_nodes = sorted({
            e.get("node") for e in errors
            if e.get("node") and not str(e.get("node", "")).endswith("_test_node")
        })

        if only_other_codes:
            console.print(
                "[bold yellow]Dynamic test found other_codes errors. Regenerating bundled sources...[/bold yellow]"
            )
            console.print(
                f"[bold cyan]other_codes targets:[/bold cyan] {', '.join(only_other_codes)}"
            )
            oc_errors = [
                e for e in errors
                if isinstance(e, dict)
                and any(_errors_for_other_code([e], oc) for oc in only_other_codes)
            ]
            other_codes_error_context = "\n".join(
                f"- error_type={e.get('error_type')} other_code=other_codes "
                f"message={e.get('exception_message')} "
                f"root_cause_analysis={e.get('root_cause_analysis')}"
                for e in oc_errors
            ) or "\n".join(
                f"- error_type={e.get('error_type')} message={e.get('exception_message')} "
                f"root_cause_analysis={e.get('root_cause_analysis')}"
                for e in errors if isinstance(e, dict)
            )
            other_codes_error_context += (
                "\n\n[REPAIR SCOPE]\n"
                f"Mode: other_codes local repair.\n"
                f"Regenerate only these other_codes entries: {', '.join(only_other_codes)}.\n"
                "Keep component bodies, node wrappers, topics, and CMake unchanged. "
                "Fix namespace/globals/typedef conflicts in the converted C++ headers and sources."
            )
            other_codes_error_context += ledger_rules
            updated_state = coder_agent(
                console,
                updated_state,
                only_other_codes=only_other_codes,
                error_context=other_codes_error_context,
            )
            for oc in only_other_codes:
                for e in _errors_for_other_code(errors, oc):
                    sig = error_signature(e, other_code=f"other_codes:{oc}")
                    repair_attempts[sig] = repair_attempts.get(sig, 0) + 1
            continue

        if shared_sim_hpp_errors:
            console.print(
                "[bold yellow]shared_sim_state.hpp compile errors. Regenerating shared header only...[/bold yellow]"
            )
            shared_error_context = "\n".join(
                f"- error_type={e.get('error_type')} message={e.get('exception_message')} "
                f"root_cause_analysis={e.get('root_cause_analysis')}"
                for e in shared_sim_hpp_errors
            )
            shared_error_context += (
                "\n\n[REPAIR SCOPE]\n"
                "Mode: shared_sim_state.hpp local repair.\n"
                "Regenerate ONLY `include/<package>/shared_sim_state.hpp` as a valid C++ header "
                "(struct SharedSimState with mtx + int fields from shared_state_variables). "
                "Do not emit component .cpp bodies or markdown fences."
            )
            shared_error_context += ledger_rules
            updated_state = coder_agent(
                console,
                updated_state,
                only_shared_sim_state=True,
                error_context=shared_error_context,
            )
            for e in shared_sim_hpp_errors:
                sig = error_signature(e)
                repair_attempts[sig] = repair_attempts.get(sig, 0) + 1
            continue

        if only_components or only_main_nodes:
            repair_components = only_components
            repair_nodes = only_main_nodes
            if only_components:
                repeated_behavior_error = has_repeated_behavior_error(errors, repair_attempts)
                repair_scope = "local repair"
                if repeated_behavior_error:
                    repair_components = expand_direct_neighbors(
                        updated_state["ros2_arch_file"],
                        only_components,
                    )
                    repair_scope = "expanded repair"
            else:
                repair_scope = "local repair"
            targets = []
            if repair_components:
                targets.append(f"components: {', '.join(repair_components)}")
            if repair_nodes:
                targets.append(f"nodes: {', '.join(repair_nodes)}")
            console.print(
                "[bold yellow]Dynamic test found codegen errors. Regenerating targets...[/bold yellow]"
            )
            console.print(f"[bold cyan]{repair_scope}[/bold cyan] {'; '.join(targets)}")
            repair_error_context = "\n".join(
                f"- error_type={e.get('error_type')} "
                f"component={e.get('component')} node={e.get('node')} "
                f"message={e.get('exception_message')} "
                f"root_cause_analysis={e.get('root_cause_analysis')}"
                for e in errors
                if isinstance(e, dict)
            )
            repair_error_context += "\n\n[REPAIR SCOPE]\n"
            if repair_components:
                repair_error_context += (
                    f"Mode: {repair_scope}.\n"
                    f"Regenerate only these components: {', '.join(repair_components)}.\n"
                    "Keep topics, message types, QoS, and unrelated files stable.\n"
                )
            if repair_nodes:
                repair_error_context += (
                    f"Regenerate only these nodes: {', '.join(repair_nodes)}.\n"
                    "Fix node wrapper / device node code only; keep unrelated components unchanged.\n"
                )
            repair_error_context += ledger_rules
            updated_state = coder_agent(
                console,
                updated_state,
                only_components=repair_components or None,
                only_nodes=repair_nodes or None,
                error_context=repair_error_context,
            )
            for e in errors:
                if not isinstance(e, dict):
                    continue
                if e.get("component") in (repair_components or []):
                    repair_attempts[error_signature(e)] = repair_attempts.get(error_signature(e), 0) + 1
                elif e.get("node") in (repair_nodes or []):
                    repair_attempts[error_signature(e)] = repair_attempts.get(error_signature(e), 0) + 1
            continue
        else:
            console.print("[bold yellow]Errors found but no node/component target identified, skip regeneration.[/bold yellow]")
            break
        
if __name__ == "__main__":
    main()