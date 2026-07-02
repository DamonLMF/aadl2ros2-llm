# -*- coding: utf-8 -*-
"""
Node implementations for AADL to ROS2 conversion system
"""

from typing import Any, Dict, List, Optional
import subprocess
import os
import sys
import json

from validator.runtime_analysis import RuntimeAnalysis
from .system_state import SystemState
from validator.error_analysis import (
    error_analysis,
    llm_add_root_cause_analysis,
    patch_timing_overruns,
)

def _timeout_warning_error_dicts(warnings_by_component: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Structured rows for timing WARNs: error_type TimeoutWarning, symptom = file + message,
    root_cause_analysis = The timing parameter settings are unreasonable, please adjust the parameters., enforced_rule empty (ledger uses empty string).
    """
    out: List[Dict[str, Any]] = []
    for comp, messages in (warnings_by_component or {}).items():
        for item in messages or []:
            if not isinstance(item, dict):
                continue
            msg = (item.get("message") or "").strip()
            out.append({
                "error_type": "TimeoutWarning",
                "exception_message": f"src/components/{comp}.cpp: {msg}",
                "root_cause_analysis": "The timing parameter settings are unreasonable, please adjust the parameters.",
                "enforced_rule": "",
            })
    return out


def update_error_ledger(ledger_path: str, errors: list, console):
    """
    add new errors and enforced rules to error ledger
    """
    ledger = []
    if os.path.exists(ledger_path):
        with open(ledger_path, "r", encoding="utf-8") as f:
            try:
                ledger = json.load(f)
            except json.JSONDecodeError:
                pass
    
    new_entries = 0
    for err in errors:
        # LLM returns 'enforced_rule', symptom from 'exception_message', root_cause from 'root_cause_analysis'
        if err.get("enforced_rule") or err.get("root_cause_analysis"):
            ledger.append({
                "error_type": err.get("error_type", "Unknown"),
                "symptom": err.get("exception_message", ""),
                "root_cause": err.get("root_cause_analysis", ""),
                "enforced_rule": err.get("enforced_rule")
                if "enforced_rule" in err
                else "AVOID this error in the future.",
            })
            new_entries += 1

    if new_entries > 0:
        with open(ledger_path, "w", encoding="utf-8") as f:
            json.dump(ledger, f, ensure_ascii=False, indent=4)
        console.print(f"[bold yellow]Ledger Updated:[/bold yellow] Added {new_entries} new rules to {ledger_path}")

def aadl_parser_tool(console, state: SystemState) -> Dict:
    """
    Deterministic AADL parser tool
    
    Args:
        console: Rich console for logging
        state: Current system state
        
    Returns:
        Updated state with parsed topology structure
    """
    # Run AADL parser
    xml_file_path = state["xml_file_path"]
    parser_script = os.path.join(os.path.dirname(os.path.dirname(__file__)), "aadl_parser", "aadl_parser.py")
    command = [
        sys.executable, parser_script,
        "-i", state["aadl_file_path"],
        "-f", state["aadl_file_name"],
        "-s", state["system_name"],
        "-o", state["output_dir"]
    ]
    console.print(f"execute command: {command}")
    subprocess.run(command, check=True, cwd=os.getcwd())
    if not os.path.exists(xml_file_path):
        console.print("[bold red]Error:[/bold red] Parser completed but no output file was generated")
        sys.exit(1)
    console.print("[bold green]Success:[/bold green] AADL parser completed successfully")
    console.print(f"Generated XML file: [green]{xml_file_path}[/green]")
    # Update state
    state["current_node"] = "aadl_parser"
    console.print(f"Current node: [green]{state['current_node']}[/green]")


    return state    

def runtime_analysis_tool(console, state: SystemState) -> Dict:
    """
    Runtime analysis tool
    
    Args:
        console: Rich console for logging
        state: Current system state
        
    Returns:
        Updated state with runtime analysis errors
    """
    # Run topic validator
    node_log_path = os.path.join(state["output_dir"], "ros_info", "node.log")
    runtime_analysis_report_path = os.path.join(state["output_dir"], "runtime_analysis_report.txt")
    if not os.path.exists(node_log_path):
        console.print(f"[bold yellow]Warning:[/bold yellow] Runtime analysis skipped, log file not found: {node_log_path}")
        state["runtime_analysis_errors"] = [{
            "node": None,
            "component": None,
            "function": "runtime_analysis",
            "error_type": "MissingLogFile",
            "exception_message": f"Runtime log file not found: {node_log_path}",
        }]
        state["current_node"] = "runtime_analysis"
        return state

    analysis = RuntimeAnalysis(node_log_path, state["ros2_arch_file"])
    analysis.run(runtime_analysis_report_path)
    # save runtime analysis report to runtime_analysis_errors
    runtime_analysis_errors = analysis.get_errors()
    state["runtime_analysis_errors"] = runtime_analysis_errors if isinstance(runtime_analysis_errors, list) else []

    if not os.path.exists(runtime_analysis_report_path):
        console.print("[bold red]Error:[/bold red] Runtime analysis tool completed but no output file was generated")
        sys.exit(1)
    console.print("[bold green]Success:[/bold green] Runtime analysis tool completed successfully")
    console.print(f"Generated runtime analysis report: [green]{runtime_analysis_report_path}[/green]")

    # Update state
    state["current_node"] = "runtime_analysis"
    console.print(f"Current node: [green]{state['current_node']}[/green]")

    return state 

def dynamic_tester_tool(console, state: SystemState) -> Dict:
    """
    Dynamic compilation and testing tool
    
    Args:
        console: Rich console for logging
        state: Current system state
        
    Returns:
        Updated state with dynamic test errors and error type
    """
    output_dir = state["output_dir"]
    node_log_path = os.path.join(output_dir, "ros_info", "node.log")
    dynamic_test_script = os.path.join(os.path.dirname(os.path.dirname(__file__)), "test_agent.py")
    command = [sys.executable, dynamic_test_script, "-p", output_dir]
    console.print(f"execute command: {command}")
    subprocess.run(command, check=True, cwd=os.getcwd())
    console.print("[bold green]Success:[/bold green] Dynamic test completed successfully")

    # Phase 4.1: parse node.log — compile / node [ERROR] vs [WARN] timing summaries
    ros_info_dir = os.path.join(output_dir, "ros_info")
    node_errors = error_analysis(
        node_log_path,
        xml_path=state['xml_file_path'],
    )
    warnings_by_component = node_errors.get("warnings_by_component", {})
    level_1_errors = list(node_errors.get("errors", []))

    # clear runtime analysis errors
    state["runtime_analysis_errors"] = []
    merged_errors: List[Dict[str, Any]] = []
    if level_1_errors:
        merged_errors = level_1_errors
    else:
        # Phase 4.2: no level-1 parse errors — runtime analysis on dynamic test output
        runtime_analysis_tool(console, state)
        level_2_errors = state["runtime_analysis_errors"]
        if level_2_errors:
            merged_errors = level_2_errors

    merged_errors = list(merged_errors or [])
    merged_errors.extend(node_errors.get("behavior_warning_errors") or [])

    hist_types = sorted({
        *(node_errors.get("error_types", [])),
        *[e.get("error_type") for e in merged_errors if isinstance(e, dict) and e.get("error_type")],
    })
    os.makedirs(ros_info_dir, exist_ok=True)
    history_path = os.path.join(ros_info_dir, "errors_history.json")
    history = []
    if os.path.exists(history_path):
        with open(history_path, "r", encoding="utf-8") as f:
            try:
                history = json.load(f)
            except json.JSONDecodeError:
                pass
    if not isinstance(history, list):
        history = []
    n = len(history) + 1
    history.append({
        "iteration": n,
        "errors": merged_errors,
        "error_types": hist_types,
        "warnings_by_component": warnings_by_component,
    })
    with open(history_path, "w", encoding="utf-8") as f:
        json.dump(history, f, ensure_ascii=False, indent=2)
    console.print(f"[bold blue]Iteration {n}:[/bold blue] Saved to {history_path}")

    # Phase 4.3: LLM root-cause (skip if colcon log was already fully diagnosed by LLM).
    if merged_errors and state.get("api_key") and not node_errors.get("unparsed_colcon_build"):
        enriched = llm_add_root_cause_analysis(
            merged_errors,
            api_key=state["api_key"],
            xml_path=state["xml_file_path"],
            raw_blocks=node_errors.get("raw_blocks", []),
        )
        if enriched:
            merged_errors = enriched
    
    timeout_errors = _timeout_warning_error_dicts(warnings_by_component)
    state["dynamic_test_errors"] = merged_errors
    state["current_node"] = "dynamic_tester"
    console.print(f"Current node: [green]{state['current_node']}[/green]")

    # Phase 4.4: conditionally patch timing parameters; timeout-only cases are recorded without retry.
    patched_timing = patch_timing_overruns(state["ros2_arch_file"], output_dir, warnings_by_component)
    state["patched_timing"] = patched_timing

    # Only write timeout_errors that met the patch threshold into the ledger.
    ledger_path = os.path.join(output_dir, "error_ledger.json")
    patched_timeout_errors = [
        e for e in timeout_errors
        if any(comp in e.get("exception_message", "") for comp in patched_timing)
    ]
    if merged_errors + patched_timeout_errors:
        update_error_ledger(ledger_path, merged_errors + patched_timeout_errors, console)

    return state


def architect_convert_tool(console, state: SystemState) -> Dict:
    """
    Architecture convert tool
    
    Args:
        console: Rich console for logging
        state: Current system state
        
    Returns:
        Updated state with ROS2 architecture JSON
    """
    ros2_arch_file = state["ros2_arch_file"]
    # Run architect agent
    architect_convert_script = os.path.join(os.path.dirname(os.path.dirname(__file__)), "architect_convert.py")
    command = [
        sys.executable, architect_convert_script,
        "-a", state["xml_file_path"],
        "-o", ros2_arch_file,
    ]
    console.print(f"execute command: {command}")
    subprocess.run(command, check=True, cwd=os.getcwd())
    if not os.path.exists(ros2_arch_file):
        console.print("[bold red]Error:[/bold red] Architect agent completed but no output file was generated")
        sys.exit(1)
    console.print("[bold green]Success:[/bold green] Architect agent completed successfully")
    console.print(f"Generated ROS2 architecture JSON: [green]{ros2_arch_file}[/green]")
    # Update state
    state["current_node"] = "architect_agent"
    console.print(f"Current node: [green]{state['current_node']}[/green]")

    return state    


def coder_agent(
    console,
    state: SystemState,
    only_components: Optional[List[str]] = None,
    only_nodes: Optional[List[str]] = None,
    only_other_codes: Optional[List[str]] = None,
    only_shared_sim_state: bool = False,
    error_context: str = None,
    config_only: bool = False,
) -> Dict:
    """
    Code generation agent (C++). Incremental regen can target components, nodes, other_codes, or config files.
    """
    ros2_arch_file = state["ros2_arch_file"]
    # Run coder agent
    project_root = os.path.dirname(os.path.dirname(__file__))
    coder_agent_script = os.path.join(
        project_root, "baselines", "baseline", "coder_agent_group3.py"
    )
    command = [
        sys.executable, coder_agent_script,
        "-r", ros2_arch_file,
        "-k", state["api_key"],
        "-o", state["output_dir"],
    ]
    if only_components:
        command += ["--only-components", ",".join(only_components)]
    if only_nodes:
        command += ["--only-nodes", ",".join(only_nodes)]
    if only_other_codes:
        command += ["--only-other-codes", ",".join(only_other_codes)]
    if only_shared_sim_state:
        command += ["--only-shared-sim-state"]
    if error_context:
        command += ["--error_context", error_context]
    if config_only:
        command += ["--config-only"]
    console.print(f"execute command: {command}")
    # DEVNULL: avoid workers blocking on inherited stdin when the CLI is run from an IDE/pipe.
    subprocess.run(command, check=True, cwd=os.getcwd(), stdin=subprocess.DEVNULL)
    console.print("[bold green]Success:[/bold green] Coder agent completed successfully")
    console.print(f"Generated ROS2 code: [green]{state['output_dir']}[/green]")
    # Update state
    state["current_node"] = "coder_agent"
    console.print(f"Current node: [green]{state['current_node']}[/green]")

    return state