#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AADL to ROS2 conversion CLI tool (RQ2 group2 baseline — no closed-loop repair).
usage: python aadl2ros2_agent_cli_group2.py -i ./example/fcc -f Flight_Controller.aadl -s Flight_Controller -o ./output -k your_api_key
"""

import os
import shutil
import sys

from rich.console import Console
from rich.panel import Panel
from rich.text import Text

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from planner.system_state import SystemState
from planner.nodes import (
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
    welcome_text = Text("AADL2ROS2 Agent CLI", style="bold cyan")
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

    parser = argparse.ArgumentParser(
        description="AADL to ROS2 conversion tool using multi-agent system (group2 baseline, no repair loop)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
        Examples:
        python aadl2ros2_agent_cli_group2.py -i ./example/fcc -f Flight_Controller.aadl -s Flight_Controller
        -o ./output -k your_api_key
        """,
    )

    parser.add_argument("-i", "--input_dir", type=str, required=True, help="Directory containing AADL files (required)")
    parser.add_argument("-f", "--file_name", type=str, required=True, help="AADL model file name (required)")
    parser.add_argument("-s", "--system", type=str, required=True, help="Top-level AADL system name (required)")
    parser.add_argument("-o", "--output_dir", type=str, default="./output", help="Output directory for generated ROS2 code (default: ./output)")
    parser.add_argument("-k", "--api_key", type=str, default=None, help="LLM API key for code generation (optional, can also use DEEPSEEK_API_KEY env variable)")
    parser.add_argument("-iv", "--inject-virtual-io", action="store_true", help="During dynamic test only, start temporary virtual topic publishers for uncovered process inputs")
    args = parser.parse_args()
    console = Console()

    console.print(create_welcome_panel())

    initial_state = create_initial_state(args)

    updated_state = aadl_parser_tool(console, initial_state)

    updated_state = architect_convert_tool(console, updated_state)
    
    updated_state = coder_agent(console, updated_state)
    if not _has_generated_main_nodes(updated_state["output_dir"]):
        console.print("[bold yellow]No *_node.cpp under output; skipping dynamic test phase.[/bold yellow]")
        return

    console.print("[bold cyan]Dynamic test iteration 1/1[/bold cyan]")
    updated_state = dynamic_tester_tool(console, updated_state)
    snapshot_iteration_metrics(updated_state["output_dir"], 1)

    errors = updated_state.get("dynamic_test_errors") or []
    patched_timing = updated_state.get("patched_timing") or []
    if errors or patched_timing:
        console.print(
            "[bold yellow]Dynamic test finished with findings; no code regeneration (group2 baseline).[/bold yellow]"
        )
    else:
        console.print("[bold green]Dynamic test finished without findings.[/bold green]")


if __name__ == "__main__":
    main()
