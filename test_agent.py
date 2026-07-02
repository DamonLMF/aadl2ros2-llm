#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamic test driver: colcon build, then run main ROS 2 nodes only (no test/injector nodes).
Logs build + runtime to <workspace>/ros_info/node.log. Process exit code is always 0; use node.log for outcomes.
"""

import argparse
import json
import logging
import os
import shlex
import signal
import subprocess
import textwrap
import time
from typing import Any, Dict, List, Optional, Tuple

from coder_template import qos_to_ros2_pub_cli_flags

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

_STARTUP_GAP_SECONDS = 1.0  # wait for the nodes to start
_POST_START_STABILIZE_SECONDS = 3.0  # wait for the nodes to stabilize
_SOAK_SECONDS = 20.0  # run the nodes for 20 seconds
_STOP_GAP_SECONDS = 0.3  # wait for the nodes to stop


def _ros_distro() -> str:
    return os.environ.get("ROS_DISTRO", "jazzy")


def _main_nodes_from_src(workspace_root: str) -> List[Tuple[str, str]]:
    """
    Scan <workspace>/<package>/src for *_node.cpp and return
    (package_name, executable_name) pairs for ros2 launch/run.
    """
    out: List[Tuple[str, str]] = []
    for pkg_name in sorted(os.listdir(workspace_root)):
        pkg_dir = os.path.join(workspace_root, pkg_name)
        if not os.path.isdir(pkg_dir):
            continue
        src_dir = os.path.join(pkg_dir, "src")
        if not os.path.isdir(src_dir):
            continue
        for filename in sorted(os.listdir(src_dir)):
            if filename.endswith("_node.cpp"):
                exe = filename[:-4]  # strip .cpp
                out.append((pkg_name.lower(), exe))
    return out


def _normalize_ros_msg_type(msg_type: str) -> str:
    t = (msg_type or "").strip()
    if not t:
        return ""
    return t.replace("::", "/")


def _discover_virtual_input_topics(
    workspace_root: str,
) -> List[Tuple[str, str, Optional[Dict[str, Any]]]]:
    """
    Discover input topics that have subscribers but no in-graph publishers.
    Returns (topic, message_type, qos) with qos taken from architecture JSON.
    Conservative filter: keep std_msgs String/Float32/Int32/Bool only.
    """
    arch_path = None
    for fn in sorted(os.listdir(workspace_root)):
        if fn.endswith("_ros.json"):
            arch_path = os.path.join(workspace_root, fn)
            break
    if not arch_path:
        logger.info("No *_ros.json found under %s; skip virtual I/O discovery", workspace_root)
        return []
    try:
        with open(arch_path, "r", encoding="utf-8") as f:
            arch = json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        logger.warning("Failed to read architecture JSON %s: %s", arch_path, e)
        return []

    published_topics = set()
    subscriber_specs: Dict[str, Tuple[str, Optional[Dict[str, Any]]]] = {}

    def _register_subscriber(topic: str, message_type: str, qos: Any) -> None:
        mtype = _normalize_ros_msg_type(message_type)
        qos_dict = qos if isinstance(qos, dict) else None
        prev = subscriber_specs.get(topic)
        if prev is None:
            subscriber_specs[topic] = (mtype, qos_dict)
            return
        if prev[0] != mtype or prev[1] != qos_dict:
            logger.warning(
                "Conflicting subscriber spec for %s (keeping first): %s vs %s",
                topic,
                prev,
                (mtype, qos_dict),
            )

    for pkg in arch.get("ROSPackages", []) if isinstance(arch, dict) else []:
        for node in pkg.get("nodes", []) or []:
            for pub in node.get("publishers", []) or []:
                if isinstance(pub, dict) and pub.get("topic"):
                    published_topics.add(str(pub.get("topic")))
            for sub in node.get("subscribers", []) or []:
                if isinstance(sub, dict) and sub.get("topic"):
                    _register_subscriber(
                        str(sub.get("topic")),
                        str(sub.get("message_type", "")),
                        sub.get("qos"),
                    )
            for comp in node.get("components", []) or []:
                for out in comp.get("outputs", []) or []:
                    if isinstance(out, dict) and out.get("topic"):
                        published_topics.add(str(out.get("topic")))
                for cb in comp.get("callbacks", []) or []:
                    if isinstance(cb, dict) and cb.get("topic"):
                        _register_subscriber(
                            str(cb.get("topic")),
                            str(cb.get("message_type", "")),
                            cb.get("qos"),
                        )

    out: List[Tuple[str, str, Optional[Dict[str, Any]]]] = []
    for topic, (mtype, qos) in sorted(subscriber_specs.items()):
        if topic in published_topics:
            continue
        key = _normalize_ros_msg_type(mtype).lower()
        if key == "std_msgs/msg/string":
            out.append((topic, "std_msgs/msg/String", qos))
        elif key == "std_msgs/msg/float32":
            out.append((topic, "std_msgs/msg/Float32", qos))
        elif key == "std_msgs/msg/int32":
            out.append((topic, "std_msgs/msg/Int32", qos))
        elif key == "std_msgs/msg/bool":
            out.append((topic, "std_msgs/msg/Bool", qos))
    logger.info("Virtual I/O candidate topics (with qos): %s", out)
    return out


def _virtual_pub_data_for_msg_type(msg_type: str) -> str:
    if msg_type == "std_msgs/msg/Float32":
        # 1..999 → 0.001..0.999; never RANDOM%1000==0 → "0." parsed as 0.0
        return '\\"{data: 0.$(( (RANDOM % 999) + 1 ))}\\"'
    if msg_type == "std_msgs/msg/Int32":
        return '\\"{data: $((RANDOM % 101))}\\"'
    if msg_type == "std_msgs/msg/Bool":
        return '\\"{data: $([ $((RANDOM % 2)) -eq 0 ] && echo -n true || echo -n false)}\\"'
    return '\\"{data: \\"$((RANDOM % 101))\\"}"'


def _start_virtual_publishers(
    workspace_root: str,
    distro: str,
    topic_specs: List[Tuple[str, str, Optional[Dict[str, Any]]]],
    phase_log_name: str,
) -> List[subprocess.Popen]:
    """One bash: each cycle publishes all topics in parallel, then sleeps."""
    if not topic_specs:
        return []
    pub_cmds: List[str] = []
    for topic, msg_type, qos in topic_specs:
        qos_flags = qos_to_ros2_pub_cli_flags(qos)
        # Escape $ so outer sh (Popen shell=True) does not expand; bash -lc re-expands each loop.
        data = _virtual_pub_data_for_msg_type(msg_type).replace("$", "\\$")
        pub_cmds.append(
            f"ros2 topic pub --once {qos_flags} {shlex.quote(topic)} "
            f"{shlex.quote(msg_type)} {data} 2>/dev/null"
        )
    burst = " & ".join(pub_cmds) + "; wait"
    cmd = (
        f'/bin/bash -lc "source /opt/ros/{distro}/setup.bash && '
        f"source install/setup.bash && "
        f"while true; do {burst}; sleep 0.1; done\""
    )
    p = run_command_in_new_terminal(
        cmd,
        cwd=workspace_root,
        node_type="virtual_io",
        append_log=True,
        log_name=phase_log_name,
    )
    return [p] if p is not None else []


def _resolve_main_nodes_to_run(workspace_root: str) -> List[Tuple[str, str]]:
    nodes = _main_nodes_from_src(workspace_root)
    logger.info("Resolved main nodes from src scan: %s", nodes)
    return nodes


def _write_generated_launch_file(
    workspace_root: str,
    nodes: List[Tuple[str, str]],
    launch_filename: str = "generated_test_launch.py",
) -> str:
    """Create a temporary launch file under launch/ (sibling of src/)."""
    launch_dir = os.path.join(workspace_root, "launch")
    if nodes:
        pkg_dir = os.path.join(workspace_root, nodes[0][0])
        if os.path.isdir(os.path.join(pkg_dir, "src")):
            launch_dir = os.path.join(pkg_dir, "launch")
    os.makedirs(launch_dir, exist_ok=True)
    launch_path = os.path.join(launch_dir, launch_filename)
    node_blocks = []
    for pkg, exe in nodes:
        node_blocks.append(
            f'        Node(package="{pkg}", executable="{exe}", output="screen"),'
        )
    launch_code = textwrap.dedent(
        f"""\
        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
        {os.linesep.join(node_blocks)}
            ])
        """
    )
    with open(launch_path, "w", encoding="utf-8") as f:
        f.write(launch_code)
    return launch_path


def run_command_in_new_terminal(
    cmd: str,
    cwd: Optional[str] = None,
    node_type: str = "default",
    append_log: bool = False,
    log_name: str = "node.log",
):
    """
    Run cmd in a new session; stdout/stderr go to <cwd>/ros_info/<log_name>.
    If append_log is True, append instead of truncating the log file.
    """
    logger.info("Executing command: %s", cmd)
    if cwd:
        logger.info("Working directory: %s", cwd)

    ros_info_dir = os.path.join(cwd, "ros_info")
    os.makedirs(ros_info_dir, exist_ok=True)
    log_file = os.path.join(ros_info_dir, log_name)
    logger.info("%s output -> %s", node_type, log_file)

    try:
        abs_log_file = os.path.abspath(log_file)
        redir = ">>" if append_log else ">"
        full_cmd = f"cd {shlex.quote(cwd)} && {cmd} {redir} {shlex.quote(abs_log_file)} 2>&1"
        process = subprocess.Popen(full_cmd, shell=True, start_new_session=True)
        logger.info("%s started, pid=%s", node_type, process.pid)
        return process
    except Exception as e:
        logger.error("%s failed to start: %s", node_type, e)
        return None


def _stop_process_group_gracefully(process: subprocess.Popen) -> None:
    """Best-effort graceful stop for ROS nodes: SIGINT -> SIGTERM -> SIGKILL."""
    try:
        pgid = os.getpgid(process.pid)
    except Exception:
        return

    if process.poll() is not None:
        return

    try:
        logger.info("Stopping process group pid=%s with SIGINT", process.pid)
        os.killpg(pgid, signal.SIGINT)
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        logger.warning("pid=%s did not exit after SIGINT; escalating to SIGTERM", process.pid)
    except Exception as e:
        logger.error("Failed SIGINT stop for pid=%s: %s", process.pid, e)
        return

    if process.poll() is not None:
        return
    try:
        os.killpg(pgid, signal.SIGTERM)
        process.wait(timeout=3)
        return
    except subprocess.TimeoutExpired:
        logger.warning("pid=%s did not exit after SIGTERM; escalating to SIGKILL", process.pid)
    except Exception as e:
        logger.error("Failed SIGTERM stop for pid=%s: %s", process.pid, e)
        return

    if process.poll() is not None:
        return
    try:
        os.killpg(pgid, signal.SIGKILL)
        process.wait(timeout=1)
    except Exception as e:
        logger.error("Failed SIGKILL stop for pid=%s: %s", process.pid, e)


def _append_phase_log(
    workspace_root: str,
    phase_log_name: str,
    phase_index: int,
    final_log_name: str = "node.log",
) -> None:
    ros_info_dir = os.path.join(workspace_root, "ros_info")
    node_log = os.path.join(ros_info_dir, final_log_name)
    phase_log = os.path.join(ros_info_dir, phase_log_name)
    try:
        with open(node_log, "a", encoding="utf-8") as out:
            out.write(f"\n=== dynamic test phase {phase_index}: {phase_log_name} ===\n")
            with open(phase_log, "r", encoding="utf-8", errors="replace") as src:
                out.write(src.read())
    except OSError as e:
        logger.warning("Failed to append %s into node.log: %s", phase_log_name, e)


def _cleanup_phase_logs(
    workspace_root: str,
    phase_runs: int,
    phase_prefix: str = "node_phase_",
) -> None:
    """Delete per-phase logs after they are merged into the final node log."""
    ros_info_dir = os.path.join(workspace_root, "ros_info")
    for phase_index in range(1, phase_runs + 1):
        phase_log = os.path.join(ros_info_dir, f"{phase_prefix}{phase_index}.log")
        try:
            if os.path.exists(phase_log):
                os.remove(phase_log)
        except OSError as e:
            logger.warning("Failed to remove %s: %s", phase_log, e)


def run_workspace_test(
    workspace_root: str,
    phase_runs: int = 2,
    phase_step: float = 0.0,
    inject_virtual_io: bool = False,
    fc_scenario_path: Optional[str] = None,
) -> bool:
    """
    colcon build once, then source install/setup.bash and run generated launch.
    Multiple phases vary the stabilization delay to improve FSM edge coverage.

    When fc_scenario_path is set, run functional-correctness mode: optional node
    exclusion, fixed inputs from scenario JSON, no random virtual I/O.
    """
    workspace_root = os.path.abspath(workspace_root)
    if not os.path.isdir(workspace_root):
        logger.error("Workspace path does not exist: %s", workspace_root)
        return False

    fc_scenario: Optional[Dict[str, Any]] = None
    fc_io = None
    fc_ros_domain_id: Optional[int] = None
    soak_seconds = _SOAK_SECONDS
    final_log_name = "node.log"
    build_log_name = "node.log"
    launch_filename = "generated_test_launch.py"
    phase_prefix = "node_phase_"
    if fc_scenario_path:
        from functional_tests import fc_io as _fc_io

        fc_io = _fc_io
        fc_cfg = fc_io.configure(os.path.abspath(fc_scenario_path), _SOAK_SECONDS)
        fc_scenario = fc_cfg["scenario"]
        phase_runs = fc_cfg["phase_runs"]
        soak_seconds = fc_cfg["soak_seconds"]
        inject_virtual_io = fc_cfg["inject_virtual_io"]
        final_log_name = fc_cfg["final_log_name"]
        build_log_name = fc_cfg["build_log_name"]
        launch_filename = fc_cfg["launch_filename"]
        phase_prefix = fc_cfg["phase_prefix"]
        fc_ros_domain_id = fc_io.ros_domain_id(workspace_root)
        fc_io.kill_stale_workspace_ros(workspace_root)
        logger.info(
            "=== FC scenario mode: %s (ROS_DOMAIN_ID=%s) ===",
            fc_scenario_path,
            fc_ros_domain_id,
        )

    logger.info("=== Dynamic test workspace: %s ===", workspace_root)

    try:
        build_process = run_command_in_new_terminal(
            "colcon build",
            cwd=workspace_root,
            node_type="build",
            append_log=False,
            log_name=build_log_name,
        )
        if build_process is None:
            return False
        build_process.wait()
        if build_process.returncode != 0:
            log_path = os.path.join(workspace_root, "ros_info", final_log_name)
            try:
                with open(log_path, "a", encoding="utf-8") as lf:
                    lf.write(f"\n=== colcon build failed (exit {build_process.returncode}) ===\n")
            except OSError:
                pass
            logger.info("colcon build failed; see %s", log_path)
            return False

        distro = _ros_distro()
        domain_prefix = ""
        if fc_ros_domain_id is not None:
            domain_prefix = f"export ROS_DOMAIN_ID={fc_ros_domain_id} && "
        setup_cmd = (
            f"/bin/bash -lc '{domain_prefix}source /opt/ros/{distro}/setup.bash && "
            f"source install/setup.bash && "
        )
        setup_cmd_end = "'"

        to_run = _resolve_main_nodes_to_run(workspace_root)
        if fc_scenario:
            to_run = fc_io.filter_nodes(to_run, fc_scenario)
        if not to_run:
            logger.error("No main nodes to run after FC filtering")
            return False

        launch_file = _write_generated_launch_file(
            workspace_root, to_run, launch_filename=launch_filename
        )
        launch_cmd = f"{setup_cmd}ros2 launch {shlex.quote(launch_file)}{setup_cmd_end}"
        phase_runs = max(1, int(phase_runs))
        phase_step = max(0.0, float(phase_step))
        topic_specs: List[Tuple[str, str, Optional[Dict[str, Any]]]] = []
        if inject_virtual_io and not fc_scenario:
            topic_specs = _discover_virtual_input_topics(workspace_root)
            logger.info("Virtual I/O injection enabled, topics=%s", topic_specs)
        for phase_index in range(1, phase_runs + 1):
            stabilize_seconds = _POST_START_STABILIZE_SECONDS + (phase_index - 1) * phase_step
            phase_log_name = f"{phase_prefix}{phase_index}.log"
            logger.info("Start launch phase %s/%s: %s", phase_index, phase_runs, launch_file)
            launch_process = run_command_in_new_terminal(
                launch_cmd,
                cwd=workspace_root,
                node_type=f"launch_phase_{phase_index}",
                append_log=False,
                log_name=phase_log_name,
            )
            if launch_process is None:
                return False
            virtual_procs: List[subprocess.Popen] = []
            if fc_scenario and fc_scenario.get("inputs"):
                virtual_procs = fc_io.start_publishers(
                    workspace_root,
                    distro,
                    fc_scenario,
                    phase_log_name,
                    run_command_in_new_terminal,
                    ros_domain_id=fc_ros_domain_id,
                )
            elif topic_specs:
                virtual_procs = _start_virtual_publishers(
                    workspace_root=workspace_root,
                    distro=distro,
                    topic_specs=topic_specs,
                    phase_log_name=phase_log_name,
                )

            try:
                time.sleep(_STARTUP_GAP_SECONDS)
                logger.info(
                    "Launch phase %s started; stabilizing for %.2f seconds before soak...",
                    phase_index,
                    stabilize_seconds,
                )
                time.sleep(stabilize_seconds)

                logger.info(
                    "Launch phase %s running (%s); soaking %.1f seconds then graceful stop (SIGINT->SIGTERM->SIGKILL)...",
                    phase_index,
                    ", ".join(f"{p}/{e}" for p, e in to_run),
                    soak_seconds,
                )
                time.sleep(soak_seconds)
            finally:
                for vp in virtual_procs:
                    _stop_process_group_gracefully(vp)
                _stop_process_group_gracefully(launch_process)
                time.sleep(_STOP_GAP_SECONDS)
                _append_phase_log(
                    workspace_root, phase_log_name, phase_index, final_log_name=final_log_name
                )

        _cleanup_phase_logs(workspace_root, phase_runs, phase_prefix=phase_prefix)
        if fc_scenario:
            logger.info(
                "FC run log (does not overwrite node.log): %s",
                os.path.join(workspace_root, "ros_info", final_log_name),
            )
        logger.info("Workspace test finished: %s", workspace_root)
        return True

    except Exception as e:
        logger.error("Workspace test exception: %s", e)
        return False


def run_version_tests(
    version: int,
    base_path: str,
    phase_runs: int = 2,
    phase_step: float = 0.0,
    inject_virtual_io: bool = False,
    fc_scenario_path: Optional[str] = None,
) -> bool:
    """Legacy batch layout: base_path/fcc_code_v{version} (or base_path when version==0)."""
    if version == 0:
        version_path = base_path
    else:
        version_path = os.path.join(base_path, f"fcc_code_v{version}")
    if not os.path.exists(version_path):
        logger.error("Version path does not exist: %s", version_path)
        return False
    logger.info("=== Legacy batch test version v%s -> %s ===", version, version_path)
    return run_workspace_test(
        version_path,
        phase_runs=phase_runs,
        phase_step=phase_step,
        inject_virtual_io=inject_virtual_io,
        fc_scenario_path=fc_scenario_path,
    )


def main():
    parser = argparse.ArgumentParser(description="Dynamic test: colcon build + main ROS 2 nodes only")
    parser.add_argument("-v", "--versions", type=str, default="", help="Version range, e.g. 1-5 or 1,2,3")
    parser.add_argument("-p", "--base-path", type=str, default="./output", help="Workspace /output directory")
    parser.add_argument("--phase-runs", type=int, default=2, help="Number of launch phases after one build")
    parser.add_argument(
        "--phase-step",
        type=float,
        default=0.0,
        help="Extra stabilization seconds added per phase (0.0 means all phases use the same duration)",
    )
    parser.add_argument(
        "--inject-virtual-io",
        action="store_true",
        help="Inject temporary virtual String publishers for uncovered input topics during dynamic test",
    )
    parser.add_argument(
        "--fc-scenario",
        type=str,
        default="",
        help="Path to functional_tests scenario.json (fixed inputs, exclude device nodes)",
    )
    args = parser.parse_args()

    fc_scenario_path = args.fc_scenario.strip() or None

    if args.versions and "-" in args.versions:
        start, end = map(int, args.versions.split("-"))
        versions = list(range(start, end + 1))
    elif args.versions:
        versions = list(map(int, args.versions.split(",")))
    else:
        versions = []

    base_path = os.path.abspath(args.base_path)
    results = {}
    if versions:
        logger.info("Batch versions: %s", versions)
        for v in versions:
            results[v] = run_version_tests(
                v,
                base_path,
                args.phase_runs,
                args.phase_step,
                inject_virtual_io=args.inject_virtual_io,
                fc_scenario_path=fc_scenario_path,
            )
    else:
        results[0] = run_version_tests(
            0,
            base_path,
            args.phase_runs,
            args.phase_step,
            inject_virtual_io=args.inject_virtual_io,
            fc_scenario_path=fc_scenario_path,
        )

    logger.info("\n=== Test Results Summary ===")
    for version, result in results.items():
        logger.info("Version v%s: %s", version, "Success" if result else "Failure")

    success_count = sum(results.values())
    fail_count = len(results) - success_count
    logger.info("Total: %s, Success: %s, Failure: %s", len(results), success_count, fail_count)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
