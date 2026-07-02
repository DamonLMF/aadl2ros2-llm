# AADL2ROS2-LLM

An automated pipeline that converts **AADL** (Architecture Analysis & Design Language) models into **ROS 2 Jazzy** C++ workspaces. The tool combines deterministic parsing and Jinja2 templates with LLM-assisted code generation, then closes the loop with dynamic build/run testing and targeted repair.

## Overview

AADL captures system structure, timing, behavior annexes, and legacy source bindings. This project turns those models into runnable ROS 2 packages while preserving architectural intent:

- **Deterministic front-end** — recursive AADL parsing, behavior-annex extraction, and rule-based mapping to a ROS 2 architecture contract (topics, QoS, timing, state machines, shared variables, bundled C/Ada sources).
- **Hybrid code generation** — Jinja2 templates emit package scaffolding, headers, node mains, and CMake/`package.xml`; an LLM fills only the component `control_loop` bodies and converts `other_codes` when needed.
- **Closed-loop validation** — `colcon build`, short runtime soak tests, log parsing, runtime behavior analysis (including FSM and timing checks), and incremental regeneration guided by an error ledger.

## Pipeline

```
AADL model
    │
    ▼
┌─────────────────┐
│  AADL parser    │  → intermediate JSON / XML
└────────┬────────┘
         ▼
┌─────────────────┐
│ Architect       │  → <System>_ros.json (ROS 2 architecture contract)
└────────┬────────┘
         ▼
┌─────────────────┐
│ Coder agent     │  → ROS 2 workspace (packages, components, nodes)
└────────┬────────┘
         ▼
┌─────────────────┐
│ Dynamic tester  │  → colcon build + run main nodes → ros_info/node.log
└────────┬────────┘
         ▼
┌─────────────────┐
│ Error analysis  │  → compile / runtime / behavior errors
│ + repair loop   │  → targeted LLM regen (components, other_codes, or neighbors)
└─────────────────┘
```

The main entry point `aadl2ros2_agent_cli.py` orchestrates all phases. After code generation, it iterates dynamic testing and repair (up to five rounds) until errors are resolved, timing thresholds are patched, or retry limits are reached.

## Features

- Cross-file, case-insensitive AADL parsing (systems, processes, threads, devices, ports, connections, properties, annexes).
- Behavior Specification annex → ROS-oriented state machines and variables.
- Automatic discovery and bundling of unreferenced `.c` / `.h` sources as `other_codes`.
- Template-driven ROS 2 package layout with QoS derived from AADL port kinds.
- LLM generation scoped to `control_loop` logic (reduces structural hallucination).
- Dynamic testing via `colcon` with optional virtual publishers for uncovered inputs (`--inject-virtual-io`).
- Runtime analysis comparing logs against the architecture contract (topic flow, FSM transitions, timing warnings).
- Incremental repair: regenerate only failing components, bundled sources, or topic neighbors on repeated behavior errors.
- Persistent **error ledger** (`error_ledger.json`) that injects learned rules into subsequent repair prompts.

## Requirements

| Component | Version / notes |
|-----------|-----------------|
| Python | 3.10+ recommended |
| ROS 2 | **Jazzy** (default; override with `ROS_DISTRO`) |
| Build tools | `colcon`, a C++17 compiler |
| LLM API | OpenAI-compatible endpoint (default: DeepSeek via `ros_generator_utils.py`) |

## Installation

```bash
git clone <repository-url>
cd aadl2ros2-llm-new

python3 -m venv .venv
source .venv/bin/activate

pip install -r requirements.txt

# Source ROS 2 (adjust path for your installation)
source /opt/ros/jazzy/setup.bash
```

Set your LLM API key before running code generation:

```bash
export DEEPSEEK_API_KEY="your-api-key"
# or pass -k / --api_key on the CLI
```

To use a different model or endpoint, edit `API_URL`, `model`, and `temperature` in `ros_generator_utils.py`.

## Quick Start

Convert the flight-controller example:

```bash
python aadl2ros2_agent_cli.py \
  -i ./example/fcc \
  -f Flight_Controller.aadl \
  -s Flight_Controller \
  -o ./output/fcc \
  -k "$DEEPSEEK_API_KEY"
```

### CLI options

| Flag | Description |
|------|-------------|
| `-i`, `--input_dir` | Directory containing AADL files (required) |
| `-f`, `--file_name` | Top-level AADL file name (required) |
| `-s`, `--system` | Root system name in the model (required) |
| `-o`, `--output_dir` | Output workspace directory (default: `./output`) |
| `-k`, `--api_key` | LLM API key (optional if set via environment) |
| `-iv`, `--inject-virtual-io` | Publish temporary topics for uncovered inputs during dynamic test |

Generated ROS 2 code lands under `<output_dir>/<package_name>/`. Logs and analysis artifacts are written to `<output_dir>/ros_info/` and `<output_dir>/runtime_analysis_report.txt`.

## Example Models

The `example/` directory ships reference AADL systems of varying complexity:

| Directory | System name | Highlights |
|-----------|-------------|------------|
| `time_triggered/` | `tt` | Time-triggered threads, Ada subprograms, shared variables |
| `ardupilot/` | `Ardupilot_Map` | Multi-thread software, C subprograms |
| `radar/` | `radar` | Device I/O, Ada code, shared state |
| `pacemaker/` | `DeviceControllerMonitor` | Behavior annex (BA) |
| `producer_consumer/` | `PC_Simple` | Multi-process, shared variables |
| `redundancy/` | `redundant_system` | Redundant BA logic |
| `regulator/` | `AirConditioner` | Multi-device BA |
| `fcc/` | `Flight_Controller` | Thread-heavy, BA + C subprograms, full timing |
| `minepump_ba/` | `MinePump` | BA + C code + shared memory |
| `robot_ba/` | `robot` | Multi-process BA |
| `doors/` | `door_management` | Large device-interaction BA |
| `rosace/` | `ROSACE_XtratuM` | Multi-process, heterogeneous sources |

Run the parser alone:

```bash
python aadl_parser/aadl_parser.py \
  -i ./example/fcc \
  -f Flight_Controller.aadl \
  -s Flight_Controller \
  -o ./output/fcc
```

## Project Structure

```
aadl2ros2-llm-new/
├── aadl2ros2_agent_cli.py   # Main orchestrator CLI
├── architect_convert.py     # AADL JSON → ROS 2 architecture contract
├── coder_agent.py           # Template + LLM code generation
├── coder_template.py        # Jinja2 context and deterministic rendering helpers
├── ros_generator_utils.py   # LLM client, memory, and shared generator utilities
├── test_agent.py            # colcon build + runtime soak test driver
├── aadl_parser/             # AADL recursive parser and XML/JSON export
│   ├── aadl_parser.py
│   ├── behavior_parser.py
│   ├── aadl_to_xml_converter.py
│   └── other_sources_scan.py
├── planner/                 # Pipeline nodes and shared state
│   ├── nodes.py
│   └── system_state.py
├── validator/               # Log parsing, runtime analysis, CBA metrics
│   ├── error_analysis.py
│   ├── runtime_analysis.py
│   └── cba_metrics.py
├── templates/               # Jinja2 templates for ROS 2 C++ artifacts
├── example/                 # Reference AADL models and legacy source code
└── requirements.txt
```

## Standalone Tools

Each pipeline stage can be invoked independently:

```bash
# 1. Parse AADL → JSON
python aadl_parser/aadl_parser.py -i <dir> -f <file> -s <system> -o <out>

# 2. Build ROS 2 architecture contract
python architect_convert.py -a <system>.json -o <system>_ros.json

# 3. Generate ROS 2 workspace
python coder_agent.py -r <system>_ros.json -o <workspace> -k <api_key>

# 4. Dynamic build and runtime test
python test_agent.py -p <workspace> --phase-runs 3

# 5. Runtime behavior analysis (after a test run)
python validator/runtime_analysis.py --log <workspace>/ros_info/node.log \
  --arch <workspace>/<system>_ros.json
```

`coder_agent.py` supports incremental regeneration:

```bash
python coder_agent.py -r arch.json -o ./output -k "$KEY" \
  --only-components thread_a,thread_b

python coder_agent.py -r arch.json -o ./output -k "$KEY" \
  --only-other-codes simu.c --error_context "fix namespace conflicts"
```

## Output Artifacts

After a full CLI run, expect:

| Path | Description |
|------|-------------|
| `<system>.json` | Parsed AADL intermediate representation |
| `<system>_ros.json` | ROS 2 architecture contract |
| `<package>/` | Generated colcon workspace packages |
| `ros_info/node.log` | Build and runtime log |
| `ros_info/errors_history.json` | Per-iteration error snapshots |
| `ros_info/iterations/iter_NN/` | Per-repair-iteration artifacts |
| `runtime_analysis_report.txt` | Behavior / timing analysis report |
| `error_ledger.json` | Accumulated repair rules for the LLM |

## Repair Strategy

When dynamic testing reports errors, the CLI applies scoped fixes:

1. **Compile errors in `other_codes`** — regenerate only the affected bundled C/C++ sources.
2. **Component errors** — regenerate named components; on repeated `BehaviorError`, expand to topic neighbors.
3. **Node-level errors** — regenerate components with full error context.
4. **Timing warnings** — patch period/deadline thresholds in the architecture JSON when criteria are met, then re-test.

Each error signature is tracked; unresolved issues after two repair attempts stop the loop with a diagnostic summary.

## Limitations

- Targets **ROS 2 Jazzy** C++; other distros may need template or API adjustments.
- LLM quality and API availability directly affect `control_loop` and `other_codes` conversion.
- Complex device-level behavior annexes (e.g., large door-controller models) may require custom test stimuli.
- The default LLM configuration points to DeepSeek; switch providers in `ros_generator_utils.py`.

## License

License terms are not specified in this repository. Contact the maintainers before redistribution.
