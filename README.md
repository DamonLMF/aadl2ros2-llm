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


| Component   | Version / notes                                                             |
| ----------- | --------------------------------------------------------------------------- |
| Python      | 3.10+ recommended                                                           |
| ROS 2       | **Jazzy** (default; override with `ROS_DISTRO`)                             |
| Build tools | `colcon`, a C++17 compiler                                                  |
| LLM API     | OpenAI-compatible endpoint (default: DeepSeek via `ros_generator_utils.py`) |




## Installation

```bash
git clone <repository-url>
cd aadl2ros2-llm

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

## Usage

`aadl2ros2_agent_cli.py` is the single entry point. It runs the full pipeline: AADL parsing → architecture conversion → code generation → dynamic testing → error analysis and repair (up to five rounds).

```bash
python aadl2ros2_agent_cli.py \
  -i ./example/fcc \
  -f Flight_Controller.aadl \
  -s Flight_Controller \
  -o ./output/fcc \
  -k "$DEEPSEEK_API_KEY"
```



### CLI options


| Flag                         | Description                                                       |
| ---------------------------- | ----------------------------------------------------------------- |
| `-i`, `--input_dir`          | Directory containing AADL files (required)                        |
| `-f`, `--file_name`          | Top-level AADL file name (required)                               |
| `-s`, `--system`             | Root system name in the model (required)                          |
| `-o`, `--output_dir`         | Output workspace directory (default: `./output`)                  |
| `-k`, `--api_key`            | LLM API key (optional if `DEEPSEEK_API_KEY` is set)               |
| `-iv`, `--inject-virtual-io` | Publish temporary topics for uncovered inputs during dynamic test |


Generated ROS 2 code lands under `<output_dir>/<package_name>/`. Logs and analysis artifacts are written to `<output_dir>/ros_info/` and `<output_dir>/runtime_analysis_report.txt`.

## Baselines and Ablation Studies

The `baselines/` directory provides alternate pipeline entry points for controlled experiments. All variants share the same AADL parser, architect conversion, and dynamic tester; they differ in **code generation strategy** and **whether the closed-loop repair loop runs**.


| Variant                   | Entry point                                        | Code generation                                                                                                                                                                                                                 | Repair loop                                                                          |
| ------------------------- | -------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| **Full system** (default) | `aadl2ros2_agent_cli.py`                           | Hybrid — Jinja2 templates for headers, node mains, and CMake; LLM fills only `control_loop` bodies (`coder_agent.py`)                                                                                                           | Yes — up to 5 rounds                                                                 |
| **Baseline group3** (RQ3) | `baselines/baseline/aadl2ros2_agent_cli_group3.py` | LLM end-to-end — components (`.hpp` + `.cpp`), process nodes, device nodes, `shared_sim_state.hpp`, and `other_codes` via LLM (`baselines/baseline/coder_agent_group3.py`); templates only for `CMakeLists.txt` / `package.xml` | Yes — up to 5 rounds                                                                 |
| **Ablation group2** (RQ2) | `baselines/ablation/aadl2ros2_agent_cli_group2.py` | Same hybrid strategy as the full system (`coder_agent.py`)                                                                                                                                                                      | **No** — one dynamic test pass only; errors are reported but code is not regenerated |




### Baseline group3 (RQ3)

Use this variant to compare **template-scaffolded + LLM** `control_loop` against **LLM-generated component headers, sources, and node wrappers**. The repair loop matches the full system (component / node / `other_codes` / `shared_sim_state.hpp` scoped regen, error ledger, neighbor expansion on repeated behavior errors).

```bash
python3 baselines/baseline/aadl2ros2_agent_cli_group3.py \
  -i ./example/time_triggered \
  -f time_triggered.aadl \
  -s tt \
  -o ./output/tt \
  -k your_api_key
```

Optional flags are identical to the main CLI (`--inject-virtual-io`, etc.).

### Ablation group2 (RQ2)

Use this variant to measure the impact of **closed-loop repair** while keeping hybrid code generation fixed. The pipeline runs parse → architect → codegen → **one** dynamic test; findings are logged under `ros_info/` but no LLM repair is attempted.

```bash
python3 baselines/ablation/aadl2ros2_agent_cli_group2.py \
  -i ./example/fcc \
  -f Flight_Controller.aadl \
  -s Flight_Controller \
  -o ./output/fcc_group2 \
  -k your_api_key
```



### Baselines layout

```
baselines/
├── baseline/
│   ├── aadl2ros2_agent_cli_group3.py   # RQ3 orchestrator (uses planner/nodes_group3.py)
│   └── coder_agent_group3.py           # LLM end-to-end code generator
└── ablation/
    └── aadl2ros2_agent_cli_group2.py   # RQ2 orchestrator (no repair loop)
```

For fair comparison across variants, use the same `-i`, `-f`, `-s`, and `-o` arguments and the same LLM API key; only the entry-point script changes.

## Example Models

The `example/` directory ships reference AADL systems of varying complexity:


| Directory            | System name               | Highlights                                                |
| -------------------- | ------------------------- | --------------------------------------------------------- |
| `time_triggered/`    | `tt`                      | Time-triggered threads, Ada subprograms, shared variables |
| `ardupilot/`         | `Ardupilot_Map`           | Multi-thread software, C subprograms                      |
| `radar/`             | `radar`                   | Device I/O, Ada code, shared state                        |
| `pacemaker/`         | `DeviceControllerMonitor` | Behavior annex (BA)                                       |
| `producer_consumer/` | `PC_Simple`               | Multi-process, shared variables                           |
| `redundancy/`        | `redundant_system`        | Redundant BA logic                                        |
| `regulator/`         | `AirConditioner`          | Multi-device BA                                           |
| `fcc/`               | `Flight_Controller`       | Thread-heavy, BA + C subprograms, full timing             |
| `minepump_ba/`       | `MinePump`                | BA + C code + shared memory                               |
| `robot_ba/`          | `robot`                   | Multi-process BA                                          |
| `doors/`             | `door_management`         | Large device-interaction BA                               |
| `rosace/`            | `ROSACE_XtratuM`          | Multi-process, heterogeneous sources                      |


For any example above, use the same CLI with the matching `-i`, `-f`, and `-s` values. For instance, the pacemaker model:

```bash
python aadl2ros2_agent_cli.py \
  -i ./example/pacemaker \
  -f DeviceControllerMonitor.aadl \
  -s DeviceControllerMonitor \
  -o ./output/pacemaker \
  -k "$DEEPSEEK_API_KEY"
```



## Project Structure

```
aadl2ros2-llm/
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
│   ├── nodes.py             # Nodes for full system and ablation group2
│   ├── nodes_group3.py      # Nodes for baseline group3 (LLM end-to-end coder)
│   └── system_state.py
├── baselines/               # Experimental variants (RQ2 ablation, RQ3 baseline)
│   ├── baseline/            # group3: LLM end-to-end codegen + repair loop
│   └── ablation/            # group2: hybrid codegen, no repair loop
├── validator/               # Log parsing, runtime analysis, CBA metrics
│   ├── error_analysis.py
│   ├── runtime_analysis.py
│   └── cba_metrics.py
├── templates/               # Jinja2 templates for ROS 2 C++ artifacts
├── example/                 # Reference AADL models and legacy source code
└── requirements.txt
```



## Output Artifacts

After a full CLI run, expect:


| Path                           | Description                             |
| ------------------------------ | --------------------------------------- |
| `<system>.json`                | Parsed AADL intermediate representation |
| `<system>_ros.json`            | ROS 2 architecture contract             |
| `<package>/`                   | Generated colcon workspace packages     |
| `ros_info/node.log`            | Build and runtime log                   |
| `ros_info/errors_history.json` | Per-iteration error snapshots           |
| `ros_info/iterations/iter_NN/` | Per-repair-iteration artifacts          |
| `runtime_analysis_report.txt`  | Behavior / timing analysis report       |
| `error_ledger.json`            | Accumulated repair rules for the LLM    |




## Repair Strategy

When dynamic testing reports errors, the CLI applies scoped fixes:

1. **Compile errors in** `other_codes` — regenerate only the affected bundled C/C++ sources.
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