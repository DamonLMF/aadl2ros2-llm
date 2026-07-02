"""Jinja2 template rendering and deterministic context for ROS2 C++ codegen (no LLM)."""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional, Tuple

from jinja2 import Environment, FileSystemLoader


def cpp_string_content_escape(s: str) -> str:
    """Escape for C++ string literal body (between double quotes)."""
    return str(s).replace("\\", "\\\\").replace('"', '\\"')


def fsm_state_names(component: Dict[str, Any]) -> List[str]:
    sm = component.get("state_machine") or {}
    if not isinstance(sm, dict):
        return []
    names: List[str] = []
    for s in sm.get("states") or []:
        if isinstance(s, dict):
            n = str(s.get("name", "")).strip()
        else:
            n = str(s).strip()
        if n:
            names.append(n)
    return names


def fsm_template_context(component: Dict[str, Any]) -> Dict[str, Any]:
    """
    use_fsm: emit std::string fsm_state_ support in generated component class/shell.
    fsm_wait_cpp: escaped initial FSM state name (states[0]).
    """
    names = fsm_state_names(component)
    callbacks = component.get("callbacks") or []
    use_fsm = bool(names) and bool(callbacks)
    all_states_cpp = [cpp_string_content_escape(n) for n in names]
    if not use_fsm:
        return {
            "use_fsm": False,
            "fsm_wait_cpp": "",
            "fsm_states_cpp": all_states_cpp,
        }
    wait = names[0]
    return {
        "use_fsm": True,
        "fsm_wait_cpp": cpp_string_content_escape(wait),
        "fsm_states_cpp": all_states_cpp,
    }


def parse_ms(prop: Any, default: float) -> float:
    if prop is None:
        return default
    s = str(prop).strip().lower()
    m = re.match(r"^([\d.]+)\s*(ms|us|s)?$", s)
    if not m:
        return default
    val = float(m.group(1))
    unit = m.group(2) or "ms"
    if unit == "s":
        return val * 1000.0
    if unit == "us":
        return val / 1000.0
    return val


def parse_compute_max_ms(cet: Any, default_max_ms: float) -> float:
    s = str(cet or "").strip().lower()
    if not s:
        return default_max_ms
    parts = re.split(r"\.\.|…|–|-", s)
    nums: List[float] = []
    for part in parts:
        part = part.strip()
        m = re.search(r"([\d.]+)\s*(ms|us|s)?", part)
        if not m:
            continue
        v = float(m.group(1))
        u = (m.group(2) or "ms").lower()
        if u == "s":
            nums.append(v * 1000.0)
        elif u == "us":
            nums.append(v / 1000.0)
        else:
            nums.append(v)
    if not nums:
        return default_max_ms
    return max(nums)


def parse_period_deadline_compute(
    props: Dict[str, Any],
) -> Tuple[int, Optional[float], Optional[float]]:
    """Parse period/deadline/compute from component properties.

    Returns (period_ms, deadline_ms, compute_max_ms) where deadline_ms and
    compute_max_ms are None when the property is absent **or** when its value
    is sub-millisecond (e.g. specified in microseconds) — such thresholds are
    impractical for ROS 2 middleware and would fire on every tick.
    """
    if not isinstance(props, dict):
        props = {}
    period = int(parse_ms(props.get("Period") or props.get("period"), 60.0))

    dl_raw = props.get("Deadline") or props.get("deadline")
    if dl_raw is not None:
        dl_val = parse_ms(dl_raw, -1.0)
        deadline: Optional[float] = dl_val if dl_val >= 1.0 else None
    else:
        deadline = None

    cet = props.get("Compute_Execution_Time") or props.get("compute_execution_time") or ""
    if cet:
        cm_val = parse_compute_max_ms(cet, default_max_ms=-1.0)
        compute_max: Optional[float] = cm_val if cm_val >= 1.0 else None
    else:
        compute_max = None

    return period, deadline, compute_max


def qos_to_ros2_pub_cli_flags(qos: Optional[Dict[str, Any]]) -> str:
    """CLI flags for ``ros2 topic pub`` aligned with ``qos_cpp_lines`` / architecture JSON."""
    if not qos:
        return (
            "--qos-reliability reliable --qos-durability transient_local "
            "--qos-history keep_last --qos-depth 10"
        )
    rel = str(qos.get("reliability", "")).upper()
    dur = str(qos.get("durability", "")).upper()
    depth = qos.get("depth", 10)
    rel_flag = "best_effort" if "BEST_EFFORT" in rel else "reliable"
    dur_flag = "transient_local" if "TRANSIENT" in dur else "volatile"
    try:
        d = int(depth)
    except (TypeError, ValueError):
        d = 10
    return (
        f"--qos-reliability {rel_flag} --qos-durability {dur_flag} "
        f"--qos-history keep_last --qos-depth {d}"
    )


def qos_cpp_lines(qos: Optional[Dict[str, Any]]) -> List[str]:
    lines = ["    auto qos = rclcpp::QoS(10);"]
    if not qos:
        lines.append("    qos.reliable();")
        lines.append("    qos.transient_local();")
        return lines
    rel = str(qos.get("reliability", "")).upper()
    dur = str(qos.get("durability", "")).upper()
    depth = qos.get("depth", 10)
    if "BEST_EFFORT" in rel:
        lines.append("    qos.best_effort();")
    else:
        lines.append("    qos.reliable();")
    if "TRANSIENT" in dur:
        lines.append("    qos.transient_local();")
    else:
        lines.append("    qos.durability_volatile();")
    try:
        d = int(depth)
        lines.append(f"    qos.keep_last({d});")
    except (TypeError, ValueError):
        pass
    return lines


_AADL_SCALAR_CPP: Dict[str, str] = {
    "boolean": "bool",
    "bool": "bool",
    "integer": "int32_t",
    "int": "int32_t",
    "int32": "int32_t",
    "natural": "uint32_t",
    "unsigned": "uint32_t",
    "positive": "uint32_t",
    "float": "float",
    "real": "float",
    "long_float": "double",
    "double": "double",
}


def _aadl_scalar_key(type_str: str) -> str:
    t = str(type_str or "").strip()
    if "::" in t:
        t = t.rsplit("::", 1)[-1]
    return t.lower().replace(" ", "_")


def _float_brace_init(init_raw: str) -> str:
    """Valid C++ float literal for brace-init (e.g. 1 -> 1.0f, not 1f)."""
    if not init_raw:
        return "0.0f"
    try:
        x = float(init_raw)
    except (TypeError, ValueError):
        return "0.0f"
    if x == int(x) and abs(x) < 1e16:
        return f"{int(x)}.0f"
    return f"{x}f"


def normalize_msg_type(message_type: str) -> str:
    t = str(message_type or "").strip()
    t = t.replace("::", "/")
    if ".msg." in t:
        pkg, msg = t.split(".msg.", 1)
        pkg = pkg.strip()
        msg = msg.strip()
        return f"{pkg}::msg::{msg}"
    if "/msg/" in t:
        pkg, msg = t.split("/msg/", 1)
        pkg = pkg.strip()
        msg = msg.strip()
        return f"{pkg}::msg::{msg}"
    if "::msg::" in t:
        return t
    return "std_msgs::msg::Float32"


def msg_include_from_type(normalized_type: str) -> str:
    if "::msg::" not in normalized_type:
        return "std_msgs/msg/float32.hpp"
    pkg, msg = normalized_type.split("::msg::", 1)
    return f"{pkg}/msg/{msg.lower()}.hpp"


def safe_identifier(text: str) -> str:
    s = str(text or "").strip()
    s = re.sub(r"[^0-9a-zA-Z_]", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        return "unnamed"
    if s[0].isdigit():
        s = f"v_{s}"
    return s


def state_var_decl_line(var_name: str, var_type: str, initial_value: Any) -> str:
    name = safe_identifier(var_name)
    init_raw = str(initial_value if initial_value is not None else "").strip()
    scalar_key = _aadl_scalar_key(var_type)

    if scalar_key in _AADL_SCALAR_CPP:
        cpp_type = _AADL_SCALAR_CPP[scalar_key]
        if cpp_type == "bool":
            init_tok = "true" if init_raw.lower() in ("1", "true", "yes") else "false"
        elif cpp_type == "float":
            init_tok = _float_brace_init(init_raw)
        elif cpp_type == "double":
            init_tok = "0.0" if not init_raw else repr(float(init_raw))
        else:
            init_tok = init_raw if init_raw else "0"
        return f"    {cpp_type} {name}_{{{init_tok}}};"

    ntype = normalize_msg_type(var_type)
    if ntype.endswith("Int32"):
        init_tok = init_raw if init_raw else "0"
        return f"    int32_t {name}_{{{init_tok}}};"
    if ntype.endswith("UInt32"):
        init_tok = init_raw if init_raw else "0"
        return f"    uint32_t {name}_{{{init_tok}}};"
    if ntype.endswith("Bool"):
        init_tok = "true" if init_raw.lower() in ("1", "true", "yes") else "false"
        return f"    bool {name}_{{{init_tok}}};"
    # Float32 ROS types and unrecognized names: keep legacy float member type.
    return f"    float {name}_{{{_float_brace_init(init_raw)}}};"


def component_callback_group_map(executor: Dict[str, Any]) -> Dict[str, str]:
    m: Dict[str, str] = {}
    for g in executor.get("callback_groups") or []:
        comp = g.get("component")
        if not comp:
            continue
        t = str(g.get("type", ""))
        m[str(comp)] = "Reentrant" if "Reentrant" in t else "MutuallyExclusive"
    return m


def make_template_env(template_dir: str) -> Environment:
    return Environment(
        loader=FileSystemLoader(template_dir),
        trim_blocks=True,
        lstrip_blocks=True,
        keep_trailing_newline=True,
    )



def output_cpp_member(out: Dict[str, Any]) -> str:
    """C++ publisher field base name: optional JSON ``publisher_member`` else logical ``port``."""
    return str(out.get("publisher_member") or out.get("port") or "").replace('"', "").strip()

def render_jinja(template_dir: str, template_relative: str, **context: Any) -> str:
    env = make_template_env(template_dir)
    return env.get_template(template_relative).render(**context)


def render_shared_sim_state_hpp(
    template_dir: str, package_name: str, shared_vars: List[Dict[str, str]]
) -> str:
    """Render the per-package SharedSimState header."""
    env = make_template_env(template_dir)
    return env.get_template("shared_sim_state.hpp.j2").render(
        package_name=package_name.lower(),
        shared_vars=shared_vars,
    )


def render_llm_component_cpp(
    template_dir: str,
    component: Dict[str, Any],
    package_name: str,
    node_name: str,
    control_loop_llm_body: str,
    shared_vars: Optional[List[Dict[str, str]]] = None,
    extra_includes: Optional[List[str]] = None,
) -> str:
    class_name = component.get("name", "").replace('"', "")
    callbacks = component.get("callbacks", []) or []
    outputs = component.get("outputs", []) or []
    props = component.get("properties", {})
    if not isinstance(props, dict):
        props = {}
    period_ms, deadline_ms, compute_max_ms = parse_period_deadline_compute(props)

    include_set = {"#include <rclcpp/rclcpp.hpp>", "#include <time.h>", "#include <cmath>"}
    callbacks_render: List[Dict[str, Any]] = []
    for cb in callbacks:
        mt = normalize_msg_type(cb.get("message_type", ""))
        include_set.add(f"#include <{msg_include_from_type(mt)}>")
        cd = dict(cb)
        cd["qos_lines"] = [ln.strip() for ln in qos_cpp_lines(cb.get("qos"))]
        callbacks_render.append(cd)
    outputs_render: List[Dict[str, Any]] = []
    for out in outputs:
        mt = normalize_msg_type(out.get("message_type", ""))
        include_set.add(f"#include <{msg_include_from_type(mt)}>")
        od = dict(out)
        od["member"] = output_cpp_member(out)
        od["qos_lines"] = [ln.strip() for ln in qos_cpp_lines(out.get("qos"))]
        outputs_render.append(od)
    includes_sorted = sorted(include_set)
    other_code_include_lines = sorted(extra_includes or [])

    fsm_ctx = fsm_template_context(component)

    env = make_template_env(template_dir)
    return env.get_template("llm_component.cpp.j2").render(
        includes=includes_sorted,
        other_code_include_lines=other_code_include_lines,
        package_name=package_name.lower(),
        class_name=class_name,
        node_name=(node_name or "").strip(),
        callbacks=callbacks_render,
        outputs=outputs_render,
        period_ms=period_ms,
        normalize_msg_type=normalize_msg_type,
        control_loop_llm_body=control_loop_llm_body,
        deadline_ms=deadline_ms,    # None → no wall-clock check generated
        compute_max_ms=compute_max_ms,  # None → no thread-CPU check generated
        use_fsm=fsm_ctx["use_fsm"],
        fsm_wait_cpp=fsm_ctx["fsm_wait_cpp"],
        has_shared_state=bool(shared_vars),
    )


def is_device_style_node(node_info: Dict[str, Any]) -> bool:
    """Node has no ``components`` but has architecture-level publishers and/or subscribers."""
    if node_info.get("components"):
        return False
    pubs = node_info.get("publishers") or []
    subs = node_info.get("subscribers") or []
    return bool(pubs or subs)


def _device_endpoint_specs(endpoints: List[Dict[str, Any]], _prefix: str) -> List[Dict[str, Any]]:
    specs: List[Dict[str, Any]] = []
    for i, e in enumerate(endpoints or []):
        mt = normalize_msg_type(e.get("message_type", ""))
        topic = str(e.get("topic", "")).strip()
        qos_lines = [ln.strip() for ln in qos_cpp_lines(e.get("qos"))]
        if "Bool" in mt:
            fam, cpp_type, inc = "bool", "std_msgs::msg::Bool", "std_msgs/msg/bool.hpp"
        elif "Int32" in mt:
            fam, cpp_type, inc = "int32", "std_msgs::msg::Int32", "std_msgs/msg/int32.hpp"
        elif "String" in mt:
            fam, cpp_type, inc = "string", "std_msgs::msg::String", "std_msgs/msg/string.hpp"
        elif "Float64" in mt:
            fam, cpp_type, inc = "float64", "std_msgs::msg::Float64", "std_msgs/msg/float64.hpp"
        else:
            fam, cpp_type, inc = "float32", "std_msgs::msg::Float32", "std_msgs/msg/float32.hpp"
        specs.append(
            {
                "index": i,
                "topic": topic,
                "port": safe_identifier(e.get("port", f"p{i}")),
                "qos_lines": qos_lines,
                "fam": fam,
                "cpp_type": cpp_type,
                "include": inc,
            }
        )
    return specs


def render_device_node_cpp(template_dir: str, node_info: Dict[str, Any], package_name: str) -> str:
    """Deterministic device/stub node: timer publishes defaults; subscriptions receive-only."""
    node_name = (node_info.get("name") or "").strip()
    pub_specs = _device_endpoint_specs(node_info.get("publishers") or [], "pub")
    sub_specs = _device_endpoint_specs(node_info.get("subscribers") or [], "sub")

    include_set = {
        "#include <rclcpp/rclcpp.hpp>",
        "#include <chrono>",
        "#include <memory>",
        "#include <functional>",
    }
    for s in pub_specs + sub_specs:
        include_set.add(f"#include <{s['include']}>")

    env = make_template_env(template_dir)
    return env.get_template("cpp_device_node.cpp.j2").render(
        package_name=package_name.lower(),
        node_name=node_name,
        class_name=f"{safe_identifier(node_name)}_device",
        publishers=pub_specs,
        subscribers=sub_specs,
        has_publishers=bool(pub_specs),
        has_subscribers=bool(sub_specs),
        include_headers=sorted(include_set),
    )


def render_node_main_cpp(
    template_dir: str,
    node_info: Dict[str, Any],
    package_name: str,
    shared_vars: Optional[List[Dict[str, str]]] = None,
) -> str:
    node_name = (node_info.get("name") or "").strip()
    node_class_name = f"{safe_identifier(node_name)}_node"
    if node_class_name == "main_node":
        node_class_name = "main_process_node"
    executor = node_info.get("executor") or {}
    gm = component_callback_group_map(executor)
    rows: List[Dict[str, str]] = []
    for c in node_info.get("components") or []:
        cname = (c.get("name") or "").strip()
        if not cname:
            continue
        gt = gm.get(cname, "MutuallyExclusive")
        rows.append(
            {
                "name": cname,
                "group_enum": "Reentrant" if gt == "Reentrant" else "MutuallyExclusive",
            }
        )
    env = make_template_env(template_dir)
    return env.get_template("cpp_node_main.cpp.j2").render(
        package_name=package_name.lower(),
        node_name=node_name,
        node_class_name=node_class_name,
        components=rows,
        has_shared_state=bool(shared_vars),
    )


def render_component_header_hpp(
    template_dir: str,
    component: Dict[str, Any],
    node_name: str = "",
    package_name: str = "",
    shared_vars: Optional[List[Dict[str, str]]] = None,
    extra_includes: Optional[List[str]] = None,
) -> str:
    class_name = component.get("name", "").replace('"', '')
    node_base = (node_name or "").strip()
    ros_logger_prefix = f"{node_base}.{class_name}" if node_base else f"<ROS_NODE_NAME>.{class_name}"
    callbacks = component.get("callbacks", []) or []
    outputs = component.get("outputs", []) or []
    state_machine = component.get("state_machine", {}) or {}
    variables = state_machine.get("variables", []) if isinstance(state_machine, dict) else []

    include_set = set()
    callback_specs: List[Dict[str, str]] = []
    output_specs: List[Dict[str, str]] = []

    for cb in callbacks:
        port = cb.get("port", "").replace('"', '')
        msg_t = normalize_msg_type(cb.get("message_type", "std_msgs::msg::Float32"))
        include_set.add(msg_include_from_type(msg_t))
        callback_specs.append({"port": port, "msg_type": msg_t})

    for out in outputs:
        port = out.get("port", "").replace('"', '')
        msg_t = normalize_msg_type(out.get("message_type", "std_msgs::msg::Float32"))
        include_set.add(msg_include_from_type(msg_t))
        output_specs.append(
            {"port": port, "msg_type": msg_t, "member": output_cpp_member(out)}
        )

    for var in variables:
        msg_t = var.get("type", "").replace('"', '')
        include_set.add(msg_include_from_type(normalize_msg_type(msg_t)))

    has_int32 = any("Int32" in spec["msg_type"] for spec in callback_specs + output_specs) or any(
        "Int32" in normalize_msg_type(v.get("type", "")) for v in variables
    ) or any(
        _aadl_scalar_key(v.get("type", "")) in _AADL_SCALAR_CPP
        and _AADL_SCALAR_CPP[_aadl_scalar_key(v.get("type", ""))] in ("int32_t", "uint32_t")
        for v in variables
    )
    state_var_decls = [
        state_var_decl_line(
            var.get("name", "").replace('"', ''),
            var.get("type", "").replace('"', ''),
            var.get("initial_value", "").replace('"', ''),
        )
        for var in variables
    ]
    fsm_ctx = fsm_template_context(component)

    env = make_template_env(template_dir)
    template = env.get_template("component_header.hpp.j2")
    return template.render(
        class_name=class_name,
        ros_logger_prefix=ros_logger_prefix,
        include_headers=sorted(include_set),
        has_int32=has_int32,
        callbacks=callback_specs,
        outputs=output_specs,
        state_var_decls=state_var_decls,
        use_fsm=fsm_ctx["use_fsm"],
        fsm_states_cpp=fsm_ctx["fsm_states_cpp"],
        has_shared_state=bool(shared_vars),
        package_name=(package_name or "").lower(),
        other_code_include_lines=sorted(extra_includes or []),
    )
