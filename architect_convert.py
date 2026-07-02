#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Architect agent: convert AADL JSON model to ROS2 architecture JSON.
"""

import json
import argparse
import re

from aadl_parser import behavior_parser

ALLOWED_AADL_PROPERTIES = {
    'dispatch_protocol',
    'period',
    'compute_execution_time',
    'deadline',
    'Source_Language',
    'Source_Name',
    'Source_Text'
}

def _qos_from_port_kind(port_kind):
        k = (port_kind or "").lower()
        if "event" in k:
            return {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 10}
        if "data" in k:
            return {"reliability": "BEST_EFFORT", "durability": "VOLATILE", "depth": 1}
        return {"reliability": "BEST_EFFORT", "durability": "VOLATILE", "depth": 1}

def _normalize_property_name(prop_name):
    if prop_name is None:
        return ''
    text = str(prop_name).strip().lower()
    if '::' in text:
        text = text.split('::')[-1]
    return text


def _is_allowed_property(prop_name):
    return _normalize_property_name(prop_name) in ALLOWED_AADL_PROPERTIES


def _is_behavior_specification_annex(annex):
    name = (annex.get('name') or '').strip().lower().replace(' ', '_')
    return name == 'behavior_specification'


def _state_machine_from_behavior_annex_body(body):
    sm = behavior_parser.parse_behavior_specification(body or '')
    converted_vars = []
    for v in sm.get('variables', []):
        v_type = v.get('type', '').strip()
        ros_v_type = (
            get_ros_message_type(v_type)
            if '::' in v_type
            else v_type
        )
        converted_vars.append({
            'name': v.get('name', ''),
            'type': ros_v_type,
            'initial_value': v.get('initial_value', ''),
        })
    sm['variables'] = converted_vars
    return sm


def _normalize_aadl_initial_value(raw) -> str:
    if raw is None:
        return ''
    return str(raw).replace('"', '').replace('(', '').replace(')', '').strip()


def _merge_data_subcomponent_variables(thread: dict, state_machine: dict) -> dict:
    """Thread-impl data subcomponents (e.g. ``vrp: data int {Initial_Value => 1}``) act as BA variables."""
    sm = state_machine or {}
    existing = {v.get('name') for v in sm.get('variables', [])}
    for sub in thread.get('subcomponents', []):
        if sub.get('category', '').lower() != 'data':
            continue
        name = sub.get('name', '')
        if not name or name in existing:
            continue
        init_val = ''
        dtype = sub.get('implementation', 'int')
        for prop in sub.get('properties', []):
            if prop.get('name', '').lower() == 'initial_value':
                init_val = _normalize_aadl_initial_value(prop.get('value', ''))
                break
        ros_v_type = get_ros_message_type(dtype) if '::' in dtype else dtype
        sm.setdefault('variables', []).append({
            'name': name,
            'type': ros_v_type,
            'initial_value': init_val,
        })
        existing.add(name)
    return sm


def _aadl_time_value_to_ms(value):
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        value = str(value)
    s = value.strip().replace(' ', '').lower()
    if not s:
        return None
    m = re.search(r'(-?\d+(?:\.\d+)?)(us|ms|s)', s)
    if not m:
        try:
            return float(s)
        except Exception:
            return None
    num = float(m.group(1))
    unit = m.group(2)
    if unit == 'ms':
        return num
    if unit == 's':
        return num * 1000.0
    if unit == 'us':
        return num / 1000.0
    return None


def _finalize_node_qos_and_executor(ros_node, dict_topology, node_key):
    """QoS + strip port_kind; executor only when the node has thread ``components`` (not for device ``state_machine``)."""
    comps = ros_node.get('components') or []
    has_sm = bool(ros_node.get('state_machine'))
    if not comps and not has_sm:
        return

    

    needs_qos = any('port_kind' in p for p in ros_node.get('publishers', []) + ros_node.get('subscribers', []))
    if not needs_qos and comps:
        for c in comps:
            if any('port_kind' in x for x in c.get('callbacks', []) + c.get('outputs', [])):
                needs_qos = True
                break

    if needs_qos:
        for pub in ros_node.get('publishers', []):
            pub['qos'] = _qos_from_port_kind(pub.get('port_kind'))
        for sub in ros_node.get('subscribers', []):
            sub['qos'] = _qos_from_port_kind(sub.get('port_kind'))
        for comp in comps:
            for cb in comp.get('callbacks', []):
                cb['qos'] = _qos_from_port_kind(cb.get('port_kind'))
            for out in comp.get('outputs', []):
                out['qos'] = _qos_from_port_kind(out.get('port_kind'))
        for pub in ros_node.get('publishers', []):
            pub.pop('port_kind', None)
        for sub in ros_node.get('subscribers', []):
            sub.pop('port_kind', None)
        for comp in comps:
            for cb in comp.get('callbacks', []):
                cb.pop('port_kind', None)
            for out in comp.get('outputs', []):
                out.pop('port_kind', None)

    if ros_node.get('executor'):
        return

    if comps:
        thread_period_ms = {}
        topo = dict_topology.get(node_key) or {}
        for tname, tprops in (topo.get('threads') or {}).items():
            if not isinstance(tprops, dict):
                continue
            period_val = next(
                (v for k, v in tprops.items() if isinstance(k, str) and k.strip().lower() == 'period'),
                None,
            )
            period_ms = _aadl_time_value_to_ms(period_val)
            if period_ms is not None:
                thread_period_ms[tname] = period_ms

        ranked_threads = []
        for comp in comps:
            tname = comp.get('name')
            if not tname:
                continue
            period_ms = thread_period_ms.get(tname)
            rank_period = period_ms if period_ms is not None else float('inf')
            ranked_threads.append((rank_period, tname, period_ms))

        ranked_threads.sort(key=lambda x: (x[0], x[1]))
        distinct_periods = []
        for rank_period, _, _ in ranked_threads:
            if rank_period == float('inf'):
                continue
            if rank_period not in distinct_periods:
                distinct_periods.append(rank_period)
        reentrant_periods = set(distinct_periods[:1])

        callback_groups_list = []
        for rank_period, tname, period_ms in ranked_threads:
            is_reentrant = rank_period in reentrant_periods
            cb_type = 'ReentrantCallbackGroup' if is_reentrant else 'MutuallyExclusiveCallbackGroup'
            execution_strategy = 'Reentrant' if is_reentrant else 'MutuallyExclusive'
            cb_record = {
                'type': cb_type,
                'execution_strategy': execution_strategy,
                'component': tname,
            }
            if period_ms is not None:
                cb_record['period_ms'] = int(round(period_ms))
            callback_groups_list.append(cb_record)

        ros_node['executor'] = {
            'callback_groups': callback_groups_list,
            'type': 'MultiThreadedExecutor',
        }


def get_ros_message_type(aadl_type):
    type_mapping = {
        'Base_Types::Float_32': 'std_msgs::msg::Float32',
        'Base_Types::Integer': 'std_msgs::msg::Int32',
        'Base_Types::Boolean': 'std_msgs::msg::Bool',
    }
    if aadl_type in type_mapping:
        return type_mapping[aadl_type]
    # Parser JSON often uses package-local names, e.g. MinePump_BA::Int, ::int — not Base_Types::Integer.
    leaf = aadl_type.rsplit('::', 1)[-1].strip().lower() if aadl_type else ''
    if leaf in ('int', 'integer', 'int32', 'int64', 'natural', 'long_integer'):
        return 'std_msgs::msg::Int32'
    if leaf in ('bool', 'boolean'):
        return 'std_msgs::msg::Bool'
    if leaf in ('float', 'float_32', 'float32', 'real', 'double'):
        return 'std_msgs::msg::Float32'
    return 'std_msgs::msg::Float32'


def _attach_shared_c_simulation_hints(ros_package):
    """
    If multiple thread components under the same node share the same Source_Text content,
    the original AADL model shares global state across threads.
    Downstream codegen should use one process-wide state, not duplicated per-component locals.
    """
    text_to_comps = {}
    for node in ros_package.get('nodes', []):
        for comp in node.get('components') or []:
            cname = (comp.get('name') or '').strip()
            if not cname:
                continue
            seen_texts = set()
            for sub in comp.get('subprograms') or []:
                props = sub.get('properties') or {}
                src_text = (props.get('Source_Text') or props.get('source_text') or '').strip()
                if not src_text:
                    continue
                if src_text in seen_texts:
                    continue
                seen_texts.add(src_text)
                text_to_comps.setdefault(src_text, set()).add(cname)
    shared = [t for t, comps in text_to_comps.items() if len(comps) >= 2]
    if shared:
        ros_package['shared_code'] = {
            'source_text': shared,
            'hint': (
                'AADL subprograms share one code block (globals + mutex). '
                'ROS2 codegen must expose one shared code block for the whole process node, '
                'not independent code blocks per component.'
            ),
        }


def build_port_dtype_map(components):
    """
    Build a flat lookup: "comp_name_lower.port_name_lower" -> ROS message type string.
    components: list of AADL component dicts (each with 'name' and 'ports' fields)
    """
    port_dtype_map = {}
    for comp in components:
        comp_name = comp.get('name', '').lower()
        if not comp_name:
            continue
        for port in comp.get('ports', []):
            port_name = port.get('name', '').lower()
            if not port_name:
                continue
            key = f"{comp_name}.{port_name}"
            dt = port.get('data_type') or {}
            data_rep = next(
                ((p.get('value') or '').strip() for p in dt.get('properties') or []
                 if (p.get('name') or '').strip().lower() == 'data_representation'),
                None
            )
            aadl_type = f"::{data_rep}" if data_rep else f"{dt.get('package', '')}::{dt.get('name', '')}"
            port_dtype_map[key] = get_ros_message_type(aadl_type)
    return port_dtype_map


def build_port_kind_map(components):
    """
    Build a flat lookup: "comp_name_lower.port_name_lower" -> port_kind string.
    """
    port_kind_map = {}
    for comp in components:
        comp_name = comp.get('name', '').lower()
        if not comp_name:
            continue
        for port in comp.get('ports', []) or []:
            port_name = port.get('name', '').lower()
            if not port_name:
                continue
            port_kind_map[f"{comp_name}.{port_name}"] = port.get('port_kind')
    return port_kind_map


def add_unique(lst, entry, dedup_keys):
    """Append entry to lst only if no existing item matches all dedup_keys."""
    for item in lst:
        if all(item.get(k) == entry.get(k) for k in dedup_keys):
            return
    lst.append(entry)


def _sanitize_topic_suffix_for_cpp_member(topic: str) -> str:
    """Normalize connection/topic slug to a valid C++ identifier fragment."""
    s = str(topic).strip().lower()
    if s.startswith("/"):
        s = s[1:]
    s = re.sub(r"[^0-9a-zA-Z_]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        return "fanout"
    if s[0].isdigit():
        s = "_" + s
    return s


def assign_output_publisher_members(outputs):
    """In-place: same logical ``port``, multiple ROS outputs → first stays default ``pub_<port>_`` (no ``publisher_member``); extras set ``publisher_member`` = ``<port>_<topic_suffix>``.

    Duplicate ports sorted by ``topic`` (typically ``conn_name.lower()``) for deterministic "first".
    """
    from collections import defaultdict

    if not outputs:
        return
    grouped = defaultdict(list)
    for i, out in enumerate(outputs):
        port = str(out.get("port", "")).strip().lower()
        if not port:
            continue
        grouped[port].append(i)
    for port, indices in grouped.items():
        if len(indices) <= 1:
            outputs[indices[0]].pop("publisher_member", None)
            continue
        keyed = [(i, outputs[i]) for i in indices]
        keyed.sort(key=lambda x: x[1].get("topic") or "")
        occupied = set()
        for rank, (_i, out) in enumerate(keyed):
            if rank == 0:
                out.pop("publisher_member", None)
                occupied.add(port)
                continue
            suffix = _sanitize_topic_suffix_for_cpp_member(out.get("topic", ""))
            candidate = f"{port}_{suffix}"
            n = 1
            while candidate in occupied:
                n += 1
                candidate = f"{port}_{suffix}_{n}"
            out["publisher_member"] = candidate
            occupied.add(candidate)


def process_process_connections(process_comp, ros_node, thread_components, port_dtype_map, port_kind_map):
    """
    Iterate over connections inside a process and populate:
      - ros_node['subscribers'] / ros_node['publishers']
      - thread_components[name]['callbacks'] / thread_components[name]['outputs']

    port_dtype_map/port_kind_map keys use "comp_name_lower.port_name_lower".
    """

    for conn in process_comp.get('connections', []):
        conn_name = conn['name']
        conn_type = (conn.get('type') or '').lower()
        source_str = conn['source']
        dest_str = conn['destination']

        src_parts = source_str.split('.')
        dst_parts = dest_str.split('.')
        if len(src_parts) < 2 or len(dst_parts) < 2:
            continue

        src_comp = src_parts[0].lower()
        src_port = '.'.join(src_parts[1:]).lower()
        dst_comp = dst_parts[0].lower()
        dst_port = '.'.join(dst_parts[1:]).lower()
        topic = conn_name.lower()

        # --- process2thread port connection ---
        # source = process port  →  Subscriber on the ROS node
        # dest   = thread port   →  callback on the thread component
        if conn_type == 'process2thread port connection':
            ros_type = port_dtype_map.get(
                f"{src_comp}.{src_port}",
                port_dtype_map.get(f"{dst_comp}.{dst_port}", 'std_msgs::msg::String')
            )
            src_kind = port_kind_map.get(f"{src_comp}.{src_port}")
            dst_kind = port_kind_map.get(f"{dst_comp}.{dst_port}")

            add_unique(ros_node['subscribers'],
                       {'topic': topic, 'port': src_port, 'message_type': ros_type, 'port_kind': src_kind},
                       ('topic', 'port'))

            thread_comp = _ensure_thread(thread_components, dst_comp)
            add_unique(thread_comp['callbacks'],
                       {'topic': topic, 'port': dst_port, 'message_type': ros_type, 'port_kind': dst_kind},
                       ('topic', 'port'))

        # --- thread2process port connection ---
        # source = thread port  →  output on the thread component
        # dest   = process port →  Publisher on the ROS node
        elif conn_type == 'thread2process port connection':
            ros_type = port_dtype_map.get(
                f"{dst_comp}.{dst_port}",
                port_dtype_map.get(f"{src_comp}.{src_port}", 'std_msgs::msg::String')
            )
            src_kind = port_kind_map.get(f"{src_comp}.{src_port}")
            dst_kind = port_kind_map.get(f"{dst_comp}.{dst_port}")

            thread_comp = _ensure_thread(thread_components, src_comp)
            add_unique(thread_comp['outputs'],
                       {'topic': topic, 'port': src_port, 'message_type': ros_type, 'port_kind': src_kind},
                       ('topic', 'port'))

            add_unique(ros_node['publishers'],
                       {'topic': topic, 'port': dst_port, 'message_type': ros_type, 'port_kind': dst_kind},
                       ('topic', 'port'))

        # --- same-level thread port connection ---
        # source thread port → output on source thread
        # dest   thread port → callback on dest thread
        elif conn_type == 'same-level thread port connection':
            ros_type = port_dtype_map.get(
                f"{src_comp}.{src_port}",
                port_dtype_map.get(f"{dst_comp}.{dst_port}", 'std_msgs::msg::String')
            )
            src_kind = port_kind_map.get(f"{src_comp}.{src_port}")
            dst_kind = port_kind_map.get(f"{dst_comp}.{dst_port}")

            src_thread = _ensure_thread(thread_components, src_comp)
            add_unique(src_thread['outputs'],
                       {'topic': topic, 'port': src_port, 'message_type': ros_type, 'port_kind': src_kind},
                       ('topic', 'port'))

            dst_thread = _ensure_thread(thread_components, dst_comp)
            add_unique(dst_thread['callbacks'],
                       {'topic': topic, 'port': dst_port, 'message_type': ros_type, 'port_kind': dst_kind},
                       ('topic', 'port'))

        # --- data access connection ---
        # source = data component port → output on data component (treated as thread component)
        # dest   = thread port         → callback on thread component
        elif conn_type == 'data access connection':
            ros_type = port_dtype_map.get(
                f"{src_comp}.{src_port}",
                port_dtype_map.get(f"{dst_comp}.{dst_port}", 'std_msgs::msg::String')
            )
            src_kind = port_kind_map.get(f"{src_comp}.{src_port}")
            dst_kind = port_kind_map.get(f"{dst_comp}.{dst_port}")

            src_thread = _ensure_thread(thread_components, src_comp)
            add_unique(src_thread['outputs'],
                       {'topic': topic, 'port': src_port, 'message_type': ros_type, 'port_kind': src_kind},
                       ('topic', 'port'))

            dst_thread = _ensure_thread(thread_components, dst_comp)
            add_unique(dst_thread['callbacks'],
                       {'topic': topic, 'port': dst_port, 'message_type': ros_type, 'port_kind': dst_kind},
                       ('topic', 'port'))


def process_system_connections(system, node_map, port_dtype_map, port_kind_map):
    """
    Handle system-level connections: process2device, device2process, and process2process.
    node_map: dict node_name_lower -> ros_node dict
    """
    for conn in system.get('connections', []):
        conn_type = (conn.get('type') or '').lower()
        if conn_type not in (
            'process2device port connection',
            'device2process port connection',
            'process2process port connection',
        ):
            continue

        conn_name = conn['name']
        src_parts = conn['source'].split('.')
        dst_parts = conn['destination'].split('.')
        if len(src_parts) < 2 or len(dst_parts) < 2:
            continue

        src_comp = src_parts[0].lower()
        src_port = '.'.join(src_parts[1:]).lower()
        dst_comp = dst_parts[0].lower()
        dst_port = '.'.join(dst_parts[1:]).lower()
        topic = conn_name.lower()

        # --- process2process: update destination callbacks to use source topic ---
        if conn_type == 'process2process port connection':
            if dst_comp in node_map:
                # The destination's internal process2thread connection created a callback
                # with topic = /<dst_comp>/<dst_port>. Replace it with the shared topic.
                old_topic = '/' + dst_comp + '/' + dst_port
                for comp in node_map[dst_comp].get('components', []):
                    for cb in comp.get('callbacks', []):
                        if cb['topic'] == old_topic:
                            cb['topic'] = topic
            continue

        ros_type = port_dtype_map.get(
            f"{src_comp}.{src_port}",
            port_dtype_map.get(f"{dst_comp}.{dst_port}", 'std_msgs::msg::String')
        )
        src_kind = port_kind_map.get(f"{src_comp}.{src_port}")
        dst_kind = port_kind_map.get(f"{dst_comp}.{dst_port}")

        # process2device: process → Publisher, device → Subscriber
        # device2process: device  → Publisher, process → Subscriber
        if conn_type == 'process2device port connection':
            pub_comp, pub_port = src_comp, src_port
            sub_comp, sub_port = dst_comp, dst_port
        else:  # device2process
            pub_comp, pub_port = src_comp, src_port
            sub_comp, sub_port = dst_comp, dst_port

        if pub_comp in node_map:
            pub_kind = src_kind if pub_comp == src_comp else dst_kind
            add_unique(node_map[pub_comp].setdefault('publishers', []),
                       {'topic': topic, 'port': pub_port, 'message_type': ros_type, 'port_kind': pub_kind},
                       ('topic', 'port'))
        if sub_comp in node_map:
            sub_kind = dst_kind if sub_comp == dst_comp else src_kind
            add_unique(node_map[sub_comp].setdefault('subscribers', []),
                       {'topic': topic, 'port': sub_port, 'message_type': ros_type, 'port_kind': sub_kind},
                       ('topic', 'port'))


def _ensure_thread(thread_components, comp_name):
    """Return or create a thread component dict by name."""
    if comp_name not in thread_components:
        thread_components[comp_name] = {
            'name': comp_name,
            'callbacks': [],
            'outputs': [],
            'properties': []
        }
    return thread_components[comp_name]


def _build_test_node(process_node):
    """Build a stimulus/monitor test node that mirrors a process node's I/O.

    Publishers  = process subscribers  (feed inputs to the process)
    Subscribers = process publishers   (observe outputs from the process)
    Period      = 10 ms, state machine emits random values per publisher port.
    """
    proc_name = process_node['name']

    def _copy_port(entry):
        return {
            'topic': entry['topic'],
            'port': entry['port'],
            'message_type': entry['message_type'],
            'qos': dict(entry.get('qos') or {'reliability': 'BEST_EFFORT', 'durability': 'VOLATILE', 'depth': 1}),
        }

    publishers  = [_copy_port(e) for e in process_node.get('subscribers', [])]
    subscribers = [_copy_port(e) for e in process_node.get('publishers',  [])]

    def _rand_expr(msg_type):
        t = (msg_type or '').lower()
        if 'bool'  in t: return '(rand() % 2 == 0) ? true : false'
        if 'float' in t: return '(float)(rand() % 1000) / 100.0f'
        if 'int' in t: return 'rand() % 100'
        return 'rand() % 100'

    actions = '; '.join(f"{p['port']} := {_rand_expr(p['message_type'])}" for p in publishers) or 'skip'

    return {
        'name': f"{proc_name}_test_node",
        'publishers':  publishers,
        'subscribers': subscribers,
        'state_machine': {
            'variables': [],
            'states': [{'name': 's', 'properties': 'INITIAL COMPLETE FINAL STATE'}],
            'transitions': [{'source': 's', 'target': 's', 'condition': 'ON DISPATCH', 'actions': actions}],
        },
        'properties': {'period': '10ms'},
    }


def _deduplicate_component_names(ros_package: dict) -> None:
    """
    Rename components whose name is shared across multiple nodes in the same package.
    e.g. th_c in proc_capteur_droit AND proc_capteur_gauche → th_c_proc_capteur_droit / th_c_proc_capteur_gauche
    This ensures the code generator produces separate .cpp/.hpp files per node with the
    correct hardcoded topic names for each instance.
    """
    from collections import defaultdict
    comp_name_nodes: dict = defaultdict(list)
    for node in ros_package.get('nodes', []):
        for comp in node.get('components', []):
            comp_name_nodes[comp['name']].append(node['name'])

    for comp_name, node_names in comp_name_nodes.items():
        if len(node_names) <= 1:
            continue
        for node in ros_package.get('nodes', []):
            if node['name'] not in node_names:
                continue
            for comp in node.get('components', []):
                if comp['name'] == comp_name:
                    comp['name'] = f"{comp_name}_{node['name']}"


def convert_aadl_to_ros2(aadl_json_path):
    """
    Convert AADL JSON to ROS2 architecture JSON, and extract three dictionaries during the conversion:

    dict_topic  (dictionary 1 - topic)
    dict_topology  (dictionary 2 - topology)

    Returns: (ros2_architecture, dict_topic, dict_topology)
            """
    with open(aadl_json_path, 'r', encoding='utf-8') as f:
        aadl_model = json.load(f)

    ros2_architecture = {'ROSPackages': []}
    dict_topic         = {}   # topic -> list of topic and QoS records
    dict_topology      = {}   # topology -> list of topology and properties records

    def _topo_add(topic, record):
        """Add endpoint record to dict_topic (deduplication)"""
        if topic not in dict_topic:
            dict_topic[topic] = []
        for existing in dict_topic[topic]:
            if existing.get('component') == record.get('component') and \
               existing.get('type') == record.get('type') and \
               existing.get('port') == record.get('port'):
                return
        dict_topic[topic].append(record)

    for system in aadl_model:
        if system['category'] != 'system':
            continue

        package_name = system['name'].lower()
        ros_package = {'name': package_name, 'nodes': []}
        other_codes = system.get('other_codes')
        if other_codes:
            ros_package['other_codes'] = other_codes
        node_map = {}

        system_port_dtype_map = build_port_dtype_map(system.get('subcomponents', []))
        system_port_kind_map = build_port_kind_map(system.get('subcomponents', []))

        for subcomponent in system['subcomponents']:
            category = subcomponent['category']

            if category == 'process':
                node_name = subcomponent['name'].lower()
                ros_node = {
                    'name': node_name,
                    'components': [],
                    'subscribers': [],
                    'publishers': []
                }
                # Build dictionary 2: process node properties
                node_props = {}
                for prop in subcomponent.get('properties', []):
                    if not _is_allowed_property(prop.get('name')):
                        continue
                    node_props[prop['name']] = prop['value']
                dict_topology[node_name] = {
                    'properties': node_props,
                    'threads': {}
                }

                thread_components = {}
                for thread in subcomponent.get('subcomponents', []):
                    if thread['category'] != 'thread':
                        continue
                    comp_name = thread['name'].lower()
                    ros_component = {
                        'name': comp_name,
                        'callbacks': [],
                        'outputs': [],
                        'properties': {}
                    }

                    # Build dictionary 2: thread properties
                    thread_props = {}
                    for prop in thread.get('properties', []):
                        if not _is_allowed_property(prop.get('name')):
                            continue
                        ros_component['properties'][prop['name']] = prop['value']
                        thread_props[prop['name']] = prop['value']

                    dict_topology[node_name]['threads'][comp_name] = thread_props

                    # Behavior annex → state machine
                    for annex in thread.get('annexes', []):
                        if _is_behavior_specification_annex(annex):
                            ros_component['state_machine'] = _merge_data_subcomponent_variables(
                                thread,
                                _state_machine_from_behavior_annex_body(annex.get('body', '')),
                            )

                    # Each subprogram is treated as a separate item, containing the name and property set.
                    subprogram_items = []
                    for sub in thread.get('calls', []):
                        sub_name = sub.get('subprogram_name', '')
                        props_obj = {}
                        for p in sub.get('subprogram_properties', []):
                            pname = p.get('name', '')
                            pval = p.get('value', '')
                            # if not _is_allowed_property(pname):
                            #     continue
                            props_obj[pname] = pval
                            if pname == 'Source_Text':
                                st_pkg = (p.get('package') or '').strip()
                                if st_pkg:
                                    props_obj['Source_Text_File'] = st_pkg
                        subprogram_items.append(
                            {
                                'name': sub_name,
                                'properties': props_obj
                            }
                        )
                    if subprogram_items:
                        ros_component['subprograms'] = subprogram_items

                    thread_components[comp_name] = ros_component

                # Build port dtype map scoped to this process
                process_scope_comps = [subcomponent] + subcomponent.get('subcomponents', [])
                process_port_dtype_map = build_port_dtype_map(process_scope_comps)
                process_port_kind_map = build_port_kind_map(process_scope_comps)
                # Convert connections → subscribers / publishers / callbacks / outputs
                process_process_connections(
                    subcomponent,
                    ros_node,
                    thread_components,
                    process_port_dtype_map,
                    process_port_kind_map,
                )

                for _tc in thread_components.values():
                    assign_output_publisher_members(_tc.get("outputs") or [])

                ros_node['components'] = list(thread_components.values())

                ros_package['nodes'].append(ros_node)
                node_map[node_name] = ros_node

                # Build dictionary 1: extract topology endpoints from ros_node
                for pub in ros_node.get('publishers', []):
                    _topo_add(pub['topic'], {
                        'component': node_name, 'type': 'publisher',
                        'port': pub['port'], 'message_type': pub['message_type']
                    })
                for sub in ros_node.get('subscribers', []):
                    _topo_add(sub['topic'], {
                        'component': node_name, 'type': 'subscriber',
                        'port': sub['port'], 'message_type': sub['message_type']
                    })
                for comp in ros_node.get('components', []):
                    for cb in comp.get('callbacks', []):
                        _topo_add(cb['topic'], {
                            'component': comp['name'], 'type': 'callback',
                            'port': cb['port'], 'message_type': cb['message_type']
                        })
                    for out in comp.get('outputs', []):
                        _topo_add(out['topic'], {
                            'component': comp['name'], 'type': 'output',
                            'port': out['port'], 'message_type': out['message_type']
                        })

            elif category == 'device':
                node_name = subcomponent['name'].lower()
                ros_node = {
                    'name': node_name,
                    'subscribers': [],
                    'publishers': []
                }
                if any(_is_behavior_specification_annex(a) for a in subcomponent.get('annexes', [])):
                    node_props = {}
                    for prop in subcomponent.get('properties', []):
                        if not _is_allowed_property(prop.get('name')):
                            continue
                        node_props[prop['name']] = prop['value']
                    dict_topology[node_name] = {'properties': node_props, 'threads': {}}
                    for annex in subcomponent.get('annexes', []):
                        if _is_behavior_specification_annex(annex):
                            ros_node['state_machine'] = _merge_data_subcomponent_variables(
                                subcomponent,
                                _state_machine_from_behavior_annex_body(annex.get('body', '')),
                            )
                    subprogram_items = []
                    for sub in subcomponent.get('calls', []):
                        sub_name = sub.get('subprogram_name', '')
                        props_obj = {}
                        for p in sub.get('subprogram_properties', []):
                            pname = p.get('name', '')
                            props_obj[pname] = p.get('value', '')
                            if pname == 'Source_Text':
                                st_pkg = (p.get('package') or '').strip()
                                if st_pkg:
                                    props_obj['Source_Text_File'] = st_pkg
                        subprogram_items.append({'name': sub_name, 'properties': props_obj})
                    if subprogram_items:
                        ros_node['subprograms'] = subprogram_items

                ros_package['nodes'].append(ros_node)
                node_map[node_name] = ros_node

        # Handle system-level process↔device connections
        process_system_connections(system, node_map, system_port_dtype_map, system_port_kind_map)

        # Rename duplicate component names across nodes in this package.
        # When multiple nodes share the same implementation (e.g. th_c used by both
        # proc_capteur_droit and proc_capteur_gauche), they map to the same component
        # name. Renaming to <comp>_<node> gives each node its own generated file with
        # the correct hardcoded topic names.
        _deduplicate_component_names(ros_package)

        for ros_node in ros_package.get('nodes', []):
            _finalize_node_qos_and_executor(ros_node, dict_topology, ros_node['name'])

        # Inject a test node per process when the system has no devices and no subprograms.
        _has_device = any(s['category'] == 'device' for s in system.get('subcomponents', []))
        _has_subprograms = any(
            thread.get('calls')
            for sub in system.get('subcomponents', []) if sub['category'] == 'process'
            for thread in sub.get('subcomponents', []) if thread['category'] == 'thread'
        )
        if not _has_device and not _has_subprograms:
            for ros_node in [n for n in ros_package['nodes'] if n.get('components')]:
                ros_package['nodes'].append(_build_test_node(ros_node))

        # Drop nodes that have no pub/sub/components/state_machine (after system connections).
        def _node_has_ros_role(n):
            return bool(
                n.get('publishers') or n.get('subscribers') or n.get('components') or n.get('state_machine')
            )

        kept_names = {n['name'] for n in ros_package['nodes'] if _node_has_ros_role(n)}
        ros_package['nodes'] = [n for n in ros_package['nodes'] if n['name'] in kept_names]
        for topo_name in list(dict_topology.keys()):
            if topo_name not in kept_names:
                del dict_topology[topo_name]

        # Deterministic QoS for device nodes (no components -> no thread Period available).
        # Use port_kind when available; otherwise fallback to sensor-like QoS.
        for ros_node in ros_package.get("nodes", []):
            if "components" in ros_node or ros_node.get("state_machine"):
                continue
            for pub in ros_node.get("publishers", []):
                pub["qos"] = _qos_from_port_kind(pub.get("port_kind"))
                pub.pop("port_kind", None)
            for sub in ros_node.get("subscribers", []):
                sub["qos"] = _qos_from_port_kind(sub.get("port_kind"))
                sub.pop("port_kind", None)

        # Build dictionary 1: extract topology endpoints from device nodes
        for ros_node in ros_package['nodes']:
            if 'components' in ros_node:
                continue  # Process nodes have been handled above
            for pub in ros_node.get('publishers', []):
                _topo_add(pub['topic'], {
                    'component': ros_node['name'], 'type': 'publisher',
                    'port': pub['port'], 'message_type': pub['message_type']
                })
            for sub in ros_node.get('subscribers', []):
                _topo_add(sub['topic'], {
                    'component': ros_node['name'], 'type': 'subscriber',
                    'port': sub['port'], 'message_type': sub['message_type']
                })

        _attach_shared_c_simulation_hints(ros_package)
        ros2_architecture['ROSPackages'].append(ros_package)

    return ros2_architecture

def main():
    parser = argparse.ArgumentParser(
        description="Architect agent: transform AADL JSON model to ROS2 architecture JSON"
    )
    parser = argparse.ArgumentParser(description='transform AADL model to ROS architecture JSON')
    parser.add_argument('-a', '--aadl', type=str, required=True, help='AADL file path (supports JSON and XML formats)')
    parser.add_argument('-o', '--output', type=str, default='ros_architecture.json', help='output JSON file path')
    args = parser.parse_args()

    ros2_architecture = convert_aadl_to_ros2(args.aadl)

    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(ros2_architecture, f, ensure_ascii=False, indent=2)
    print(f"ROS2 architecture saved to: {args.output}")

if __name__ == "__main__":
    main()