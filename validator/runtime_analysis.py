import json
import os
import re
import argparse
from datetime import datetime

from validator.cba_metrics import (
    cba_from_comparison_result,
    system_cba_from_comparison_results,
)

def _summarize_fsm_errors(error_list: list) -> list:
    """Deduplicate FSM error entries by (from, to, reason), keeping only count."""
    counts: dict = {}
    for e in error_list:
        key = (e.get("from", ""), e.get("to", ""), e.get("reason", ""))
        counts[key] = counts.get(key, 0) + 1
    return [
        {"from": f, "to": t, "reason": r, "count": c}
        for (f, t, r), c in counts.items()
    ]


def _fsm_mismatch_guidance(result: dict) -> tuple[str, str]:
    """Return (root_cause, enforced_rule) for LLM repair prompts."""
    expected_states = sorted(result.get("expected_fsm_states") or [])
    endpoint_summary = _summarize_fsm_errors(result.get("fsm_bad_endpoints") or [])
    edge_summary = _summarize_fsm_errors(result.get("fsm_bad_edges") or [])
    missing_edges = result.get("fsm_missing_edges") or []
    root_cause = (
        "FSM mismatch details:\n"
        f"1) Endpoint validity (fsm_bad_endpoints): logged from/to must be in architecture states "
        f"(or allowed synthetic bootstrap states). Expected states={expected_states}; violations={endpoint_summary}.\n"
        f"2) Edge validity (fsm_bad_edges): when architecture declares transitions, logged edges must be in allowed edge set; "
        f"violations={edge_summary}.\n"
        f"3) Missing edges (fsm_missing_edges): architecture-declared transitions not observed/implemented in logs={missing_edges}."
    )
    enforced_rule = (
        "FSM: only declared states/edges; no extra synthetic states for single-state models. "
        "Do not gate the whole control_loop on all caches unless the architecture condition requires it; "
        "null-check only before `->`. Smallest fix: remove an outer all-inputs guard if transitions are missing from logs."
    )
    return root_cause, enforced_rule


# Log tokens that are not required to appear in architecture state_machine.states
_FSM_SYNTHETIC_STATES = frozenset({
    "__begin__",
    "bootstrap",
    "__start__",
})

_STATE_TRANSITION_LOG_RE = re.compile(
    r"State\s+transition:\s*(\S+)\s*->\s*(\S+)",
    re.IGNORECASE,
)


class RuntimeAnalysis:
    REPORT_EVENT_TAIL_LIMIT = 20

    def __init__(self, log_file_path: str, ros_architecture_file: str):
        self.log_file_path = log_file_path
        self.ros_architecture_file = ros_architecture_file
        self.components_behavior = {}
        self.ros_components = []
        self.ros_nodes = []  # {name, package} per logical node in ROSPackages
        self.comparison_results = []
        self.system_cba = None

    def _ensure_component_record(self, component_name_lower):
        if component_name_lower not in self.components_behavior:
            self.components_behavior[component_name_lower] = {
                'received': [],
                'published': [],
                'state_transitions': [],
                'events': []
            }

    @staticmethod
    def _normalize_port_key(name: str) -> str:
        """Canonical port name for comparison: lowercase; spaces/hyphens -> _; trailing _value dropped (pwm1_value -> pwm1)."""
        if not name or not isinstance(name, str):
            return ""
        s = name.strip().lower()
        s = re.sub(r"[\s\-]+", "_", s)
        s = re.sub(r"_+", "_", s).strip("_")
        if s.endswith("_value"):
            s = s[: -len("_value")]
        s = re.sub(r"_+", "_", s).strip("_")
        return s

    def _parse_port_pairs(self, ports_data):
        """Parse 'port:value' or 'port=value' style payload."""
        port_pairs = []
        format1_pairs = re.findall(r'([^:,=]+):\s*([^,]+)', ports_data)
        format2_pairs = re.findall(r'([^:,=]+)=\s*([^,]+)', ports_data)
        if format1_pairs:
            port_pairs = format1_pairs
        elif format2_pairs:
            port_pairs = format2_pairs
        elif ':' in ports_data:
            port, value = ports_data.split(':', 1)
            port_pairs = [(port, value)]
        elif '=' in ports_data:
            port, value = ports_data.split('=', 1)
            port_pairs = [(port, value)]
        elif ports_data:
            port_pairs = [(ports_data, None)]
        return port_pairs

    def _append_port_event(self, component_name_lower, event, event_type, timestamp, port, value):
        port = port.strip() if port else ''
        value = value.strip() if value else None
        target = 'published' if event_type == 'publish' else 'received'
        self.components_behavior[component_name_lower][target].append({
            'timestamp': timestamp,
            'port': port,
            'value': value
        })
        port_event = event.copy()
        port_event['type'] = event_type
        port_event['port'] = port
        port_event['value'] = value
        self.components_behavior[component_name_lower]['events'].append(port_event)

    @staticmethod
    def _build_fsm_allowed_edges(
        state_names: list,
        transition_pairs: list,
    ) -> set:
        """Directed edges allowed in logs: architecture transitions + boot -> first state."""
        edges: set = set()
        for a, b in transition_pairs:
            if a and b:
                edges.add((a, b))
        if state_names:
            first = state_names[0]
            for syn in ("__begin__", "bootstrap", "__start__"):
                edges.add((syn, first))
        return edges

    def _validate_logged_fsm_transitions(
        self,
        transitions: list,
        arch_state_names: list,
        arch_transition_pairs: list,
    ) -> tuple:
        """
        If architecture lists no FSM states, skip (no failure).
        Otherwise require each endpoint to be arch state or synthetic; if arch declares
        transitions, each log edge must match allowed edge set.
        Same-state transitions (from == to, self-loops) are not validated.
        """
        if not arch_state_names:
            return [], [], False
        allowed_states = {str(s).strip().lower() for s in arch_state_names if str(s).strip()}
        first_state = str(arch_state_names[0]).strip().lower() if arch_state_names else ""
        synth = _FSM_SYNTHETIC_STATES
        check_edges = bool(arch_transition_pairs)
        allowed_edges = (
            self._build_fsm_allowed_edges(arch_state_names, arch_transition_pairs)
            if check_edges
            else set()
        )
        bad_endpoints: list = []
        bad_edges: list = []
        for tr in transitions:
            raw_f = str(tr.get("from", "")).strip()
            raw_t = str(tr.get("to", "")).strip()
            fl, tl = raw_f.lower(), raw_t.lower()
            ts = tr.get("timestamp")
            # Bootstrap transition initial -> first architecture state is expected and not validated.
            if fl == "initial" and first_state and tl == first_state:
                continue
            # No-op self-loop: skip endpoint/edge checks (architecture rarely lists explicit self-edges).
            if fl == tl:
                continue
            if fl not in allowed_states and fl not in synth:
                bad_endpoints.append(
                    {
                        "timestamp": ts,
                        "from": raw_f,
                        "to": raw_t,
                        "reason": f"from-state {raw_f!r} not in architecture states",
                    }
                )
            if tl not in allowed_states and tl not in synth:
                bad_endpoints.append(
                    {
                        "timestamp": ts,
                        "from": raw_f,
                        "to": raw_t,
                        "reason": f"to-state {raw_t!r} not in architecture states",
                    }
                )
            if not check_edges:
                continue
            if fl in synth and tl in synth:
                if (fl, tl) not in allowed_edges:
                    bad_edges.append(
                        {
                            "timestamp": ts,
                            "from": raw_f,
                            "to": raw_t,
                            "reason": "synthetic->synthetic edge not allowed",
                        }
                    )
            elif fl in synth or tl in synth:
                if (fl, tl) not in allowed_edges:
                    bad_edges.append(
                        {
                            "timestamp": ts,
                            "from": raw_f,
                            "to": raw_t,
                            "reason": "edge not allowed for synthetic endpoint",
                        }
                    )
            else:
                if (fl, tl) not in allowed_edges:
                    bad_edges.append(
                        {
                            "timestamp": ts,
                            "from": raw_f,
                            "to": raw_t,
                            "reason": "edge not declared in architecture transitions (± reverse)",
                        }
                    )
        return bad_endpoints, bad_edges, True

    @staticmethod
    def _missing_logged_fsm_transitions(transitions: list, arch_transition_pairs: list) -> list:
        """Architecture transitions that never appeared in the runtime transition log.

        Self-loop edges are excluded end-to-end: log lines with from == to do not populate
        observed, and architecture pairs with source == target are not reported as missing.
        """
        expected = set()
        for src, dst in arch_transition_pairs or []:
            a = str(src).strip().lower()
            b = str(dst).strip().lower()
            if not a or not b or a == b:
                continue
            expected.add((a, b))
        observed = set()
        for tr in transitions or []:
            raw_f = str(tr.get("from", "")).strip().lower()
            raw_t = str(tr.get("to", "")).strip().lower()
            if not raw_f or not raw_t or raw_f == raw_t:
                continue
            observed.add((raw_f, raw_t))
        return [
            {"from": src, "to": dst}
            for src, dst in sorted(expected - observed)
        ]

    def _workspace_root_from_log(self) -> str:
        log_dir = os.path.dirname(os.path.abspath(self.log_file_path))
        if os.path.basename(log_dir) == "ros_info":
            return os.path.dirname(log_dir)
        return log_dir

    def _component_cpp_path(self, component_name: str) -> str:
        root = self._workspace_root_from_log()
        filename = f"{component_name}.cpp"
        for pkg_name in sorted(os.listdir(root)) if os.path.isdir(root) else []:
            candidate = os.path.join(root, pkg_name, "src", "components", filename)
            if os.path.exists(candidate):
                return candidate
        return ""

    def _cpp_has_fsm_edge(self, component_name: str, src: str, dst: str) -> bool:
        cpp_path = self._component_cpp_path(component_name)
        if not cpp_path:
            return False
        try:
            with open(cpp_path, "r", encoding="utf-8", errors="replace") as f:
                code = f.read()
        except OSError:
            return False
        src_pat = re.compile(r'fsm_state_\s*==\s*["\']' + re.escape(src) + r'["\']', re.IGNORECASE)
        dst_pat = re.compile(r'fsm_state_\s*=\s*["\']' + re.escape(dst) + r'["\']', re.IGNORECASE)
        for match in src_pat.finditer(code):
            if dst_pat.search(code[match.end(): match.end() + 2500]):
                return True
        return False

    def _extract_pwm_values(self, message):
        pwm_values_match = re.match(
            r'Published\s+PWM\s+values\s*:\s*(.+)',
            message,
            re.IGNORECASE
        )
        if not pwm_values_match:
            return []
        raw_values = pwm_values_match.group(1).strip()
        return [v.strip() for v in raw_values.split(',') if v.strip()]
    
    def extract_log_behavior(self):
        """extract behavior trajectories from log file"""
        # regular expression to match log lines
        log_pattern = r'(?:\[[^\]]+\]\s+)?\[INFO\]\s+\[(\d+\.\d+)\]\s+\[([^\]]+)\]:\s+([^\n]+)'
        # regular expression to match received messages (supports both 'Received', supports multi-port format)
        receive_pattern = r'(?:Received)\s*(.*)'
        # regular expression to match published messages (supports both 'Published?', supports multi-port format, case-insensitive)
        publish_pattern = r'(?:Published?\s*:?\s*)\s*(.*)'
        
        with open(self.log_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                match = re.match(log_pattern, line.strip())
                if match:
                    timestamp, component_name, message = match.groups()
                    ts = float(timestamp)

                    # Default entity key comes from ROS logger name (usually node name).
                    # If message starts with "[node.component] ...", use that as finer-grained
                    # entity ownership so thread-level behaviors are not collapsed into one node.
                    component_name_lower = component_name.lower()
                    message_body = message.strip()
                    entity_from_prefix = re.match(r'^\[([^\]]+)\]\s*(.*)$', message_body)
                    if entity_from_prefix:
                        prefix_label = entity_from_prefix.group(1).strip().lower()
                        message_body = entity_from_prefix.group(2).strip()
                        if "." in prefix_label:
                            node_part, comp_part = prefix_label.split(".", 1)
                            # Normalize node.main init logs to node-level entity.
                            if comp_part == "main":
                                component_name_lower = node_part
                            else:
                                component_name_lower = f"{node_part}.{comp_part}"
                        else:
                            component_name_lower = prefix_label
                    self._ensure_component_record(component_name_lower)
                    
                    # record event
                    event = {
                        'timestamp': ts,
                        'type': 'unknown',
                        'message': message_body
                    }
                    
                    # check if it's a received message (supports case-insensitive matching)
                    receive_match = re.match(receive_pattern, message_body, re.IGNORECASE)
                    if receive_match:
                        ports_data = receive_match.group(1).strip()
                        port_pairs = self._parse_port_pairs(ports_data)
                        
                        if port_pairs:
                            for port, value in port_pairs:
                                self._append_port_event(
                                    component_name_lower, event, 'receive', ts, port, value
                                )
                            continue
                    
                    # check if it's a published message (supports case-insensitive matching)
                    publish_match = re.match(publish_pattern, message_body, re.IGNORECASE)
                    if publish_match:
                        pwm_values = self._extract_pwm_values(message_body)
                        if pwm_values:
                            for idx, value in enumerate(pwm_values, start=1):
                                self._append_port_event(
                                    component_name_lower, event, 'publish', ts, f"pwm{idx}", value
                                )
                            continue

                        ports_data = publish_match.group(1).strip().lower()
                        prefixes_to_remove = ['pwms:', 'forces', 'forces:', 'pwm values', 'results:', '-', 'outputs:']
                        for prefix in prefixes_to_remove:
                            if prefix in ports_data:
                                ports_data = ports_data.replace(prefix, '')
                        if '>' in ports_data:
                            ports_data = ports_data.replace('>', '')
                        ports_data = ports_data.strip()
                        port_pairs = self._parse_port_pairs(ports_data)
                        
                        if port_pairs:
                            for port, value in port_pairs:
                                self._append_port_event(
                                    component_name_lower, event, 'publish', ts, port, value
                                )
                            continue
                    
                    # state transition line (INFO body)
                    transition_match = _STATE_TRANSITION_LOG_RE.match(message_body.strip())
                    if transition_match:
                        from_state, to_state = transition_match.group(1), transition_match.group(2)
                        self.components_behavior[component_name_lower]['state_transitions'].append({
                            'timestamp': ts,
                            'from': from_state,
                            'to': to_state
                        })
                        event['type'] = 'state_transition'
                        event['from'] = from_state
                        event['to'] = to_state
                        continue
                    
                    # add original event only if it's not a port-specific event or state transition
                    if event['type'] == 'unknown':
                        self.components_behavior[component_name_lower]['events'].append(event)
        
        # sort events by timestamp for each component
        for component in self.components_behavior.values():
            component['events'].sort(key=lambda x: x['timestamp'])
    
    @staticmethod
    def _parse_fsm_from_dict(sm: dict) -> tuple:
        """Extract (state_name_list, transition_pair_list) from a state_machine dict."""
        fsm_state_names: list = []
        fsm_transition_pairs: list = []
        if not isinstance(sm, dict):
            return fsm_state_names, fsm_transition_pairs
        for st in sm.get("states") or []:
            if isinstance(st, dict):
                nm = str(st.get("name", "")).strip()
                if nm:
                    fsm_state_names.append(nm.lower())
        for tr in sm.get("transitions") or []:
            if isinstance(tr, dict):
                src = str(tr.get("source", "")).strip().lower()
                tgt = str(tr.get("target", "")).strip().lower()
                if src and tgt:
                    fsm_transition_pairs.append((src, tgt))
        return fsm_state_names, fsm_transition_pairs

    def extract_ros_components_info(self):
        """extract components and device-nodes info from ROS architecture file"""
        if self.ros_architecture_file.endswith('.json'):
            with open(self.ros_architecture_file, 'r', encoding='utf-8') as f:
                ros_architecture = json.load(f)
            parsed_components = []
            parsed_nodes = []

            # ROS architecture json usually has ROSPackages -> nodes -> components
            if isinstance(ros_architecture, dict) and 'ROSPackages' in ros_architecture:
                for pkg in ros_architecture.get('ROSPackages', []):
                    pkg_name = (pkg.get('name') or '').strip()
                    for node in pkg.get('nodes', []):
                        node_raw = (node.get('name') or '').strip()
                        if node_raw:
                            parsed_nodes.append({'name': node_raw, 'package': pkg_name})
                        node_name = node_raw.lower()
                        components = node.get('components', []) or node.get('components_in_node', [])

                        if components:
                            parsed_components.append({
                                'name': node_raw,
                                'node_name': node_name,
                                'subscribers': [],
                                'publishers': [],
                                'fsm_state_names': [],
                                'fsm_transition_pairs': [],
                                'is_device': False,
                                'entity_kind': 'process',
                            })
                            # Node with sub-components: parse each component
                            for component in components:
                                component_name = component.get('name')
                                if not component_name:
                                    continue
                                component_name = str(component_name).strip()
                                callback_ports = []
                                for callback in component.get('callbacks', []) or []:
                                    if isinstance(callback, dict):
                                        callback_ports.append(callback.get('port') or callback.get('name') or '')
                                    elif isinstance(callback, str):
                                        callback_ports.append(callback)

                                output_ports = []
                                for output in component.get('outputs', []) or []:
                                    if isinstance(output, dict):
                                        output_ports.append(output.get('port') or output.get('name') or '')
                                    elif isinstance(output, str):
                                        output_ports.append(output)

                                fsm_state_names, fsm_transition_pairs = self._parse_fsm_from_dict(
                                    component.get("state_machine") or {}
                                )
                                parsed_components.append({
                                    'name': component_name,
                                    'node_name': node_name,
                                    'subscribers': [p for p in callback_ports if p],
                                    'publishers': [p for p in output_ports if p],
                                    'fsm_state_names': fsm_state_names,
                                    'fsm_transition_pairs': fsm_transition_pairs,
                                    'is_device': False,
                                    'entity_kind': 'thread',
                                })
                        else:
                            # Device/bare node: no sub-components; treat the node itself as the
                            # observable unit (AADL device translates to a standalone ROS node).
                            if not node_raw:
                                continue
                            sub_ports = [
                                p.get('port') or p.get('name') or ''
                                for p in (node.get('subscribers') or [])
                            ]
                            pub_ports = [
                                p.get('port') or p.get('name') or ''
                                for p in (node.get('publishers') or [])
                            ]
                            fsm_state_names, fsm_transition_pairs = self._parse_fsm_from_dict(
                                node.get("state_machine") or {}
                            )
                            parsed_components.append({
                                'name': node_raw,
                                'node_name': node_name,
                                'subscribers': [p for p in sub_ports if p],
                                'publishers': [p for p in pub_ports if p],
                                'fsm_state_names': fsm_state_names,
                                'fsm_transition_pairs': fsm_transition_pairs,
                                'is_device': True,
                                'entity_kind': 'device',
                            })
            # compatible with already-flattened list format and AADL parser list format
            elif isinstance(ros_architecture, list):
                def _extract_ports_from_aadl_ports(ports: list) -> tuple[list, list]:
                    subs = []
                    pubs = []
                    for port in ports or []:
                        if not isinstance(port, dict):
                            continue
                        pname = (port.get('name') or '').strip()
                        direction = (port.get('direction') or '').strip().lower()
                        if not pname:
                            continue
                        if direction == 'in':
                            subs.append(pname)
                        elif direction == 'out':
                            pubs.append(pname)
                    return subs, pubs

                has_aadl_hierarchy = any(
                    isinstance(item, dict) and (
                        item.get('category') in {'system', 'process', 'thread', 'device'}
                        or bool(item.get('subcomponents'))
                    )
                    for item in ros_architecture
                )

                if has_aadl_hierarchy:
                    for system in ros_architecture:
                        if not isinstance(system, dict):
                            continue
                        system_subs = system.get('subcomponents') or []
                        for proc in system_subs:
                            if not isinstance(proc, dict):
                                continue
                            if (proc.get('category') or '').strip().lower() != 'process':
                                continue

                            proc_name_raw = (proc.get('name') or '').strip()
                            if not proc_name_raw:
                                continue
                            proc_name = proc_name_raw.lower()
                            parsed_nodes.append({'name': proc_name_raw, 'package': ''})
                            parsed_components.append({
                                'name': proc_name_raw,
                                'node_name': proc_name,
                                'subscribers': [],
                                'publishers': [],
                                'fsm_state_names': [],
                                'fsm_transition_pairs': [],
                                'is_device': False,
                                'entity_kind': 'process',
                            })

                            threads = proc.get('subcomponents') or []
                            for th in threads:
                                if not isinstance(th, dict):
                                    continue
                                if (th.get('category') or '').strip().lower() != 'thread':
                                    continue
                                th_name_raw = (th.get('name') or '').strip()
                                if not th_name_raw:
                                    continue
                                sub_ports, pub_ports = _extract_ports_from_aadl_ports(th.get('ports') or [])
                                parsed_components.append({
                                    'name': th_name_raw,
                                    'node_name': proc_name,
                                    'subscribers': sub_ports,
                                    'publishers': pub_ports,
                                    'fsm_state_names': [],
                                    'fsm_transition_pairs': [],
                                    'is_device': False,
                                    'entity_kind': 'thread',
                                })
                else:
                    parsed_components = ros_architecture
                    parsed_nodes = []

            self.ros_components = parsed_components
            self.ros_nodes = parsed_nodes
    
    def compare_behavior_with_architecture(self):
        """compare component behavior with ROS architecture"""
        architecture_entities = []
        
        # process _extract_ros_components return structure
        for item in self.ros_components:
            # handle components info from json architecture file
            if isinstance(item, dict) and ('subscribers' in item or 'publishers' in item):
                component_name = item['name']
                key = component_name.lower() if isinstance(component_name, str) else component_name
                nn = (item.get('node_name') or '').strip().lower()
                entity_kind = item.get('entity_kind') or ('device' if item.get('is_device') else 'thread')
                entity_key = f'{nn}.{key}' if entity_kind == 'thread' and nn else key
                info = {
                    'subscribers': item.get('subscribers', []),
                    'publishers': item.get('publishers', []),
                    'fsm_state_names': item.get('fsm_state_names') or [],
                    'fsm_transition_pairs': item.get('fsm_transition_pairs') or [],
                    'is_device': item.get('is_device', False),
                    'entity_kind': entity_kind,
                }
                architecture_entities.append((entity_key, info))
            # handle ports info from aadl-xml file
            elif isinstance(item, dict) and 'ports' in item:
                component_name = item['name']
                key = component_name.lower() if isinstance(component_name, str) else component_name
                info = {
                    'subscribers': [],
                    'publishers': [],
                    'fsm_state_names': [],
                    'fsm_transition_pairs': [],
                    'is_device': False,
                    'entity_kind': 'thread',
                }
                ports = item['ports']
                for port in ports:
                    port_type = port['direction']
                    if port_type == 'in':
                        info['subscribers'].append(port['name'])
                    elif port_type == 'out':
                        info['publishers'].append(port['name'])
                architecture_entities.append((key, info))

        # Compare every AADL entity from architecture, even if it produced no log.
        for component_name, component_info in architecture_entities:
            self._ensure_component_record(component_name)
            self._compare_component_with_architecture(component_name, component_info)
        self.compute_system_cba()
    
    def _compare_component_with_architecture(self, component_name, component_info: dict):
        """compare component behavior with architecture definition"""
        component_behavior = self.components_behavior[component_name]
        expected_subscribers = component_info.get("subscribers") or []
        expected_publishers = component_info.get("publishers") or []

        def _arch_port_str(entry) -> str:
            if isinstance(entry, dict):
                return (entry.get("port") or entry.get("name") or "").strip()
            if isinstance(entry, str):
                return entry.strip()
            return str(entry).strip() if entry else ""

        # Architecture ports -> normalized keys (expected_yaw matches log "expected yaw")
        expected_subscriber_ports = set()
        for sub in expected_subscribers:
            raw = _arch_port_str(sub)
            if raw:
                expected_subscriber_ports.add(self._normalize_port_key(raw))

        expected_publisher_ports = set()
        for pub in expected_publishers:
            raw = _arch_port_str(pub)
            if raw:
                expected_publisher_ports.add(self._normalize_port_key(raw))

        actual_subscriber_ports = set()
        for event in component_behavior["received"]:
            p = (event.get("port") or "").strip()
            if p:
                actual_subscriber_ports.add(self._normalize_port_key(p))

        actual_publisher_ports = set()
        for event in component_behavior["published"]:
            p = (event.get("port") or "").strip()
            if p:
                actual_publisher_ports.add(self._normalize_port_key(p))

        arch_fsm_states = component_info.get("fsm_state_names") or []
        arch_fsm_pairs = component_info.get("fsm_transition_pairs") or []
        bad_fsm_endpoints, bad_fsm_edges, fsm_checked = self._validate_logged_fsm_transitions(
            component_behavior.get("state_transitions") or [],
            arch_fsm_states,
            arch_fsm_pairs,
        )
        missing_fsm_edges = (
            self._missing_logged_fsm_transitions(
                component_behavior.get("state_transitions") or [],
                arch_fsm_pairs,
            )
            if fsm_checked and arch_fsm_pairs
            else []
        )

        # Set overlap on normalized names (underscore vs space in logs is equivalent)
        matched_expected_subscribers = expected_subscriber_ports & actual_subscriber_ports
        matched_expected_publishers = expected_publisher_ports & actual_publisher_ports

        missing_subscriber_ports = expected_subscriber_ports - matched_expected_subscribers
        missing_publisher_ports = expected_publisher_ports - matched_expected_publishers
        
        # extra ports: actual but not matched
        # extra_subscriber_ports = actual_subscriber_ports - matched_actual_subscribers
        # extra_publisher_ports = actual_publisher_ports - matched_actual_publishers
        
        fsm_coverage_warnings = []
        if (
            missing_fsm_edges
            and not bad_fsm_endpoints
            and not bad_fsm_edges
            and not missing_subscriber_ports
            and not missing_publisher_ports
        ):
            still_missing = []
            component_short_name = component_name.split('.')[-1] if '.' in component_name else component_name
            for edge in missing_fsm_edges:
                src = str(edge.get("from", "")).strip().lower()
                dst = str(edge.get("to", "")).strip().lower()
                if src and dst and self._cpp_has_fsm_edge(component_short_name, src, dst):
                    fsm_coverage_warnings.append(edge)
                else:
                    still_missing.append(edge)
            missing_fsm_edges = still_missing

        fsm_mismatch = bool(bad_fsm_endpoints or bad_fsm_edges or missing_fsm_edges)
        # build comparison result
        comparison = {
            'component_name': component_name,
            'is_device': component_info.get('is_device', False),
            'entity_kind': component_info.get('entity_kind', 'device' if component_info.get('is_device') else 'thread'),
            'expected_subscriber_ports': expected_subscriber_ports,
            'actual_subscriber_ports': actual_subscriber_ports,
            'expected_publisher_ports': expected_publisher_ports,
            'actual_publisher_ports': actual_publisher_ports,
            'missing_subscriber_ports': missing_subscriber_ports,
            'missing_publisher_ports': missing_publisher_ports,
            'arch_fsm_transition_pairs': list(arch_fsm_pairs or []),
            'has_mismatch': any([
                missing_subscriber_ports,
                missing_publisher_ports,
                fsm_mismatch,
            ]),
            'total_received_events': len(component_behavior['received']),
            'total_published_events': len(component_behavior['published']),
            'total_state_transitions': len(component_behavior['state_transitions']),
            'fsm_validation_enabled': fsm_checked,
            'expected_fsm_states': sorted(set(arch_fsm_states)),
            'fsm_bad_endpoints': bad_fsm_endpoints,
            'fsm_bad_edges': bad_fsm_edges,
            'fsm_missing_edges': missing_fsm_edges,
            'fsm_coverage_warnings': fsm_coverage_warnings,
        }
        comp_score, comp_weight = cba_from_comparison_result(comparison)
        comparison['component_cba'] = comp_score
        comparison['component_weight'] = comp_weight

        self.comparison_results.append(comparison)

    def compute_system_cba(self) -> float:
        """Compute system-level CBA as contract-element-weighted average."""
        self.system_cba = system_cba_from_comparison_results(self.comparison_results)
        return self.system_cba

    def get_system_cba(self) -> float | None:
        return self.system_cba
    
    def generate_report(self, output_file: str):
        """Generate comparison report"""
        limit = self.REPORT_EVENT_TAIL_LIMIT

        def _tail_events(events: list) -> tuple[list, int]:
            """Return chronologically ordered tail events and total count."""
            ordered = sorted(events, key=lambda x: x['timestamp'])
            total = len(ordered)
            if total <= limit:
                return ordered, total
            return ordered[-limit:], total

        with open(output_file, 'w', encoding='utf-8') as f:
            f.write('# Component Behavior Trajectory and Architecture Comparison Report\n\n')
            f.write(f'Generated at: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            
            n_threads = sum(1 for c in self.ros_components if c.get('entity_kind') == 'thread')
            n_processes = sum(1 for c in self.ros_components if c.get('entity_kind') == 'process')
            n_device_nodes = sum(1 for c in self.ros_components if c.get('entity_kind') == 'device')
            n_logged_entities = sum(
                1
                for c in self.components_behavior.values()
                if c['received'] or c['published'] or c['state_transitions'] or c['events']
            )

            # Overview
            f.write('## 1. Overview\n\n')
            f.write(f'- Log file analyzed: {self.log_file_path}\n')
            f.write(f'- ROS architecture file: {self.ros_architecture_file}\n')
            f.write(f'- Number of entities found in log: {n_logged_entities}\n')
            f.write(f'- Number of threads defined in architecture: {n_threads}\n')
            f.write(f'- Number of processes defined in architecture: {n_processes}\n')
            f.write(f'- Number of device nodes defined in architecture: {n_device_nodes}\n')
            f.write(f'- Number of nodes defined in architecture: {len(self.ros_nodes)}\n\n')
            
            # Comparison Results Statistics
            total_entities = len(self.comparison_results)
            matched_entities = sum(1 for r in self.comparison_results if not r['has_mismatch'])
            mismatched_entities = total_entities - matched_entities
            
            f.write('## 2. Comparison Results Statistics\n\n')
            f.write(f'- Total entities compared: {total_entities}\n')
            f.write(f'- Matched entities: {matched_entities}\n')
            f.write(f'- Mismatched entities: {mismatched_entities}\n')
            cba_val = self.system_cba if self.system_cba is not None else self.compute_system_cba()
            f.write(f'- System behavior consistency (CBA): {cba_val:.6f}\n\n')
            
            f.write('## 3. Component Details\n\n')
            # create a mapping from component name to comparison result
            component_results = {}
            for result in self.comparison_results:
                component_results[result['component_name']] = result
            
            # generate details for each component
            for component_name in sorted(self.components_behavior.keys()):
                f.write(f'### 3.1 {component_name}\n\n')
                f.write('#### 3.1.1 Component Behavior Trajectory\n\n')
                # Received Events
                behavior = self.components_behavior[component_name]
                if behavior['received']:
                    f.write('##### Received Events\n\n')
                    received_tail, received_total = _tail_events(behavior['received'])
                    for event in received_tail:
                        f.write(f'- [{event["timestamp"]:.6f}] received {event["port"]}')
                        if event['value'] is not None:
                            f.write(f': {event["value"]}')
                        f.write('\n')
                    if received_total > limit:
                        f.write(f'- ... truncated: showing latest {limit} of {received_total} received events\n')
                    f.write('\n')
                
                # Published Events
                if behavior['published']:
                    f.write('##### Published Events\n\n')
                    published_tail, published_total = _tail_events(behavior['published'])
                    for event in published_tail:
                        f.write(f'- [{event["timestamp"]:.6f}] published {event["port"]}')
                        if event['value'] is not None:
                            f.write(f': {event["value"]}')
                        f.write('\n')
                    if published_total > limit:
                        f.write(f'- ... truncated: showing latest {limit} of {published_total} published events\n')
                    f.write('\n')
                
                # State Transitions
                if behavior['state_transitions']:
                    f.write('##### State Transitions\n\n')
                    transition_tail, transition_total = _tail_events(behavior['state_transitions'])
                    for event in transition_tail:
                        f.write(f'- [{event["timestamp"]:.6f}] state transition from {event["from"]} to {event["to"]}\n')
                    if transition_total > limit:
                        f.write(f'- ... truncated: showing latest {limit} of {transition_total} state transitions\n')
                    f.write('\n')
                
                # 2. Component Matching Analysis
                f.write('#### 3.1.2 Component Matching Analysis\n\n')
                if component_name in component_results:
                    result = component_results[component_name]
                    self._write_component_comparison(f, result)
                else:
                    f.write('- this entity is not defined in the architecture (neither as a component nor as a device node), so it cannot be matched.\n\n')
    
    def _write_component_comparison(self, file, result):
        """Write component comparison results"""
        kind_by_entity = {
            "device": "Device Node",
            "process": "Process",
            "thread": "Thread",
        }
        kind = kind_by_entity.get(result.get("entity_kind"), "Component")
        file.write(f'#### {kind}: {result["component_name"]}\n\n')
        file.write(f'- Status: {"✓ Fully Matched" if not result["has_mismatch"] else "✗ Mismatched"}\n')
        file.write(f'- Total Received Events: {result["total_received_events"]}\n')
        file.write(f'- Total Published Events: {result["total_published_events"]}\n')
        file.write(f'- Total State Transitions: {result["total_state_transitions"]}\n')
        if result.get("fsm_validation_enabled"):
            file.write(f'- Expected FSM States (architecture): {result.get("expected_fsm_states", [])}\n')
            if result.get("fsm_bad_endpoints"):
                file.write(f'  - **FSM invalid endpoints:\n** {_summarize_fsm_errors(result["fsm_bad_endpoints"])}\n')
            if result.get("fsm_bad_edges"):
                file.write(f'  - **FSM invalid edges:\n** {_summarize_fsm_errors(result["fsm_bad_edges"])}\n')
            if result.get("fsm_missing_edges"):
                file.write(f'  - **FSM transitions not observed/implemented:\n** {result["fsm_missing_edges"]}\n')
            if result.get("fsm_coverage_warnings"):
                file.write(
                    f'  - **CoverageWarning:** FSM transitions implemented in code but not observed in this run: '
                    f'{result["fsm_coverage_warnings"]}\n'
                )
            if not result.get("fsm_bad_endpoints") and not result.get("fsm_bad_edges") and not result.get("fsm_missing_edges"):
                file.write('  - FSM transition check: **ok** (endpoints and edges)\n')
        file.write('\n')
        
        file.write(f'  **Subscriber Port Comparison:**\n')
        file.write(f'  - Expected Subscriber Ports: {sorted(result["expected_subscriber_ports"])}\n')
        file.write(f'  - Actual Subscriber Ports: {sorted(result["actual_subscriber_ports"])}\n')
        
        if result["missing_subscriber_ports"]:
            file.write(f'  - Missing Subscriber Ports in Log: {sorted(result["missing_subscriber_ports"])}\n')
        
        file.write(f'  **Publisher Port Comparison:**\n')
        file.write(f'  - Expected Publisher Ports: {sorted(result["expected_publisher_ports"])}\n')
        file.write(f'  - Actual Publisher Ports: {sorted(result["actual_publisher_ports"])}\n')
        
        if result["missing_publisher_ports"]:
            file.write(f'  - Missing Publisher Ports in Log: {sorted(result["missing_publisher_ports"])}\n')
        
        file.write('\n')
    
    def get_errors(self) -> list:
        """Return node_errors-compatible records for all mismatched components.

        Each component produces at most one record; all port mismatches are
        merged into a single exception_message.

        Each record follows the same schema as topic_validator errors:
            {"node": None, "component": ..., "function": ...,
             "error_type": ..., "exception_message": ...}
        """
        errors = []
        seen_components = set()

        for result in self.comparison_results:
            if not result["has_mismatch"]:
                continue
            component = result["component_name"].split('.')[-1] if '.' in result["component_name"] else result["component_name"]
            if component in seen_components:
                continue
            seen_components.add(component)

            parts = []
            if result["missing_subscriber_ports"]:
                parts.append(
                    f"missing subscriber ports {sorted(result['missing_subscriber_ports'])}"
                )
            if result["missing_publisher_ports"]:
                parts.append(
                    f"missing publisher ports {sorted(result['missing_publisher_ports'])}"
                )
            fsm_root_cause = ""
            fsm_enforced_rule = ""
            if result.get("fsm_bad_endpoints") or result.get("fsm_bad_edges") or result.get("fsm_missing_edges"):
                fsm_root_cause, fsm_enforced_rule = _fsm_mismatch_guidance(result)
                parts.append(f"FSM diagnosis: {fsm_root_cause}")
            msg = f"Component '{component}' runtime mismatch: " + "; ".join(parts)
            error_record = {
                "node": None,
                "component": component,
                "function": "runtime_analysis",
                "error_type": "BehaviorError",
                "exception_message": msg,
            }
            if fsm_root_cause:
                error_record["root_cause_analysis"] = fsm_root_cause
                error_record["enforced_rule"] = fsm_enforced_rule
            errors.append(error_record)

        return errors

    def get_coverage_warnings(self) -> dict:
        """Return CoverageWarning records grouped by component for errors_history."""
        warnings = {}
        for result in self.comparison_results:
            edges = result.get("fsm_coverage_warnings") or []
            if not edges:
                continue
            component = result["component_name"].split('.')[-1] if '.' in result["component_name"] else result["component_name"]
            warnings.setdefault(component, []).append({
                "kind": "CoverageWarning",
                "warning_number": len(edges),
                "message": (
                    "FSM transitions are implemented in generated code but were not observed "
                    f"in runtime logs: {edges}"
                ),
                "fsm_missing_edges": edges,
            })
        return warnings

    def run(self, output_file: str):
        """run the complete analysis process"""
        print(f"extracting component behavior from log file: {self.log_file_path}")
        self.extract_log_behavior()
        
        print(f"extracting component information from ros architecture file: {self.ros_architecture_file}")
        self.extract_ros_components_info()
        
        print("comparing component behavior with architecture definition...")
        self.compare_behavior_with_architecture()
        
        print(f"generating report: {output_file}")
        self.generate_report(output_file)
        
        print("analysis completed!")
        # print statistics
        total_entities = len(self.comparison_results)
        matched_entities = sum(1 for r in self.comparison_results if not r['has_mismatch'])
        print(f"- total entities compared: {total_entities}")
        print(f"- entities matched: {matched_entities}")
        print(f"- entities mismatched: {total_entities - matched_entities}")
        cba_val = self.system_cba if self.system_cba is not None else self.compute_system_cba()
        print(f"- system behavior consistency (CBA): {cba_val:.6f}")

def main():
    parser = argparse.ArgumentParser(description='runtime analysis tool with ROS architecture')
    parser.add_argument('-l', '--log', type=str, default='/ros_info/node.log',
                        help='ROS node log file path')
    parser.add_argument('-a', '--architecture', type=str, default='Flight_Controller_ros.json',
                        help='ROS architecture file path')
    parser.add_argument('-o', '--output', type=str, default='./runtime_analysis_report.txt',
                        help='output report file path')
    
    args = parser.parse_args()
    
    analysis = RuntimeAnalysis(args.log, args.architecture)
    analysis.run(args.output)

if __name__ == '__main__':
    main()