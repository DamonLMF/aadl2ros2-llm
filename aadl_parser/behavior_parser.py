#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AADL Behavior Specification Parser
Parses the behavior_specification annex in AADL models
Extracts variables, states, transition logic, actions, and conditions
"""

import re
from typing import Any, Dict, List


def parse_behavior_specification(body: str) -> Dict[str, Any]:
    """
    Main function to parse behavior specification
    Args:
        body: behavior_specification body content
    Returns:
        Dictionary containing variables, states, transitions, actions, and conditions
    """
    if not body or not isinstance(body, str):
        return {
            "variables": [],
            "states": [],
            "transitions": [],
            "actions": [],
            "conditions": []
        }
    
    body = body.strip()
    # parse each part
    variables = parse_variables(body)
    states = parse_states(body)
    transitions = parse_transitions(body)
    
    return {
        "variables": variables,
        "states": states,
        "transitions": transitions,
    }

def parse_variables(body: str) -> List[Dict[str, str]]:
    """
    Parse variable declarations
    Format: variables var1 : Type1; var2 : Type2; ...
    """
    variables = []
    var_match = re.search(r'variables\s+([^;]+(?:;\s*[^;]+)*?)(?=\s*;\s*states|\s*states)', body, re.IGNORECASE)
    if var_match:
        var_section = var_match.group(1).strip()
        
        # Declaration of segmented variables (separated by ;)
        var_declarations = [v.strip() for v in var_section.split(';') if v.strip()]
        
        for var_decl in var_declarations:
            initial_value = None
            if "Initial_Value" in var_decl:
                var_decl, properties_content = var_decl.split("{", 1)
                properties_value = properties_content.split("=>", 1)[-1].strip()
                initial_value= properties_value.replace('"', '').replace('(', '').replace(')', '')
            # Parse each variable declaration: name : type
            if ':' in var_decl and not var_decl.strip().startswith('states'):
                parts = var_decl.split(':', 1)
                var_name = parts[0].strip()
                var_type = parts[1].strip()
                
                # Ensure it's not a state declaration
                if not any(keyword in var_type.lower() for keyword in ['initial', 'final', 'complete', 'state']):
                    variables.append({
                        "name": var_name,
                        "type": var_type,
                        "initial_value": initial_value if initial_value is not None else ""
                    })
    
    return variables

def parse_states(body: str) -> List[Dict[str, Any]]:
    """
    Parse state declarations
    Format: states state1 : initial complete state; state2 : final state; ...
    """
    states = []
    
    # Find states declaration, using more precise regex
    state_match = re.search(r'states\s+([^;]+(?:;\s*[^;]+)*?)(?=\s*;\s*transitions|\s*transitions)', body, re.IGNORECASE)
    if state_match:
        state_section = state_match.group(1).strip()
        
        # Segmentation of state declarations (separated by ;)
        state_declarations = [s.strip() for s in state_section.split(';') if s.strip()]
        
        for state_decl in state_declarations:
            # Parse each state declaration: name : properties
            # Handles both "s0 : initial state;" and "s1, s2 : state;"
            if ':' in state_decl and not state_decl.strip().startswith('transitions'):
                parts = state_decl.split(':', 1)
                state_props = parts[1].strip()
                
                # Ensure it contains the state keyword
                if 'state' in state_props.lower():
                    # A single declaration may list several names: s1, s2 : state;
                    for state_name in [n.strip() for n in parts[0].split(',') if n.strip()]:
                        states.append({
                            "name": state_name,
                            "properties": state_props,
                        })
    
    return states

def parse_transitions(body: str) -> List[Dict[str, Any]]:
    """
    Parse transition declarations
    Format: transitions source -[condition]-> target { actions };
    """
    transitions = []
    
    # Find transitions declaration, using more precise regex
    trans_match = re.search(r'transitions\s+(.+?)(?:;\s*$|$)', body, re.IGNORECASE | re.DOTALL)
    if trans_match:
        trans_section = trans_match.group(1).strip()
        
        # Parse transition pattern: source -[condition]-> target { actions }
        # Also handles transitions without an action block: source -[condition]-> target ;
        trans_pattern_with_actions = r'(\w+)\s*-\[([^\]]+)\]->\s*(\w+)\s*\{([^}]+)\}'
        trans_pattern_no_actions = r'(\w+)\s*-\[([^\]]+)\]->\s*(\w+)\s*;'

        # Collect source-target pairs already captured (with actions take priority)
        seen_pairs: set = set()
        for source, condition, target, actions_str in re.findall(trans_pattern_with_actions, trans_section):
            key = (source.strip(), target.strip())
            seen_pairs.add(key)
            transitions.append({
                "source": source.strip(),
                "target": target.strip(),
                "condition": condition.strip(),
                "actions": parse_actions(actions_str),
            })

        for source, condition, target in re.findall(trans_pattern_no_actions, trans_section):
            key = (source.strip(), target.strip())
            if key not in seen_pairs:
                seen_pairs.add(key)
                transitions.append({
                    "source": source.strip(),
                    "target": target.strip(),
                    "condition": condition.strip(),
                    "actions": "",
                })
    
    return transitions

def parse_actions(actions_str: str) -> List[Dict[str, str]]:
    """
    Parse action strings
    Save all action codes as one action, only keep the source code
    """
    actions = ''
    
    if not actions_str:
        return actions
    # Save the entire action block as one action, only keep the source code
    for action in actions_str:
        actions += action
    return actions