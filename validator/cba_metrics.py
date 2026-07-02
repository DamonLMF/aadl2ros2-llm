# -*- coding: utf-8 -*-
"""
Runtime behavior agreement (CBA) per paper/chapter_4.md.

Per component c:
  Pcontract = total declared subscriber + publisher ports
  Tcontract = total declared state-transition edges
  Pobs / Tobs = observed ports / transitions in runtime logs

  rho_p = Pobs / Pcontract,  rho_t = Tobs / Tcontract
  If Pcontract=0, port weight omega_p=0 (excluded from score_c).
  If Tcontract=0, state weight omega_t=0 (excluded from score_c).
  If both > 0: omega_p=P/(P+T), omega_t=T/(P+T), score_c = omega_p*rho_p + omega_t*rho_t
  If only one > 0: score_c uses that dimension's coverage only.

System:
  Wc = Pcontract + Tcontract
  CBA = sum(Wc * score_c) / sum(Wc)  (only components with Wc > 0)
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple


def _coverage_ratio(observed: int, contract: int) -> float:
    """Coverage ratio when contract count > 0; clamped to [0, 1]."""
    if contract <= 0:
        raise ValueError("contract must be positive for coverage ratio")
    return max(0.0, min(1.0, observed / contract))


def count_contract_fsm_edges(arch_transition_pairs: Sequence) -> int:
    """Count contract FSM edges (self-loops excluded)."""
    count = 0
    for pair in arch_transition_pairs or []:
        if isinstance(pair, (list, tuple)) and len(pair) >= 2:
            src, dst = str(pair[0]).strip().lower(), str(pair[1]).strip().lower()
        elif isinstance(pair, dict):
            src = str(pair.get("from", "")).strip().lower()
            dst = str(pair.get("to", "")).strip().lower()
        else:
            continue
        if src and dst and src != dst:
            count += 1
    return count


def component_behavior_score(
    p_contract: int,
    t_contract: int,
    p_obs: int,
    t_obs: int,
) -> Tuple[float, int]:
    """
    Return (component behavior score, component weight Wc).
    Dimensions with Pcontract=0 or Tcontract=0 get zero weight and are omitted from score_c.
    """
    weight = p_contract + t_contract
    if weight <= 0:
        return 1.0, 0

    score = 0.0
    if p_contract > 0 and t_contract > 0:
        rho_p = _coverage_ratio(p_obs, p_contract)
        rho_t = _coverage_ratio(t_obs, t_contract)
        omega_p = p_contract / weight
        omega_t = t_contract / weight
        score = omega_p * rho_p + omega_t * rho_t
    elif p_contract > 0:
        score = _coverage_ratio(p_obs, p_contract)
    elif t_contract > 0:
        score = _coverage_ratio(t_obs, t_contract)

    return score, weight


def system_cba_from_component_scores(
    scores_and_weights: Sequence[Tuple[float, int]],
) -> float:
    """Compute system-level CBA from (score_c, Wc) pairs."""
    weighted = [(s, w) for s, w in scores_and_weights if w > 0]
    if not weighted:
        return 1.0
    total_w = sum(w for _, w in weighted)
    if total_w <= 0:
        return 1.0
    return sum(s * w for s, w in weighted) / total_w


def cba_from_comparison_result(result: Dict[str, Any]) -> Tuple[float, int]:
    """
    Compute score and Wc from a single runtime_analysis comparison result.
    Expects port sets, arch_fsm_transition_pairs, fsm_missing_edges, etc.
    """
    expected_sub = result.get("expected_subscriber_ports") or set()
    expected_pub = result.get("expected_publisher_ports") or set()
    if not isinstance(expected_sub, set):
        expected_sub = set(expected_sub)
    if not isinstance(expected_pub, set):
        expected_pub = set(expected_pub)

    missing_sub = result.get("missing_subscriber_ports") or set()
    missing_pub = result.get("missing_publisher_ports") or set()
    if not isinstance(missing_sub, set):
        missing_sub = set(missing_sub)
    if not isinstance(missing_pub, set):
        missing_pub = set(missing_pub)

    p_contract = len(expected_sub) + len(expected_pub)
    p_obs = p_contract - len(missing_sub) - len(missing_pub)

    arch_pairs = result.get("arch_fsm_transition_pairs") or []
    t_contract = count_contract_fsm_edges(arch_pairs)
    missing_fsm = result.get("fsm_missing_edges") or []
    if result.get("fsm_validation_enabled") and t_contract > 0:
        t_obs = max(0, t_contract - len(missing_fsm))
    elif t_contract > 0:
        t_obs = 0
    else:
        t_obs = 0

    return component_behavior_score(p_contract, t_contract, p_obs, t_obs)


def system_cba_from_comparison_results(
    comparison_results: Sequence[Dict[str, Any]],
) -> float:
    scores = [cba_from_comparison_result(r) for r in comparison_results]
    return system_cba_from_component_scores(scores)
