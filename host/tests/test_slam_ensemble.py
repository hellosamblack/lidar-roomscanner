"""Tests for tools/slam_ensemble.py.

The SLAM pass itself is a GPU integration test (validated on-rig against the
recorded coffeeRoomCircuitNoMnt baseline, 0.74 +/- 0.19 m over 23.9 m). What is
unit-testable here is the part that decides what the numbers MEAN: the
perturbation set, and the split of a start-to-end gap into horizontal drift and
vertical error, which depends on the world-up convention being right.
"""
from __future__ import annotations

import numpy as np

from tools.slam_ensemble import (MAX_DIST_DELTAS, compare, perturbations,
                                 split_closure)


def test_perturbations_are_deterministic():
    assert perturbations(7) == perturbations(7)


def test_a_small_ensemble_still_spans_the_perturbation_space():
    # A 5-run ensemble must not be five copies of the same run, or the sd it
    # reports is zero for the wrong reason.
    p = perturbations(5)
    assert len({x["max_dist_delta"] for x in p}) == len(MAX_DIST_DELTAS)
    assert len({x["start_frame"] for x in p}) > 1
    assert len({(x["start_frame"], x["max_dist_delta"]) for x in p}) == 5


def test_perturbations_stay_numerically_innocuous():
    # 1e-4 against the 0.05 m correspondence radius is 0.2%: enough to move the
    # solver onto a different chaotic path, far too small to change the physics.
    for x in perturbations(12):
        assert abs(x["max_dist_delta"]) <= 1e-4
        assert x["start_frame"] <= 4


def _pose(x, y, z):
    p = np.eye(4)
    p[:3, 3] = (x, y, z)
    return p


def test_closure_splits_on_the_real_world_up_axis():
    # Up is -Y on this rig (slam.frames.world_up()), NOT -Z. A 3-4-5 displacement
    # laid in the horizontal plane must read as 5 m of drift and no height error.
    r = split_closure([_pose(0, 0, 0), _pose(3, 0, 4)])

    assert abs(r["horizontal_closure_m"] - 5.0) < 1e-9
    assert abs(r["vertical_error_m"]) < 1e-9


def test_vertical_error_is_positive_when_the_run_ends_higher():
    # +Y is DOWN, so ending at -Y means ending above the start.
    up = split_closure([_pose(0, 0, 0), _pose(0, -0.25, 0)])
    down = split_closure([_pose(0, 0, 0), _pose(0, 0.25, 0)])

    assert up["vertical_error_m"] > 0
    assert down["vertical_error_m"] < 0
    assert up["horizontal_closure_m"] < 1e-9, "pure height change is not drift"


def test_horizontal_closure_ignores_height_but_total_closure_does_not():
    r = split_closure([_pose(0, 0, 0), _pose(3, -4, 0)])

    assert abs(r["horizontal_closure_m"] - 3.0) < 1e-9
    assert abs(r["vertical_error_m"] - 4.0) < 1e-9
    assert abs(r["closure_m"] - 5.0) < 1e-9


def test_a_degenerate_trajectory_does_not_explode():
    assert split_closure([])["horizontal_closure_m"] == 0.0
    assert split_closure([_pose(0, 0, 0)])["closure_m"] == 0.0


def test_compare_feeds_the_paired_gate_the_fields_it_needs():
    # The gate keys on horizontal_closure_m / lost / died -- the whole reason this
    # tool computes horizontal_closure_m, which nothing else in the repo produced.
    base = {"runs": [{"horizontal_closure_m": 1.0, "lost": 0, "died": False}
                     for _ in range(6)]}
    better = {"runs": [{"horizontal_closure_m": 0.4, "lost": 0, "died": False}
                       for _ in range(6)]}

    r = compare(base, better)

    assert r["accepted"] is True
    assert r["mean_improvement_m"] > 0
    assert r["ci95_m"][0] > 0


def test_compare_rejects_an_arm_that_died():
    base = {"runs": [{"horizontal_closure_m": 1.0, "lost": 0, "died": False}
                     for _ in range(6)]}
    dead = {"runs": [{"horizontal_closure_m": 0.1, "lost": 0, "died": True}
                     for _ in range(6)]}

    r = compare(base, dead)

    assert r["accepted"] is False, "a dead run's closure is not a measurement"
