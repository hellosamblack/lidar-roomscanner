# pyright: strict
"""Pyright NEGATIVE fixture for the frame-tagged types (issue #225, BUG-026).

Every line tagged `# EXPECT-ERROR` must produce exactly one Pyright error and
no other line may. `tests/test_pyright_strict.py::test_frame_tags_reject_mismatch`
asserts that -- so this file documents, line by line, which wrong programs the
frame tags can actually see. It is never imported at runtime.
"""
from __future__ import annotations

import numpy as np

from roomscan.domain import CV, Body, Cloud, CvWorld, Rotation, World

_I = np.eye(3)
_P = np.zeros((4, 3))

r_body_from_cv: Rotation[Body, CV] = Rotation(_I, Body, CV)
r_world_from_body: Rotation[World, Body] = Rotation(_I, World, Body)
r_cvworld_from_world: Rotation[CvWorld, World] = Rotation(_I, CvWorld, World)
cv_cloud: Cloud[CV] = Cloud(_P, CV)
body_cloud: Cloud[Body] = Cloud(_P, Body)

# --- accepted: the documented sandwich, right to left ------------------------
display: Rotation[CvWorld, CV] = r_cvworld_from_world @ r_world_from_body @ r_body_from_cv
world_cloud: Cloud[CvWorld] = display @ cv_cloud
also_world: Cloud[CvWorld] = display.apply(cv_cloud)
back: Cloud[CV] = display.inverse() @ world_cloud

# --- rejected ---------------------------------------------------------------
# BUG-026 itself: a CV-frame cloud handed out as the render-world cloud.
bug026: Cloud[CvWorld] = cv_cloud                                         # EXPECT-ERROR
# Rotating a Body cloud with a CvWorld <- World rotation (wrong source frame).
wrong_src = r_cvworld_from_world @ body_cloud                             # EXPECT-ERROR
# The sandwich with its middle missing: CvWorld <- World composed with Body <- CV.
no_middle = r_cvworld_from_world @ r_body_from_cv                         # EXPECT-ERROR
# Right maths, wrong declared frame: this IS a Body cloud, not a World one.
mislabelled: Cloud[World] = r_body_from_cv @ cv_cloud                     # EXPECT-ERROR
# Inverse applied the wrong way round.
inv_wrong: Cloud[CV] = display.inverse() @ cv_cloud                       # EXPECT-ERROR
# A rotation is not a cloud and a cloud is not a rotation.
not_a_rotation: Rotation[CvWorld, CV] = world_cloud                       # EXPECT-ERROR
