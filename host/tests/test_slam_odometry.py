import numpy as np
import open3d as o3d
import pytest

from roomscan.slam.odometry import register, RegistrationResult


def _plane_cloud(n=40, z=1.0):
    xs, ys = np.meshgrid(np.linspace(-0.5, 0.5, n), np.linspace(-0.4, 0.4, n))
    pts = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1).astype(np.float32)
    # add mild curvature so ICP has translational grip in x and y too
    pts[:, 2] += 0.15 * (pts[:, 0] ** 2 + pts[:, 1] ** 2)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def test_translation_recovered():
    target = _plane_cloud()
    src_pts = target.point.positions.numpy().copy()
    shift = np.array([0.03, -0.02, 0.04], dtype=np.float32)
    src_pts += shift
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts)
    res = register(source, target, np.eye(4), mode="translation")
    assert res.ok
    # source-to-target moves source back by -shift
    assert np.allclose(res.pose[:3, 3], -shift, atol=0.01)
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-9)   # rotation held


def test_6dof_leaves_rotation_free():
    # Strengthened beyond the brief: identical clouds registered in 6dof mode
    # from init=eye(4) must converge to (near-)identity — this genuinely
    # exercises the "6dof runs unmodified ICP" behavior instead of merely
    # checking isinstance/shape, which would pass even for a stub.
    target = _plane_cloud()
    src = _plane_cloud()
    res = register(src, target, np.eye(4), mode="6dof")
    assert isinstance(res, RegistrationResult)
    assert res.pose.shape == (4, 4)
    assert res.ok
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-2)
    assert np.allclose(res.pose[:3, 3], np.zeros(3), atol=1e-2)


def _corner_cloud(n=15):
    """Inside corner of a unit box: three mutually perpendicular faces
    (z=0, x=0, y=0), giving point normals along all three axes so 6dof
    point-to-plane ICP has enough structure to constrain a full 3-DoF
    rotation (unlike the single near-planar `_plane_cloud`, which suffers
    rotational ambiguity)."""
    lin = np.linspace(0.05, 0.95, n)
    a, b = np.meshgrid(lin, lin)
    a = a.ravel()
    b = b.ravel()
    zeros = np.zeros_like(a)
    floor = np.stack([a, b, zeros], axis=1)    # z=0 face, normal ~ +/-z
    wall_x = np.stack([zeros, a, b], axis=1)   # x=0 face, normal ~ +/-x
    wall_y = np.stack([a, zeros, b], axis=1)   # y=0 face, normal ~ +/-y
    pts = np.concatenate([floor, wall_x, wall_y], axis=0).astype(np.float32)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def _rotation_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def test_mode_switch_is_discriminating():
    # Discriminates the mode branch: source is the corner cloud rotated by a
    # KNOWN, nonzero rotation about a body diagonal (constrained by all three
    # perpendicular faces), with init_pose = eye(4). 6dof ICP must recover a
    # rotation close to the true inverse rotation (not identity); translation
    # mode on the identical inputs must hold the prior's rotation exactly
    # (= identity here), regardless of what ICP itself estimates. A mutation
    # forcing the rotation-override to fire unconditionally (`if True:`)
    # would collapse the 6dof result's rotation to identity too, failing the
    # first assertion below.
    target = _corner_cloud()
    tgt_pts = target.point.positions.numpy()
    centroid = tgt_pts.mean(axis=0)

    axis = np.array([1.0, 1.0, 1.0])
    angle = np.deg2rad(7.0)
    R_true = _rotation_matrix(axis, angle)

    src_pts = (R_true @ (tgt_pts - centroid).T).T + centroid
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts.astype(np.float32))

    res_6dof = register(source, target, np.eye(4), mode="6dof", max_dist=0.2)
    assert res_6dof.ok
    # ICP aligns source (= R_true @ target, about the shared centroid) back
    # onto target, so the recovered rotation should be R_true's inverse
    # (== transpose, since it's a rotation) -- and clearly not identity.
    assert np.allclose(res_6dof.pose[:3, :3], R_true.T, atol=0.05)
    assert not np.allclose(res_6dof.pose[:3, :3], np.eye(3), atol=0.05)

    res_translation = register(source, target, np.eye(4), mode="translation", max_dist=0.2)
    assert res_translation.ok
    assert np.allclose(res_translation.pose[:3, :3], np.eye(3), atol=1e-9)


def test_low_overlap_trips_gate():
    target = _plane_cloud()
    far = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    far.point.positions = o3d.core.Tensor((target.point.positions.numpy() + 5.0).astype(np.float32))
    res = register(far, target, np.eye(4), mode="translation", max_dist=0.05)
    assert not res.ok


def _flat_plane_cloud(n=40, z=1.0):
    """Perfectly flat, coplanar grid (no curvature, unlike _plane_cloud) --
    all normals identical. Point-to-plane ICP's 6x6 normal-equations solve is
    singular on this geometry (e.g. a ToF sensor square to a blank wall)."""
    xs, ys = np.meshgrid(np.linspace(-0.5, 0.5, n), np.linspace(-0.4, 0.4, n))
    pts = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1).astype(np.float32)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def test_singular_geometry_returns_not_ok():
    # Open3D's point-to-plane ICP raises RuntimeError("... Singular 6x6 linear
    # system detected, tracking failed.") on a perfectly flat, texture-poor
    # target. register() must degrade to a not-ok result instead of letting
    # the exception propagate and crash the mapper.
    target = _flat_plane_cloud()
    source = _flat_plane_cloud()
    res = register(source, target, np.eye(4), mode="translation")
    assert isinstance(res, RegistrationResult)
    assert not res.ok
    assert res.fitness == 0.0
    assert res.rmse == float("inf")
    assert np.allclose(res.pose, np.eye(4))


def test_translation_gate_reflects_genuine_translation_fit_not_stale_6dof():
    # Task 9.5 Lever 2 regression guard: translation mode must gate on the
    # ACTUAL translation-only alignment quality, not a full 6-DoF ICP result
    # whose rotation is discarded afterward. A large (40 deg) rotation about
    # the corner's body diagonal is a case where 6-DoF ICP still converges
    # perfectly (rotation absorbs all the error, fitness=1.0) but a genuine
    # translation-only fit cannot correct a rotation and should fail the
    # default gate (min_fitness=0.3). The old "run 6dof, keep its fitness,
    # override rotation after" implementation would have reported this
    # ok=True (reusing the 6dof fit's fitness=1.0) even though the returned
    # translation-only pose does not actually align the clouds well -- a
    # silently corrupt integration. Confirmed empirically before writing this
    # test (see profiling notes): 6dof fitness=1.0 vs translation fitness
    # ~0.28 at this angle.
    target = _corner_cloud()
    tgt_pts = target.point.positions.numpy()
    centroid = tgt_pts.mean(axis=0)
    axis = np.array([1.0, 1.0, 1.0])
    angle = np.deg2rad(40.0)
    R_true = _rotation_matrix(axis, angle)
    src_pts = (R_true @ (tgt_pts - centroid).T).T + centroid
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts.astype(np.float32))

    res_6dof = register(source, target, np.eye(4), mode="6dof", max_dist=0.2)
    assert res_6dof.ok
    assert res_6dof.fitness > 0.9   # 6dof genuinely converges on this case

    res_translation = register(source, target, np.eye(4), mode="translation")
    assert not res_translation.ok  # genuine translation-only fit correctly rejects it
    assert np.allclose(res_translation.pose[:3, :3], np.eye(3), atol=1e-9)
