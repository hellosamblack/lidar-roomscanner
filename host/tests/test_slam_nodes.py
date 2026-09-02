"""Per-node cloud store beside the TSDF (Issue #150).

The structural blocker the pose-graph backbone spec names
(`docs/superpowers/specs/2026-08-19-slam-posegraph-backbone-design.md` §Decision 3)
is that our poses are baked into the `VoxelBlockGrid` at integrate time, so
correcting a pose means de-integrating. RTAB-Map applies a graph
re-optimization as *one mat4 write per node*. These tests pin exactly that
property: a node keeps its cloud in the SENSOR frame, so `set_pose`/
`update_poses` re-poses the map without touching a single point.

Quantities, not types: every assertion here names an exact point count, an
exact byte count, or an exact coordinate, because the failure this store has to
survive is "the store silently kept the world-frame points" -- which is the
right *type* and the wrong *data*.
"""
import numpy as np
import pytest

from roomscan.slam.nodes import MapNode, NodeCloudStore


def _pts(n: int) -> np.ndarray:
    """n distinct points, so a decimation stride is checkable value-by-value."""
    a = np.arange(n * 3, dtype=np.float32).reshape(n, 3)
    return a


def _pose(tx=0.0, ty=0.0, tz=0.0, yaw_deg=0.0) -> np.ndarray:
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    T[:3, 3] = (tx, ty, tz)
    return T


# ---- the empty store -------------------------------------------------------

def test_empty_store_reports_zero_nodes_points_and_bytes():
    store = NodeCloudStore()
    assert len(store) == 0
    assert store.point_count == 0
    assert store.nbytes == 0
    assert store.dropped == 0
    assert store.world_cloud().shape == (0, 3)


# ---- admission + ids -------------------------------------------------------

def test_added_nodes_get_sequential_ids_and_keep_every_point_at_stride_one():
    store = NodeCloudStore()
    a = store.add(_pose(), _pts(10), frame_index=3)
    b = store.add(_pose(tx=1.0), _pts(7), frame_index=9)
    assert (a.node_id, b.node_id) == (0, 1)
    assert len(store) == 2
    assert (a.point_count, b.point_count) == (10, 7)
    assert store.point_count == 17
    assert (a.frame_index, b.frame_index) == (3, 9)
    assert store.get(1) is b
    assert [n.node_id for n in store.nodes] == [0, 1]


def test_decimation_keeps_exactly_every_nth_point_and_the_same_values():
    """A stride of 4 over 10 points keeps ceil(10/4) = 3 points, and they are
    points 0, 4, 8 -- not a random 30% sample, so two runs of the same capture
    retain the same geometry."""
    store = NodeCloudStore(decimation=4)
    src = _pts(10)
    node = store.add(_pose(), src)
    assert node.point_count == 3
    assert np.array_equal(node.points, src[::4])
    assert store.point_count == 3


def test_decimation_of_one_and_zero_and_negative_all_keep_every_point():
    src = _pts(9)
    for stride in (1, 0, -3):
        store = NodeCloudStore(decimation=stride)
        assert store.add(_pose(), src).point_count == 9


# ---- the load-bearing property: sensor-frame points, re-posable ------------

def test_points_are_stored_in_the_sensor_frame_not_the_world_frame():
    """A node integrated at a pose 5 m along +x must still hold the RAW sensor
    points. Storing world points is the defect this store exists to avoid: it
    reads as a working store until the first pose correction, then re-poses
    nothing."""
    src = np.array([[0.0, 0.0, 1.0], [0.5, 0.0, 2.0]], dtype=np.float32)
    store = NodeCloudStore()
    node = store.add(_pose(tx=5.0), src)
    assert np.array_equal(node.points, src)
    # ...and the world view is the pose applied on demand.
    assert np.allclose(node.world_points(), src + np.array([5.0, 0.0, 0.0]), atol=1e-6)


def test_world_points_apply_rotation_and_translation_of_the_node_pose():
    src = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    store = NodeCloudStore()
    node = store.add(_pose(tx=2.0, tz=-1.0, yaw_deg=90.0), src)
    # +90 deg about Y sends +z -> +x, then translate.
    assert np.allclose(node.world_points(), [[3.0, 0.0, -1.0]], atol=1e-6)


def test_set_pose_moves_the_world_cloud_and_leaves_the_points_bit_identical():
    """The #150 acceptance property: a graph correction is ONE mat4 write per
    node -- no re-integration, no point rewrite."""
    src = _pts(6)
    store = NodeCloudStore()
    node = store.add(_pose(), src)
    before_points = node.points.copy()
    before_world = node.world_points().copy()

    node.set_pose(_pose(tx=0.25, ty=-0.5))

    assert np.array_equal(node.points, before_points)     # not one point rewritten
    delta = node.world_points() - before_world
    assert np.allclose(delta, np.array([0.25, -0.5, 0.0]), atol=1e-6)
    assert node.point_count == 6


def test_update_poses_re_poses_only_the_named_nodes():
    store = NodeCloudStore()
    n0 = store.add(_pose(), _pts(4))
    n1 = store.add(_pose(tx=1.0), _pts(4))
    n2 = store.add(_pose(tx=2.0), _pts(4))
    before1 = n1.world_points().copy()

    moved = store.update_poses({0: _pose(tx=-9.0), 2: _pose(tx=9.0)})

    assert moved == 2
    assert n0.pose[0, 3] == -9.0
    assert n2.pose[0, 3] == 9.0
    assert n1.pose[0, 3] == 1.0
    assert np.array_equal(n1.world_points(), before1)


def test_update_poses_rejects_an_unknown_node_id_and_changes_nothing():
    store = NodeCloudStore()
    n0 = store.add(_pose(), _pts(4))
    with pytest.raises(KeyError):
        store.update_poses({0: _pose(tx=1.0), 99: _pose(tx=2.0)})
    assert n0.pose[0, 3] == 0.0        # the valid half was not applied either


def test_a_rejected_pose_shape_never_lands_on_the_node():
    store = NodeCloudStore()
    node = store.add(_pose(), _pts(3))
    with pytest.raises(ValueError):
        node.set_pose(np.eye(3))
    assert node.pose.shape == (4, 4)


# ---- ownership of the arrays ----------------------------------------------

def test_the_store_copies_its_input_so_a_caller_reusing_a_buffer_cannot_corrupt_it():
    """`Mapper.step` hands over a slice of its per-frame deprojection buffer;
    if the store aliased it, every node would end up holding the LAST frame."""
    src = _pts(5)
    store = NodeCloudStore()
    node = store.add(_pose(), src)
    src[:] = -1.0
    assert node.points[0, 0] == 0.0
    assert float(node.points.max()) == 14.0

    pose = _pose()
    node2 = store.add(pose, _pts(2))
    pose[0, 3] = 77.0
    assert node2.pose[0, 3] == 0.0


def test_points_are_stored_as_float32_at_three_columns():
    node = NodeCloudStore().add(_pose(), np.zeros((8, 3), dtype=np.float64))
    assert node.points.dtype == np.float32
    assert node.points.shape == (8, 3)


def test_a_non_three_column_cloud_is_rejected():
    store = NodeCloudStore()
    with pytest.raises(ValueError):
        store.add(_pose(), np.zeros((5, 2), dtype=np.float32))
    assert len(store) == 0


# ---- the cost this store has to justify (spec kill criterion) --------------

def test_nbytes_is_the_exact_retained_array_cost():
    """12 bytes per retained point (3 x float32) + 128 bytes of pose (4x4
    float64). The spec's kill criterion is measured in MB, so this number has
    to be the real one rather than a proxy."""
    store = NodeCloudStore()
    store.add(_pose(), _pts(100))
    store.add(_pose(), _pts(50))
    assert store.nbytes == (100 + 50) * 12 + 2 * 128
    assert store.nbytes_mb == pytest.approx(((150 * 12) + 256) / (1024 * 1024))


def test_decimation_cuts_retained_bytes_by_the_stride():
    full = NodeCloudStore()
    thinned = NodeCloudStore(decimation=4)
    for _ in range(3):
        full.add(_pose(), _pts(100))
        thinned.add(_pose(), _pts(100))
    assert full.point_count == 300
    assert thinned.point_count == 75            # ceil(100/4) x 3
    assert thinned.nbytes == 75 * 12 + 3 * 128


def test_max_nodes_stops_retaining_and_counts_what_it_refused():
    """An unbounded store on a 10-minute scan is the spec's kill criterion. The
    cap refuses NEW nodes rather than evicting old ones -- evicting would punch
    a hole in the pose graph the store exists to serve."""
    store = NodeCloudStore(max_nodes=2)
    a = store.add(_pose(), _pts(4))
    b = store.add(_pose(), _pts(4))
    c = store.add(_pose(), _pts(4))
    assert (a.node_id, b.node_id) == (0, 1)
    assert c is None
    assert len(store) == 2
    assert store.dropped == 1
    assert store.point_count == 8


def test_clear_releases_every_node_and_resets_the_counters():
    store = NodeCloudStore()
    store.add(_pose(), _pts(4))
    store.clear()
    assert len(store) == 0
    assert store.point_count == 0
    assert store.nbytes == 0
    # ids keep counting up: a node id must never be reused by a later graph.
    assert store.add(_pose(), _pts(1)).node_id == 1


# ---- the whole-map view ----------------------------------------------------

def test_world_cloud_concatenates_every_node_in_id_order_at_its_current_pose():
    store = NodeCloudStore()
    p = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
    store.add(_pose(tx=0.0), p)
    store.add(_pose(tx=1.0), p)
    store.add(_pose(tx=2.0), p)
    cloud = store.world_cloud()
    assert cloud.shape == (3, 3)
    assert np.allclose(cloud[:, 0], [0.0, 1.0, 2.0], atol=1e-6)

    store.update_poses({1: _pose(tx=10.0)})
    assert np.allclose(store.world_cloud()[:, 0], [0.0, 10.0, 2.0], atol=1e-6)


def test_stats_reports_the_numbers_the_spec_asks_to_be_measured():
    store = NodeCloudStore(decimation=2)
    store.add(_pose(), _pts(100), frame_index=0)
    store.add(_pose(), _pts(100), frame_index=40)
    s = store.stats()
    assert s["nodes"] == 2
    assert s["points"] == 100
    assert s["decimation"] == 2
    assert s["dropped"] == 0
    assert s["bytes"] == 100 * 12 + 2 * 128
    assert s["points_per_node"] == 50.0


def test_map_node_is_constructible_on_its_own_for_an_offline_graph_pass():
    node = MapNode(node_id=7, pose=_pose(tz=1.0),
                   points=np.zeros((3, 3), dtype=np.float32), frame_index=12)
    assert node.node_id == 7
    assert node.point_count == 3
    assert np.allclose(node.world_points()[:, 2], [1.0, 1.0, 1.0], atol=1e-6)
