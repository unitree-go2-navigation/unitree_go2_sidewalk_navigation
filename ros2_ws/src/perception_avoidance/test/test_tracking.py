"""lidar_obstacle_node의 frame-to-frame 추적(_associate) 테스트."""

import pytest

from perception_avoidance.lidar_obstacle_node import LidarObstacleNode


@pytest.fixture()
def node():
    n = LidarObstacleNode()
    yield n
    n.destroy_node()


def test_first_frame_new_tracks_zero_velocity(node):
    results = node._associate([(1.0, 0.0, 0.5), (3.0, 1.0, 0.5)], stamp_s=10.0)
    assert len(results) == 2
    ids = {tid for tid, _, _ in results}
    assert len(ids) == 2                      # distinct track ids
    assert all(vx == 0.0 and vy == 0.0 for _, vx, vy in results)


def test_matched_track_velocity_ema(node):
    node._associate([(1.0, 0.0, 0.5)], stamp_s=10.0)
    # moved +0.1m in x over dt=0.1 → raw v=1.0, EMA(0.5) from 0 → 0.5
    results = node._associate([(1.1, 0.0, 0.5)], stamp_s=10.1)
    tid, vx, vy = results[0]
    assert vx == pytest.approx(0.5, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)


def test_track_id_persists_across_frames(node):
    r1 = node._associate([(1.0, 0.0, 0.5)], stamp_s=10.0)
    r2 = node._associate([(1.1, 0.0, 0.5)], stamp_s=10.1)
    assert r1[0][0] == r2[0][0]


def test_beyond_assoc_max_dist_new_track(node):
    r1 = node._associate([(1.0, 0.0, 0.5)], stamp_s=10.0)
    # jumped 2m > assoc_max_dist(1.0) → new id, velocity reset
    r2 = node._associate([(3.0, 0.0, 0.5)], stamp_s=10.1)
    assert r1[0][0] != r2[0][0]
    assert r2[0][1] == 0.0


def test_out_of_range_dt_keeps_previous_velocity(node):
    node._associate([(1.0, 0.0, 0.5)], stamp_s=10.0)
    node._associate([(1.1, 0.0, 0.5)], stamp_s=10.1)      # v → 0.5
    # dt=1.0 > vel_max_dt(0.5): velocity must not be recomputed from the jump
    results = node._associate([(1.2, 0.0, 0.5)], stamp_s=11.1)
    assert results[0][1] == pytest.approx(0.5, abs=1e-6)
