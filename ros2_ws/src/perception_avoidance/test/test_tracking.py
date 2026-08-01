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


def test_fast_track_predictive_association_keeps_id(node):
    # 자전거급(6m/s, 5Hz 열화 가정): 프레임당 1.2m 접근. 부트스트랩은 게이트
    # (1.6) 이내, 이후는 CVM 예측 위치 매칭으로 id 유지 → EMA가 실속도로 수렴.
    # (구: 마지막 위치 NN + 게이트 1.0 → 매 프레임 id churn, vx≈0 반복 — P4 실측)
    r1 = node._associate([(10.0, 0.0, 0.5)], stamp_s=10.0)
    r2 = node._associate([(8.8, 0.0, 0.5)], stamp_s=10.2)
    r3 = node._associate([(7.6, 0.0, 0.5)], stamp_s=10.4)
    assert r1[0][0] == r2[0][0] == r3[0][0]
    assert r3[0][1] == pytest.approx(-4.5, abs=1e-6)   # raw -6, EMA 0→-3→-4.5


def test_greedy_keeps_static_clutter_and_fast_track_separate(node):
    # 고속 track이 정적 클러터 옆(게이트 거리 내)을 지나도 정적 클러터는
    # 자기 자신(오차 ~0)이 greedy 우선 → 교차 연관 없음
    r1 = node._associate([(5.0, 1.0, 0.5), (6.5, 0.0, 0.5)], stamp_s=10.0)
    r2 = node._associate([(5.0, 1.0, 0.5), (5.3, 0.0, 0.5)], stamp_s=10.2)
    assert r2[0][0] == r1[0][0]
    assert r2[1][0] == r1[1][0]
    assert r2[1][1] == pytest.approx(-3.0, abs=1e-6)   # raw -6, EMA → -3


def test_track_coasts_through_dropout_frames(node):
    # 열화(real_l1) dropout: 한 프레임 소실에도 track id·속도가 살아남아야
    # fast 트리거 지속 카운트가 이어진다 (미발행, 재연관용 유지만).
    r1 = node._associate([(10.0, 0.0, 0.5)], stamp_s=10.0)
    node._associate([(9.5, 0.0, 0.5)], stamp_s=10.1)      # raw -5 → EMA -2.5
    node._associate([], stamp_s=10.2)                     # dropout — coast
    r4 = node._associate([(8.5, 0.0, 0.5)], stamp_s=10.3)
    assert r4[0][0] == r1[0][0]                           # id 유지
    # 마지막 실측(9.5@10.1) 기준 gap 전체로 raw 산출: (8.5-9.5)/0.2=-5 → EMA -3.75
    assert r4[0][1] == pytest.approx(-3.75, abs=1e-6)


def test_track_expires_after_coast_limit(node):
    r1 = node._associate([(10.0, 0.0, 0.5)], stamp_s=10.0)
    for k in range(4):                                    # 미스 4 > 한도 3
        node._associate([], stamp_s=10.1 + 0.1 * k)
    r = node._associate([(10.0, 0.0, 0.5)], stamp_s=10.6)
    assert r[0][0] != r1[0][0]                            # 만료 → 새 track
    assert r[0][1] == 0.0
