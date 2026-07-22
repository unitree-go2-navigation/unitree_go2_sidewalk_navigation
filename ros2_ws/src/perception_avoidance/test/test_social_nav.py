"""Phase 5a gap 판단 순수 함수 검증 (계획 v2 게이트: 'gap 판단 pytest')."""

import pytest

from perception_avoidance.social_nav import (
    Blocker, Gap, GapRules, blockers_from_detections, choose_gap,
    classify_gap, classify_kind, compute_gaps, lateral_band, merge_blockers)

R = GapRules()


# --- compute_gaps / merge_blockers -----------------------------------------

def test_empty_band_single_edge_gap():
    gaps = compute_gaps([], -1.5, 1.5)
    assert gaps == [Gap(-1.5, 1.5, 'edge', 'edge')]


def test_single_blocker_splits_band():
    gaps = compute_gaps([Blocker(-0.2, 0.3, 'person')], -1.5, 1.5)
    assert gaps == [Gap(-1.5, -0.2, 'edge', 'person'),
                    Gap(0.3, 1.5, 'person', 'edge')]


def test_blocker_clipped_to_band():
    gaps = compute_gaps([Blocker(1.0, 5.0, 'static')], -1.5, 1.5)
    assert gaps == [Gap(-1.5, 1.0, 'edge', 'static')]


def test_merge_keeps_higher_priority_kind():
    merged = merge_blockers([Blocker(0.0, 0.5, 'static'),
                             Blocker(0.4, 0.9, 'person')])
    assert merged == [Blocker(0.0, 0.9, 'person')]


# --- classify_gap: 폭 임계 (0.7 / 1.0 / 1.2 규칙) ---------------------------

def test_static_squeeze_passable_with_slow_cap():
    ok, target, cap = classify_gap(Gap(0.0, 0.8, 'static', 'static'), R)
    assert ok and cap == pytest.approx(0.3)
    assert target == pytest.approx(0.4)  # 중앙


def test_static_below_attempt_min_blocked():
    # attempt_min 0.8 (0.7→0.8: 코리도 게이트와의 채터링 회피 — GapRules 주석)
    assert not classify_gap(Gap(0.0, 0.75, 'static', 'static'), R)[0]
    assert not classify_gap(Gap(0.0, 0.6, 'static', 'static'), R)[0]


def test_two_standing_persons_gap_1m_passes_centered():
    # 게이트 시나리오: 정지 보행자 2명 gap 1.0m 통과, 각 clearance ≥ 0.2
    ok, target, cap = classify_gap(Gap(0.0, 1.0, 'person', 'person'), R)
    assert ok and cap == pytest.approx(0.3)
    assert target == pytest.approx(0.5)
    clearance = target - 0.0 - R.robot_width / 2.0
    assert clearance >= 0.2


def test_standing_persons_below_1m_blocked():
    ok, _, _ = classify_gap(Gap(0.0, 0.9, 'person', 'person'), R)
    assert not ok


def test_moving_person_needs_1p2m():
    assert not classify_gap(Gap(0.0, 1.1, 'person_moving', 'static'), R)[0]
    ok, _, cap = classify_gap(Gap(0.0, 1.3, 'person_moving', 'static'), R)
    assert ok and cap == pytest.approx(0.3)


# --- classify_gap: 목표 배치 (정적 밀착 / 사람 여유 / 우측 편향) ------------

def test_person_static_gap_person_margin_floor():
    # 좁은 사람|정적 gap: 정적 밀착(0.25) 한도와 사람 하드 여유(0.45) 중
    # 사람 여유가 우선 — static_margin 0.25 상향 후 w=1.0에선 0.605 지점
    ok, target, _ = classify_gap(Gap(0.0, 1.0, 'person', 'static'), R)
    assert ok
    assert target == pytest.approx(R.person_margin + R.robot_width / 2)


def test_person_edge_gap_prioritizes_person_margin():
    # edge 밀착이 원칙이나 (edge_margin 0.30), 사람 여유 0.45가 우선이라
    # w=1.0에서는 사람 여유 확보 지점(0.395)까지만 접근
    ok, target, _ = classify_gap(Gap(0.0, 1.0, 'edge', 'person'), R)
    assert ok
    assert target == pytest.approx(1.0 - R.person_margin - R.robot_width / 2)


def test_open_band_no_bias():
    # 빈 밴드는 무편향 (keep-right 사행 방지 — 우측 관습은 측 선택 동률에만)
    ok, target, cap = classify_gap(Gap(-1.5, 1.5, 'edge', 'edge'), R)
    assert ok and cap is None
    assert target == pytest.approx(0.0)


def test_wide_person_gap_minimal_deviation():
    # 넓은 gap에서 반대편 최대 밀착 금지 — comfort(0.6) 여유 지점까지만
    # (프로브 실측: 필요 이탈 0.4m에 0.9m 우회)
    ok, target, cap = classify_gap(Gap(-1.15, 0.35, 'edge', 'person'), R)
    assert ok and cap == pytest.approx(0.3)
    assert target == pytest.approx(0.35 - R.person_comfort - R.robot_width / 2)


def test_narrow_static_gap_centered_not_right_biased():
    ok, target, _ = classify_gap(Gap(-0.75, 0.75, 'static', 'static'), R)
    assert ok and target == pytest.approx(0.0)


# --- choose_gap: 우측 통과 관습 + YIELD 후보 --------------------------------

def test_choose_prefers_rightmost_passable():
    gaps = compute_gaps([Blocker(-0.2, 0.3, 'static')], -1.5, 1.5)
    chosen = choose_gap(gaps, R)
    assert chosen is not None
    gap, target, _ = chosen
    assert gap.y_max == -0.2  # 우측(-y) gap
    assert target < 0


def test_choose_skips_blocked_right_gap():
    # 우측 gap 0.6(불가) / 좌측 gap 1.2(가능) → 좌측 선택
    gaps = compute_gaps([Blocker(-0.9, 0.3, 'static')], -1.5, 1.5)
    chosen = choose_gap(gaps, R)
    assert chosen is not None and chosen[0].y_min == 0.3


def test_no_passable_gap_returns_none():
    # 이동 보행자 사이 0.5m — 게이트 시나리오 'gap 0.5m → YIELD'의 판단부
    gaps = compute_gaps([Blocker(-1.5, -0.3, 'person_moving'),
                         Blocker(0.2, 1.5, 'person_moving')], -1.5, 1.5)
    assert choose_gap(gaps, R) is None


# --- lateral_band: polygon → 전방 y 밴드 ------------------------------------

def test_band_axis_aligned_rect():
    poly = [(-26.3, -1.5), (26.3, -1.5), (26.3, 1.5), (-26.3, 1.5)]
    assert lateral_band(poly, 0.0, 5.0) == pytest.approx((-1.5, 1.5))


def test_band_no_overlap_returns_none():
    poly = [(-10.0, -1.5), (-5.0, -1.5), (-5.0, 1.5), (-10.0, 1.5)]
    assert lateral_band(poly, 0.0, 5.0) is None


def test_band_rotated_polygon_narrows():
    # 45° 기운 폭 √2 밴드: x∈[0,1] 스트립과의 교차 y범위는 해석적으로
    # [-1, 2] (모서리 (0,±1), (1, 0/2))
    poly = [(0.0, -1.0), (2.0, 1.0), (1.0, 2.0), (-1.0, 0.0)]
    y = lateral_band(poly, 0.0, 1.0)
    assert y == pytest.approx((-1.0, 2.0))


def test_band_partial_overlap_clipped():
    poly = [(3.0, -1.0), (10.0, -1.0), (10.0, 1.0), (3.0, 1.0)]
    assert lateral_band(poly, 0.0, 5.0) == pytest.approx((-1.0, 1.0))


# --- classify_kind ----------------------------------------------------------

def test_moving_track_is_person_moving():
    # 이동 + 키 큼 → person_moving. 낮은 이동 track(부트스트랩 스파이크)은
    # person_moving 금지 (YIELD 오발 방어) — ever 이력으로 person까지만.
    assert classify_kind(True, True, True, True) == 'person_moving'
    assert classify_kind(True, True, False, True) == 'person'
    assert classify_kind(True, False, False, True) == 'static'


def test_stopped_walker_stays_person():
    assert classify_kind(False, True, False, True) == 'person'


def test_tall_compact_static_is_person():
    assert classify_kind(False, False, True, True) == 'person'


def test_low_box_is_static():
    assert classify_kind(False, False, False, True) == 'static'


def test_tall_extended_structure_is_static():
    # 울타리/벽 파편: 키가 커도 컴팩트가 아니면 사물
    assert classify_kind(False, False, True, False) == 'static'


def test_extended_structure_never_person_moving():
    # 가림 경계 슬라이딩 유사 이동체(상대속도≈0 울타리): 이동 판정보다
    # 컴팩트 검사가 우선 — moving 킬스위치 영구 래치 방지 (프로브 실측)
    assert classify_kind(True, True, True, False) == 'static'


# --- blockers_from_detections -----------------------------------------------

def test_blockers_window_filter_and_inflate():
    dets = [(2.0, 0.5, 0.4, 0.4, 'person'),    # 창 안
            (8.0, 0.0, 0.4, 0.4, 'static'),    # 창 밖 (전방 초과)
            (-2.0, 0.0, 0.4, 0.4, 'static')]   # 창 밖 (후방)
    out = blockers_from_detections(dets, -0.5, 5.0, 0.1)
    assert out == [Blocker(0.5 - 0.2 - 0.1, 0.5 + 0.2 + 0.1, 'person')]


def test_blocker_straddling_window_edge_included():
    dets = [(5.2, 0.0, 0.6, 0.4, 'static')]    # xmin 4.9 < x_max 5.0
    assert len(blockers_from_detections(dets, -0.5, 5.0, 0.1)) == 1
