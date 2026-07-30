"""find_band_edges (라이다 밴드 추정 순수 함수) 단위 테스트."""

import math

import numpy as np

from perception_avoidance.sidewalk_polygon_node import find_band_edges


def make_bins(lat=8.0, size=0.2):
    nb = int(2 * lat / size)
    bin_y = -lat + (np.arange(nb) + 0.5) * size
    ground = np.zeros(nb)
    wall = np.zeros(nb, dtype=int)
    return bin_y, ground, wall


def test_curb_both_sides():
    # 보도 y[-1.5, 1.5], 밖은 0.16 낮은 도로
    bin_y, g, w = make_bins()
    g[(bin_y < -1.5) | (bin_y > 1.5)] = -0.16
    y0, y1, *_ = find_band_edges(bin_y, g, w, ref_h=0.0)
    # 경계 = 첫 낙차 bin 중심 (bin 폭 0.2 → 연석 실위치 ±1 bin 허용)
    assert -1.75 < y0 < -1.3 and 1.3 < y1 < 1.75


def test_wall_north_curb_south():
    # 남측(-y) 연석, 북측(+y) 벽 (y=2.0부터 수직 점 다수)
    bin_y, g, w = make_bins()
    g[bin_y < -1.2] = -0.16
    w[bin_y > 2.0] = 20
    y0, y1, *_ = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -1.4 < y0 < -1.0 and 1.9 < y1 < 2.3


def test_occlusion_gap_stops_at_last_seen():
    # +y 방향 1.0부터 표본 없음 (폐색) → 마지막 유효 bin에서 보수적 정지
    bin_y, g, w = make_bins()
    g[bin_y > 1.0] = np.nan
    g[bin_y < -2.0] = -0.16
    y0, y1, *_ = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -2.2 < y0 < -1.8 and 0.7 < y1 < 1.3


def test_open_range_uses_limit():
    # 경계 없음 (평지) → 탐색 한계까지 개방
    bin_y, g, w = make_bins()
    y0, y1, *_ = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert y0 < -7.5 and y1 > 7.5


def test_nan_ref_area_tolerated():
    # 중앙 부근 일부 NaN이어도 (로봇 그림자) 경계 탐색은 진행
    bin_y, g, w = make_bins()
    g[np.abs(bin_y) < 0.3] = np.nan
    g[bin_y < -3.0] = -0.2
    g[bin_y > 3.0] = -0.2
    y0, y1, *_ = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -3.3 < y0 < -2.7 and 2.7 < y1 < 3.3


# --- 평면 보정 + 경계 추적기 (요동 억제, 2026-07-29) ---

from perception_avoidance.sidewalk_polygon_node import (EdgeTracker,
                                                        fit_ground_plane)


def test_plane_fit_recovers_tilt():
    # 1도 피치 평면 → 계수 복원 (8m에서 0.14m 오차 원인 제거 확인)
    rng = np.random.default_rng(7)
    xy = rng.uniform(-1.2, 1.2, (200, 2))
    a = math.tan(math.radians(1.0))
    z = a * xy[:, 0] + 0.0 * xy[:, 1] - 0.3 + rng.normal(0, 0.01, 200)
    pts = np.column_stack([xy[:, 0], xy[:, 1], z])
    coef = fit_ground_plane(pts)
    assert coef is not None and abs(coef[0] - a) < 0.005


def test_tracker_absorbs_noise():
    t = EdgeTracker(2.0)
    for v in [2.1, 1.9, 2.05, 1.95, 2.1]:
        t.update(v)
    assert 1.9 < t.v < 2.1


def test_tracker_ignores_single_outlier():
    t = EdgeTracker(2.0)
    t.update(5.0)      # 원거리 깜빡임 스파이크 1프레임
    t.update(2.0)
    assert abs(t.v - 2.0) < 0.05


def test_tracker_follows_persistent_change_rate_limited():
    t = EdgeTracker(2.0)
    for _ in range(3):
        t.update(4.0)  # 3프레임 연속 합의 → 이동 시작
    assert 2.2 < t.v <= 2.4      # rate 0.3/프레임 제한
    for _ in range(10):
        t.update(4.0)
    assert t.v > 3.5             # 지속 신호는 결국 수렴


# --- 연석 방향 직선 적합 (2026-07-30 요잉 대각 문제) ---

from perception_avoidance.sidewalk_polygon_node import fit_band_lines


def test_band_lines_recover_tilt():
    # 로봇이 보도 축 대비 ~11°(기울기 0.2) 요잉한 상황의 구간별 경계
    sx = [1.4, 3.3, 5.2, 7.1]
    lo = [0.2 * x - 1.5 for x in sx]
    hi = [0.2 * x + 1.5 for x in sx]
    a, b_lo, b_hi = fit_band_lines(sx, lo, hi)
    assert abs(a - 0.2) < 0.02
    assert abs(b_lo + 1.5) < 0.05 and abs(b_hi - 1.5) < 0.05


def test_band_lines_nan_slices_tolerated():
    sx = [1.4, 3.3, 5.2, 7.1]
    lo = [-1.5, math.nan, -1.5, math.nan]
    hi = [math.nan, 1.5, math.nan, 1.5]
    a, b_lo, b_hi = fit_band_lines(sx, lo, hi)
    assert abs(a) < 0.05 and abs(b_lo + 1.5) < 0.1 and abs(b_hi - 1.5) < 0.1


def test_band_lines_single_slice_fallback():
    sx = [4.0]
    a, b_lo, b_hi = fit_band_lines(sx, [-1.3], [1.3])
    assert a == 0.0 and b_lo == -1.3 and b_hi == 1.3


def test_band_lines_slope_clamped():
    sx = [1.0, 7.0]
    lo = [0.0, 6.0]     # 기울기 1.0 (비상식) → 0.6 클램프
    hi = [3.0, 9.0]
    a, _, _ = fit_band_lines(sx, lo, hi)
    assert abs(a) <= 0.6


# --- odom 프레임 밴드 모델 v3 (2026-07-30) ---

from perception_avoidance.sidewalk_polygon_node import (band_to_base,
                                                        band_to_odom)


def test_band_odom_roundtrip():
    # base 추정 → odom → base 역변환이 원래 직선과 일치 (요 0.3에서)
    rx, ry, ryaw = 10.0, 5.0, 0.3
    ux, uy, c_lo, c_hi = band_to_odom(0.0, -1.5, 1.5, rx, ry, ryaw)
    pts = band_to_base(ux, uy, c_lo, c_hi, rx, ry, ryaw, 0.0, 4.0)
    # 같은 x 꼭짓점 쌍의 y가 ±1.5 (base에서 밴드 폭 복원)
    assert abs(pts[0][1] + 1.5) < 1e-6 and abs(pts[3][1] - 1.5) < 1e-6


def test_band_model_is_yaw_invariant():
    # 모델 고정 + 로봇 요만 변화 → odom상 꼭짓점(월드 위치)은 불변
    rx, ry = 10.0, 5.0
    ux, uy, c_lo, c_hi = band_to_odom(0.0, -1.5, 1.5, rx, ry, 0.0)

    def world_corners(ryaw):
        pts = band_to_base(ux, uy, c_lo, c_hi, rx, ry, ryaw, 0.0, 4.0)
        out = []
        for bx, by in pts:
            wx = rx + math.cos(ryaw) * bx - math.sin(ryaw) * by
            wy = ry + math.sin(ryaw) * bx + math.cos(ryaw) * by
            out.append((round(wx, 6), round(wy, 6)))
        return out
    assert world_corners(0.0) == world_corners(0.5)   # 요잉해도 월드 고정


def test_band_direction_sign_flip_safe():
    # 반대 방향 추정(무방향 직선)도 같은 밴드를 표현
    rx, ry = 0.0, 0.0
    u1 = band_to_odom(0.0, -1.0, 2.0, rx, ry, 0.0)
    u2 = band_to_odom(0.0, -2.0, 1.0, rx, ry, math.pi)
    w1 = sorted([min(u1[2], u1[3]), max(u1[2], u1[3])])
    w2 = sorted([min(u2[2], u2[3]), max(u2[2], u2[3])])
    assert abs((w1[1]-w1[0]) - (w2[1]-w2[0])) < 1e-6   # 폭 동일
