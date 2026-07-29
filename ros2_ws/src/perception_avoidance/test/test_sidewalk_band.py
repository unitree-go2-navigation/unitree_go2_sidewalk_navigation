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
    y0, y1 = find_band_edges(bin_y, g, w, ref_h=0.0)
    # 경계 = 첫 낙차 bin 중심 (bin 폭 0.2 → 연석 실위치 ±1 bin 허용)
    assert -1.75 < y0 < -1.3 and 1.3 < y1 < 1.75


def test_wall_north_curb_south():
    # 남측(-y) 연석, 북측(+y) 벽 (y=2.0부터 수직 점 다수)
    bin_y, g, w = make_bins()
    g[bin_y < -1.2] = -0.16
    w[bin_y > 2.0] = 20
    y0, y1 = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -1.4 < y0 < -1.0 and 1.9 < y1 < 2.3


def test_occlusion_gap_stops_at_last_seen():
    # +y 방향 1.0부터 표본 없음 (폐색) → 마지막 유효 bin에서 보수적 정지
    bin_y, g, w = make_bins()
    g[bin_y > 1.0] = np.nan
    g[bin_y < -2.0] = -0.16
    y0, y1 = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -2.2 < y0 < -1.8 and 0.7 < y1 < 1.3


def test_open_range_uses_limit():
    # 경계 없음 (평지) → 탐색 한계까지 개방
    bin_y, g, w = make_bins()
    y0, y1 = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert y0 < -7.5 and y1 > 7.5


def test_nan_ref_area_tolerated():
    # 중앙 부근 일부 NaN이어도 (로봇 그림자) 경계 탐색은 진행
    bin_y, g, w = make_bins()
    g[np.abs(bin_y) < 0.3] = np.nan
    g[bin_y < -3.0] = -0.2
    g[bin_y > 3.0] = -0.2
    y0, y1 = find_band_edges(bin_y, g, w, ref_h=0.0)
    assert -3.3 < y0 < -2.7 and 2.7 < y1 < 3.3
