"""verification/worlds.py 월드 생성 가드 테스트.

gz <actor>는 headless 센서 렌더링에서 loop=false면 sim을 wedge시킴
(2026-07-09~10) — 이동 보행자 actor가 안전 레시피(loop=true + 준정지 hold +
tension=1.0 + 비매칭 trajectory type)로 생성되고, 퇴화 궤적(초저속/제자리
세그먼트)이 생성 단계에서 거부되는지 고정한다.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[4] / 'verification'))
import worlds  # noqa: E402


def test_degenerate_hold_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 10.0, 5.2, 0], [9999, 10.001, 5.2, 0]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.moving_ped_xml(spec)


def test_pause_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [
        [0, 12.0, 2.0, 1.57], [6, 12.0, 5.2, 1.57], [16, 12.0, 5.2, 1.57]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.moving_ped_xml(spec)


def test_normal_trajectory_accepted():
    spec = {'name': 'ped', 'loop': False,
            'waypoints': [[0, 4.0, 5.2, 0], [12, 10.0, 5.2, 0]]}
    xml = worlds.moving_ped_xml(spec)
    assert '<actor name="ped">' in xml
    # headless 안전 레시피 4종 고정
    assert '<loop>true</loop>' in xml           # loop=false는 wedge
    assert 'interpolate_x' not in xml           # 있으면 시간 무시(거리 재타이밍)
    assert 'tension="1.0"' in xml               # 낮으면 스플라인 오버슈트
    assert 'type="route"' in xml                # animation 이름과 매칭 금지
    # spec loop=false → 준정지 hold 세그먼트 자동 추가 (2 + 1)
    assert xml.count('<waypoint>') == 3
    assert '<time>10011.0</time>' in xml        # 12 + HOLD_DURATION


def test_single_waypoint_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 4.0, 5.2, 0]]}
    with pytest.raises(ValueError, match='static'):
        worlds.moving_ped_xml(spec)


def test_static_ped_is_model_not_actor():
    xml = worlds.static_ped_xml({'name': 'ped_s', 'x': 10.0, 'y': 5.2})
    assert '<model name="ped_s">' in xml
    assert '<static>true</static>' in xml
    assert '<actor' not in xml
