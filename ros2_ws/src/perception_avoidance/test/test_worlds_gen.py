"""verification/worlds.py 월드 생성 가드 테스트.

퇴화 actor 궤적(초저속/제자리 세그먼트)은 센서 렌더링 활성 시 gz sim을
wedge시키는 것이 확인됨(2026-07-09) — 생성 단계에서 거부되는지 고정한다.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[4] / 'verification'))
import worlds  # noqa: E402


def test_degenerate_hold_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 10.0, 5.2, 0], [9999, 10.001, 5.2, 0]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.actor_xml(spec)


def test_pause_segment_rejected():
    spec = {'name': 'ped', 'waypoints': [
        [0, 12.0, 2.0, 1.57], [6, 12.0, 5.2, 1.57], [16, 12.0, 5.2, 1.57]]}
    with pytest.raises(ValueError, match='degenerate'):
        worlds.actor_xml(spec)


def test_normal_trajectory_accepted():
    spec = {'name': 'ped', 'loop': False,
            'waypoints': [[0, 4.0, 5.2, 0], [12, 10.0, 5.2, 0]]}
    xml = worlds.actor_xml(spec)
    assert '<actor name="ped">' in xml
    assert xml.count('<waypoint>') == 2
    assert '<loop>false</loop>' in xml


def test_single_waypoint_rejected():
    spec = {'name': 'ped', 'waypoints': [[0, 4.0, 5.2, 0]]}
    with pytest.raises(ValueError, match='static'):
        worlds.actor_xml(spec)


def test_static_ped_is_model_not_actor():
    xml = worlds.static_ped_xml({'name': 'ped_s', 'x': 10.0, 'y': 5.2})
    assert '<model name="ped_s">' in xml
    assert '<static>true</static>' in xml
    assert '<actor' not in xml
