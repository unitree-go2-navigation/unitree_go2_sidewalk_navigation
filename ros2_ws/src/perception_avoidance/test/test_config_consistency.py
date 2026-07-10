"""설정 파일 간 정합 고정.

lidar_obstacle_node의 corridor_half(코리도 내 최근접점 계산 밴드)는
safety_stop_node의 robot_half_width + corridor_margin(게이트 코리도)과
같은 값이어야 한다 — 서로 다른 yaml에 살기 때문에 드리프트를 테스트로 막는다.
"""

from pathlib import Path

import yaml

CONFIG = Path(__file__).parents[1] / 'config'


def test_corridor_half_matches_gate_corridor():
    lidar = yaml.safe_load((CONFIG / 'self_filter.yaml').read_text())
    gate = yaml.safe_load((CONFIG / 'safety_stop.yaml').read_text())
    lidar_p = lidar['lidar_obstacle_node']['ros__parameters']
    gate_p = gate['safety_stop_node']['ros__parameters']
    expected = gate_p['robot_half_width'] + gate_p['corridor_margin']
    assert abs(lidar_p['corridor_half'] - expected) < 1e-9, (
        f"corridor_half({lidar_p['corridor_half']}) != "
        f"robot_half_width+corridor_margin({expected}) — 두 yaml을 함께 수정할 것")
