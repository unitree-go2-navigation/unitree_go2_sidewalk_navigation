"""YAML 설정값 ↔ 노드 선언 기본값 정합 테스트.

YAML 미로드(오타, launch 누락) 시 코드 기본값으로 조용히 동작하는
footgun을 막는다: 두 값이 갈라지면 여기서 실패한다.
"""

from pathlib import Path

import pytest
import yaml

from perception_avoidance.lidar_obstacle_node import LidarObstacleNode
from perception_avoidance.safety_stop_node import SafetyStopNode

CONFIG_DIR = Path(__file__).parents[1] / 'config'


def yaml_params(filename, node_name):
    with open(CONFIG_DIR / filename) as f:
        data = yaml.safe_load(f)
    return data[node_name]['ros__parameters']


def check_parity(node, params):
    mismatches = []
    for key, yaml_val in params.items():
        default_val = node.get_parameter(key).value
        if isinstance(yaml_val, float) or isinstance(default_val, float):
            ok = float(default_val) == pytest.approx(float(yaml_val))
        else:
            ok = default_val == yaml_val
        if not ok:
            mismatches.append(f'{key}: yaml={yaml_val!r} default={default_val!r}')
    assert not mismatches, 'YAML↔default drift:\n' + '\n'.join(mismatches)


def test_safety_stop_yaml_matches_declared_defaults():
    node = SafetyStopNode()
    try:
        check_parity(node, yaml_params('safety_stop.yaml', 'safety_stop_node'))
    finally:
        node.destroy_node()


def test_self_filter_yaml_matches_declared_defaults():
    node = LidarObstacleNode()
    try:
        check_parity(node, yaml_params('self_filter.yaml', 'lidar_obstacle_node'))
    finally:
        node.destroy_node()
