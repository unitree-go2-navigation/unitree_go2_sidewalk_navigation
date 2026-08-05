"""회귀 하네스의 시행 간 정리 목록 ↔ launch 파일 정합 (Phase 3.5).

launch 가 띄우는 노드 중 하나라도 LEFTOVER_PATTERNS 에서 빠지면, 그 노드가
살아남아 다음 trial 과 같은 ROS 그래프에서 계속 발행한다. TF/odom 발행 노드가
빠진 경우가 특히 나쁘다 — 다음 trial 의 TF 트리가 오염돼 /obstacles/lidar 가
나오지 않고, 한 번 발생하면 이후 모든 trial 이 연쇄 실패한다 (2026-08-04 실측:
ekf_node·state_estimation_node 누락으로 bike_pass·crossing 10연속 FAIL).

launch 에 노드를 추가하면 이 테스트가 먼저 깨진다.
"""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LAUNCH = (REPO / 'ros2_ws/src/go2_simulation/launch'
          / 'unitree_go2_launch_small_city.py')
VERIFICATION = REPO / 'verification'

# 정리 대상이 아닌 executable — 이유를 명시해야 목록이 썩지 않는다.
EXEMPT = {
    'rviz2',      # 회귀는 rviz:=false. 사용자가 띄운 rviz 를 죽이지 않는다.
}

# executable= 이름 → LEFTOVER_PATTERNS 에서 쓰는 경로 포함 패턴.
# 'create'/'spawner' 처럼 흔한 이름은 반드시 경로째로 매칭해야 무관한
# 프로세스를 죽이지 않는다.
PATH_QUALIFIED = {
    'create': 'ros_gz_sim/create',
    'spawner': 'controller_manager/spawner',
}


def load_patterns():
    sys.path.insert(0, str(VERIFICATION))
    try:
        import run_scenario
        return list(run_scenario.LEFTOVER_PATTERNS)
    finally:
        sys.path.remove(str(VERIFICATION))


def launch_executables():
    text = LAUNCH.read_text()
    return {m.group(1) for m in
            re.finditer(r'executable=["\']([A-Za-z0-9_]+)["\']', text)}


def test_cleanup_covers_every_launch_node():
    patterns = load_patterns()
    missing = []
    for exe in sorted(launch_executables() - EXEMPT):
        want = PATH_QUALIFIED.get(exe, exe)
        if not any(want in p for p in patterns):
            missing.append(f'{exe} (기대 패턴: {want!r})')
    assert not missing, (
        'launch 노드가 LEFTOVER_PATTERNS 에서 누락 — 시행 간 오염 위험:\n  '
        + '\n  '.join(missing))


def test_driver_not_self_killed():
    # 드라이버를 목록에 넣으면 실행 중 자기 자신을 죽인다.
    patterns = load_patterns()
    assert not any('cmd_publisher' in p for p in patterns)


def test_common_names_are_path_qualified():
    # 'create'/'spawner' 를 맨 이름으로 두면 무관한 프로세스를 죽인다.
    for p in load_patterns():
        assert p not in ('create', 'spawner'), f'경로 미포함 패턴: {p!r}'
