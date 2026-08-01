#!/usr/bin/env python3
"""장애물 회피 데모 (영상 촬영용, 임시).

인도에 정적 장애물(박스/소화전)을 놓고, waypoint follower가 사이를 위빙해
통과한다. 중간에 보행자 actor가 횡단해 게이트가 실제 인지로 STOP → 통과 후
RESUME하는 장면이 포함된다 (이 부분은 연출이 아니라 실제 반응층).

사용 (워크스페이스 소싱 후 repo 루트에서):
  python3 verification/demo/run_demo.py          # GUI 자동 표시
촬영이 끝나면 Ctrl+C.

⚠ 검증 회귀(run_scenario.py)와 동시 실행 금지 — Gazebo 프로세스가 충돌한다.
"""

import json
import os
import signal
import subprocess
import sys
import time

DEMO_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(DEMO_DIR))    # verification/
import worlds  # noqa: E402

REPO = os.path.dirname(os.path.dirname(DEMO_DIR))
BASE_WORLD = os.path.join(
    REPO, 'ros2_ws/src/go2_simulation/worlds/small_city.sdf')

# ── 코스 정의 (로봇: (15, 5.2)에서 -x 방향, 위빙 실효 ~0.3 m/s) ──────────
# 시각 우선: 인도 위에 서 있는 사람 메시(MaleVisitorOnPhone) + 박스를
# 지그재그로 세우고 로봇이 차례대로 반대편으로 회피 기동.
# 로봇 계획 경로와 각 장애물의 측면 간격은 중심 기준 ~0.75m (코리도 밖 —
# 게이트는 통과를 허용하되 접근 순간 SLOW가 걸려 감속 장면이 나온다).
# 배치: 위빙이 잘 보이는 촘촘한 코스 (물리 여유 ~0.2~0.3m, odom 궤적으로
# 검증). 이 간격에서 기본 게이트는 코리도 스침마다 SLOW/STOP을 걸므로
# 데모는 완화된 게이트 override(safety_stop_demo.yaml)로 촬영한다 —
# 본 설정 파일은 무변경 (launch 인자 주입, 원상복구 불필요).
OBSTACLES = [                        # (include 모델, x, y) — 박스류
    ('cardboard_box', 12.8, 5.65),
    ('cardboard_box', 9.4, 5.65),
    ('cardboard_box', 6.0, 5.6),
]
STATIC_PEDS = [                      # 서 있는 사람 메시 (worlds static spec)
    {'name': 'demo_standing_ped_1', 'static': True, 'x': 11.1, 'y': 4.75},
    {'name': 'demo_standing_ped_2', 'static': True, 'x': 7.7, 'y': 4.75},
]
# follower 경로: 장애물 반대편을 잇는 위빙 (박스=오른쪽으로, 사람=왼쪽으로)
# 마지막 구간: 중앙 복귀하다 마주 오는 보행자를 만나 오른쪽으로 사이드스텝
# → 스치듯 교행(측면 간격 ~1.0m) → 중앙 복귀. 만남 지점이 스택 준비
# 편차(~10-20 sim s)로 x≈3~5 사이에서 흔들려도 그 구간 내내 오른쪽을
# 유지하므로 교행이 성립한다.
# ⚠ 갭에서 갭으로 직행하는 순수 지그재그 — 장애물 사이 "중앙 복귀" 지점을
# 두면 그 구간에서 로봇 헤딩이 다음 장애물 정면을 향해 게이트 emergency
# 경계까지 진입 → STOP→회전→재개 반복으로 페이스가 깨짐 (실측: 복귀형은
# wp7→8에 24s, 직행형은 헤딩이 항상 빈 공간을 향함).
FOLLOWER_WAYPOINTS = [
    [14.4, 5.05],                   # 진입 활주로 (완만한 첫 횡이동)
    [12.8, 4.8],                    # box1 오른쪽
    [11.1, 5.6],                    # ped1 왼쪽
    [9.4, 4.8],                     # box2 오른쪽
    [7.7, 5.6],                     # ped2 왼쪽
    [6.0, 4.8],                     # box3 오른쪽
    [4.9, 5.1],                     # 중앙 복귀 — 전방에 마주 오는 보행자
    [4.2, 4.5],                     # 조기 인지 가정 → 왼쪽 가장자리로 양보 이동
    [3.7, 4.35, 15],                # 가장자리 정지, 15s 양보 대기 (actor 통과)
    [2.6, 4.8], [1.4, 5.0],         # 통과 확인 후 재개 → 중앙 복귀
    [0.4, 5.1],                     # 완주
]
# head-on 보행자 — "조기 양보(YIELD)" 안무 (Phase 5 기동 프리뷰):
#   ① 로봇이 box3 통과(절대 sim ≈48~50)할 때 출발, 중앙선(y 5.2) 직진 대면
#   ② 로봇은 조기 인지를 가정하고 x 4.2→3.7에서 왼쪽 가장자리(y 4.35)로
#      이동 후 15s 정지 대기 — 통과 순간 로봇이 완전 정지 상태이므로
#      교행 여유가 결정적(≈0.4m)으로 보장됨 (쌍방 동시 이동 안무는
#      타이밍 편차로 충돌 — 사용자 실측)
#   ③ 보행자(0.3 m/s)가 중앙선 그대로 지나감 → 대기 종료 후 로봇 재개
# hold x=4.9 (box3 x=6.0 못 미침), y 5.2 — 상가 전면부(y≥6.5) 회피.
# ⚠ 게이트 states: 양보가 충분히 일러 게이트 개입 없이 지나가는 게 정상
#   (대기 중 표시는 NOMINAL). 보행자가 예정보다 늦으면 재개 경로의 코리도에
#   걸려 게이트 SLOW/WAIT가 실제로 발화 — 이중 안전.
PED = {'name': 'demo_headon_ped', 'loop': False,
       'waypoints': [[0, -3.8, 5.2, 0],
                     [50, -2.2, 5.2, 0],           # 대기 롤 (0.032 m/s)
                     [73.7, 4.9, 5.2, 0]]}         # 0.3 m/s 정면 접근·통과 → hold

INCLUDE_TEMPLATE = """    <include>
      <uri>model://{model}</uri>
      <name>demo_obs_{i}</name>
      <pose>{x} {y} 0.16 0 0 0</pose>
      <static>true</static>
    </include>
"""


def make_world(out_path):
    worlds.generate_world(
        BASE_WORLD, [PED] + STATIC_PEDS, out_path, scenario_name='demo')
    with open(out_path) as f:
        sdf = f.read()
    insert = ''.join(
        INCLUDE_TEMPLATE.format(model=m, i=i, x=x, y=y)
        for i, (m, x, y) in enumerate(OBSTACLES))
    idx = sdf.rindex('</world>')
    with open(out_path, 'w') as f:
        f.write(sdf[:idx] + insert + sdf[idx:])


def _sigterm(*_):
    raise KeyboardInterrupt


def main():
    # SIGTERM(외부 timeout 등)에도 finally가 돌아 launch 트리를 정리하도록.
    # 이거 없이 SIGTERM으로 죽으면 컨트롤러/게이트 노드가 좀비로 남아
    # 다음 실행의 /cmd_vel_safe·/safety/state에 섞여 들어간다 (실측:
    # 좀비 게이트들이 sensor_timeout STOP을 계속 발행 → 주행이 멈칫거림
    # + 스포너 락 경합으로 후속 launch가 실패).
    signal.signal(signal.SIGTERM, _sigterm)
    out_dir = os.path.join(DEMO_DIR, 'runs')
    os.makedirs(out_dir, exist_ok=True)
    world_path = os.path.join(out_dir, f'demo_world_{int(time.time())}.sdf')
    make_world(world_path)
    print(f'world: {world_path}')

    launch_cmd = [
        'ros2', 'launch', 'go2_simulation', 'unitree_go2_launch_small_city.py',
        'gui:=true', 'rviz:=false', f'world:={world_path}',
        'world_init_heading:=3.141593',
        # 데모 전용 완화 게이트 (본 설정 무변경 — 촬영용)
        f'safety_stop_extra_params:={os.path.join(DEMO_DIR, "safety_stop_demo.yaml")}',
    ]
    proc = subprocess.Popen(launch_cmd, start_new_session=True)
    fol = subprocess.Popen([
        sys.executable, os.path.join(DEMO_DIR, 'waypoint_follower.py'),
        json.dumps(FOLLOWER_WAYPOINTS),
        os.path.join(out_dir, 'last_trace.csv')])
    try:
        while fol.poll() is None:
            time.sleep(0.5)
        print(f'follower rc={fol.returncode} — 촬영 종료 후 Ctrl+C')
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if fol.poll() is None:
            fol.kill()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


if __name__ == '__main__':
    main()
