#!/usr/bin/env python3
"""자동화 시나리오 러너 (검증 공통 원칙의 집행자).

trial마다 fresh Gazebo(headless)를 띄우고, 드라이버가 cmd 프로파일을 재생하고,
oracle CSV + 상태 로그를 수집해 임계와 비교, PASS/FAIL을 집계 CSV에 남긴다.

사용 (워크스페이스 소싱 후 repo 루트에서):
  python3 verification/run_scenario.py --all -n 5 --out verification/phase3_regression.csv
  python3 verification/run_scenario.py -s verification/scenarios/static_stop.yaml -n 1
  python3 verification/run_scenario.py -s verification/scenarios/static_stop.yaml \
      --degrade verification/profiles/real_l1.yaml -n 5
  python3 verification/run_scenario.py -s verification/scenarios/head_on.yaml \
      -n 1 --gui   # Gazebo GUI로 육안 확인 (판정 로직은 동일)

수동 teleop 육안 확인은 통과 근거가 아니다 — 이 러너의 CSV가 근거다.
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import time

import yaml

import metrics
import worlds

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_WORLD = os.path.join(
    REPO, 'ros2_ws/src/go2_simulation/worlds/small_city.sdf')
SCENARIO_DIR = os.path.join(REPO, 'verification/scenarios')
RUNS_DIR = os.path.join(REPO, 'verification/runs')
DRIVER = os.path.join(REPO, 'verification/cmd_publisher.py')


def load_scenario(path):
    with open(path) as f:
        return yaml.safe_load(f)


# 스택 전체를 정리해야 한다. gz/bridge만 죽이면 launch가 SIGINT에 완전히
# 죽지 않은 경우 나머지 노드가 살아남아 다음 trial과 같은 ROS 그래프에서
# 계속 발행한다. 두 가지 형태로 나타난다 (2026-08-04 Phase 3.5 재기준화 실측):
#   · 지각/게이트 노드 생존 → 이전 시뮬의 장애물로 STOP/STUCK 난사
#     (유령 노드 34분 생존, static_stop travel 0.00m·대조군 stops=145)
#   · TF/odom 발행 노드(ekf_node, state_estimation_node,
#     robot_state_publisher, static_transform_publisher) 생존 → 다음 trial의
#     TF 트리 오염 → /obstacles/lidar 가 영영 안 나옴 → readiness timeout.
#     한 번 발생하면 이후 모든 trial 이 연쇄 실패한다 (bike_pass·crossing
#     10연속 FAIL → 정리 후 단독 실행은 PASS 로 확인)
#
# 목록은 launch 파일의 executable= 전수와 일치해야 한다 —
# test_harness_cleanup.py 가 드리프트를 차단한다.
# ⚠ cmd_publisher(드라이버)는 포함하지 않는다 — 실행 중 자기 자신을 죽인다.
# ⚠ 'create'/'spawner' 같은 흔한 이름은 반드시 경로째로 매칭한다.
LEFTOVER_PATTERNS = [
    'gz sim',
    'parameter_bridge',
    'ros_gz_sim/create',
    'controller_manager/spawner',
    'quadruped_controller_node',
    'state_estimation_node',
    'heading_correction_node',
    'robot_state_publisher',
    'static_transform_publisher',
    'ekf_node',
    'lidar_obstacle_node',
    'safety_stop_node',
    'collision_oracle_node',
    'sidewalk_polygon_node',
    'degrade_pointcloud_node',
    'odom_tf_broadcaster',
    'list_controllers',   # 기동 점검용 bash 프로브 — 웨지된 채 남는 것 실측
]


def kill_gazebo_leftovers():
    for pat in LEFTOVER_PATTERNS:
        subprocess.run(['pkill', '-f', pat], check=False)
    time.sleep(2.0)
    # 웨지된 gz 서버는 SIGTERM을 무시하고 살아남아 다음 trial의
    # /controller_manager 서비스를 오염시킴 — SIGKILL로 격상
    for pat in LEFTOVER_PATTERNS:
        subprocess.run(['pkill', '-9', '-f', pat], check=False)
    time.sleep(1.0)


def run_trial(scenario, trial_idx, degrade_profile, log, gui=False):
    run_dir = os.path.join(
        RUNS_DIR, f"{scenario['name']}_{trial_idx}_{int(time.time())}")
    os.makedirs(run_dir, exist_ok=True)
    world_path = os.path.join(run_dir, 'world.sdf')
    oracle_csv = os.path.join(run_dir, 'oracle.csv')
    states_csv = os.path.join(run_dir, 'states.csv')

    worlds.generate_world(
        BASE_WORLD, scenario.get('actors', []), world_path,
        scenario_name=scenario['name'])

    launch_cmd = [
        'ros2', 'launch', 'go2_simulation', 'unitree_go2_launch_small_city.py',
        f'gui:={"true" if gui else "false"}', 'rviz:=false',
        f'world:={world_path}',
        f'oracle_csv:={oracle_csv}',
    ]
    for k, v in (scenario.get('launch_args') or {}).items():
        launch_cmd.append(f'{k}:={v}')
    if degrade_profile:
        launch_cmd.append(f'degrade_profile:={os.path.abspath(degrade_profile)}')

    settle = float(scenario.get('settle_time', 8.0))
    duration = float(scenario['duration'])
    # 드라이버는 sim time 으로 settle/duration 을 재생하므로 필요한 벽시계는
    # (settle+duration)/RTF 다. 앞의 상수는 기동(gz 스폰 + 컨트롤러 + 첫 라이다
    # 프레임) 몫으로, 드라이버 자신의 readiness 상한(cmd_publisher 300s)보다
    # 커야 한다 — 구판 60은 하네스가 드라이버보다 먼저 끊어 기동이 느린 trial
    # 을 판정 실패가 아니라 인프라 실패로 기록했다.
    # 배수 8: RTF 0.25 는 캡이지 하한이 아니다. 머신이 못 따라가면 실제 RTF 가
    # 그 아래로 떨어지고, 특히 bike 시나리오(GLB 자전거 메시)는 GPU 라이다
    # 부하가 커 실측 RTF 가 0.07 까지 내려갔다 (33s 시뮬에 462s 초과 — 4배로는
    # 부족). 여유는 웨지된 trial 에서만 비용이 되므로 넉넉히 잡는다.
    wall_cap = 330 + 8 * (settle + duration)

    log.write(f"launch: {' '.join(launch_cmd)}\n")
    launch_log = open(os.path.join(run_dir, 'launch.log'), 'w')
    proc = subprocess.Popen(
        launch_cmd, stdout=launch_log, stderr=subprocess.STDOUT,
        start_new_session=True)

    driver_rc = -1
    try:
        driver_cmd = [
            sys.executable, DRIVER,
            '--profile', str(scenario['cmd_profile']).replace("'", '"'),
            '--settle', str(settle),
            '--duration', str(duration),
            '--states-csv', states_csv,
        ]
        log.write(f"driver: {' '.join(driver_cmd)}\n")
        driver = subprocess.run(
            driver_cmd, timeout=wall_cap,
            stdout=open(os.path.join(run_dir, 'driver.log'), 'w'),
            stderr=subprocess.STDOUT)
        driver_rc = driver.returncode
    except subprocess.TimeoutExpired:
        log.write('driver wall-timeout\n')
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        launch_log.close()
        kill_gazebo_leftovers()

    if driver_rc != 0:
        return {'run_dir': run_dir, 'passed': False,
                'failures': [f'driver rc={driver_rc} (readiness/timeout)'],
                'oracle': {'min_clearance': float('inf'), 'collisions': -1},
                'driver': {'states': [], 'stop_entries': -1, 'travel': -1,
                           'collision_events': -1}}

    oracle = metrics.parse_oracle_csv(oracle_csv)
    driver_m = metrics.parse_states_csv(states_csv)
    criteria = dict(scenario.get('criteria', {}))
    if degrade_profile:
        # 열화 런 전용 기준 오버레이 — 센서 열화로 정보 도달이 늦어 물리적으로
        # 달성 불가능한 항목(예: 조기정지 사치 마진)만 명시적으로 재정의.
        # 안전 의미론 항목(충돌/최소 이격/상태)은 오버레이하지 않는 것이 원칙.
        criteria.update(scenario.get('criteria_degraded', {}))
    passed, failures = metrics.evaluate(oracle, driver_m, criteria)
    return {'run_dir': run_dir, 'passed': passed, 'failures': failures,
            'oracle': oracle, 'driver': driver_m}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--scenario', action='append', default=[])
    parser.add_argument('--all', action='store_true',
                        help='scenarios/ 아래 4종 전부')
    parser.add_argument('-n', '--trials', type=int, default=5)
    parser.add_argument('--degrade', default=None,
                        help='열화 프로파일 YAML (real_l1 등)')
    parser.add_argument('--gui', action='store_true',
                        help='Gazebo GUI 표시 (육안 확인용 — 판정은 동일)')
    parser.add_argument('--out', default=os.path.join(
        RUNS_DIR, '..', 'phase3_regression.csv'))
    args = parser.parse_args()

    scenario_files = list(args.scenario)
    if args.all:
        scenario_files += sorted(
            os.path.join(SCENARIO_DIR, f)
            for f in os.listdir(SCENARIO_DIR) if f.endswith('.yaml'))
    if not scenario_files:
        parser.error('need -s or --all')

    os.makedirs(RUNS_DIR, exist_ok=True)
    out_exists = os.path.exists(args.out)
    out_f = open(args.out, 'a', newline='')
    writer = csv.writer(out_f)
    if not out_exists:
        writer.writerow(
            ['scenario', 'degrade', 'trial', 'pass', 'min_clearance',
             'collisions', 'stop_entries', 'travel_m', 'states', 'failures',
             'run_dir'])

    # 외부 잔존 gz/bridge가 첫 trial을 오염시키지 않도록 시작 전에도 정리
    kill_gazebo_leftovers()

    total = failed = 0
    for sf in scenario_files:
        scenario = load_scenario(sf)
        tag = os.path.basename(args.degrade) if args.degrade else '-'
        for i in range(args.trials):
            total += 1
            print(f"[{scenario['name']} #{i+1}/{args.trials} degrade={tag}] "
                  f"running...", flush=True)
            r = run_trial(scenario, i + 1, args.degrade, sys.stdout,
                          gui=args.gui)
            status = 'PASS' if r['passed'] else 'FAIL'
            if not r['passed']:
                failed += 1
            mc = r['oracle']['min_clearance']
            print(f"  {status} min_clr={mc:.3f} "
                  f"collisions={r['oracle']['collisions']} "
                  f"stops={r['driver']['stop_entries']} "
                  f"travel={r['driver']['travel']:.2f}m "
                  f"states={'/'.join(r['driver']['states'])}"
                  + (f" | {'; '.join(r['failures'])}" if r['failures'] else ''),
                  flush=True)
            writer.writerow([
                scenario['name'], tag, i + 1, status,
                f'{mc:.4f}' if mc != float("inf") else 'inf',
                r['oracle']['collisions'], r['driver']['stop_entries'],
                f"{r['driver']['travel']:.3f}",
                '/'.join(r['driver']['states']),
                '; '.join(r['failures']), r['run_dir']])
            out_f.flush()

    print(f'\n=== {total - failed}/{total} PASS → {args.out} ===')
    out_f.close()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
