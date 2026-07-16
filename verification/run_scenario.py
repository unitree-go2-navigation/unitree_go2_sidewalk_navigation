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


def kill_gazebo_leftovers():
    subprocess.run(['pkill', '-f', 'gz sim'], check=False)
    subprocess.run(['pkill', '-f', 'parameter_bridge'], check=False)
    time.sleep(2.0)
    # 웨지된 gz 서버는 SIGTERM을 무시하고 살아남아 다음 trial의
    # /controller_manager 서비스를 오염시킴 — SIGKILL로 격상
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'parameter_bridge'], check=False)
    time.sleep(1.0)


def run_trial(scenario, trial_idx, degrade_profile, log, gui=False):
    run_dir = os.path.join(
        RUNS_DIR, f"{scenario['name']}_{trial_idx}_{int(time.time())}")
    os.makedirs(run_dir, exist_ok=True)
    world_path = os.path.join(run_dir, 'world.sdf')
    oracle_csv = os.path.join(run_dir, 'oracle.csv')
    states_csv = os.path.join(run_dir, 'states.csv')

    worlds.generate_world(BASE_WORLD, scenario.get('actors', []), world_path)

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
    wall_cap = 60 + 4 * (settle + duration)

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
    passed, failures = metrics.evaluate(
        oracle, driver_m, scenario.get('criteria', {}))
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
