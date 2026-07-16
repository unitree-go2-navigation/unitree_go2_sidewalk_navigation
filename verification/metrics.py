"""메트릭 파싱 + 임계 평가.

oracle CSV(clearance 시계열)와 driver states CSV(FSM 전이/이동거리)에서
trial 메트릭을 뽑아 시나리오 criteria와 비교, PASS/FAIL을 판정한다.
"""

import math
import re

# actor가 "최종(hold) 위치"에서 이보다 멀면 아직 이동 중으로 본다.
# (샘플 간 순간 변위 기준은 스플라인 easing으로 waypoint 근처에서 속도가
#  출렁여 저속 꼬리를 놓침 — 최종 위치 기준이 샘플레이트에 무관하게 강건)
FINAL_DIST_EPS = 0.05


def parse_oracle_csv(path):
    """→ {min_clearance, collisions, actor_motion_end}.

    파일 없음/행 없음 = actor 없는 시나리오.
    actor_motion_end = 어떤 actor든 최종 위치에서 FINAL_DIST_EPS 밖에 있던
    마지막 sim 시각 (전혀 이동 안 했으면 None).
    """
    min_clearance = float('inf')
    collisions = 0
    tracks = {}                      # name → [(t, x, y), ...]
    try:
        with open(path) as f:
            header = f.readline()
            for line in f:
                if line.startswith('#'):
                    m = re.search(r'collisions=(\d+)', line)
                    if m:
                        collisions = int(m.group(1))
                    continue
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    min_clearance = min(min_clearance, float(parts[2]))
                if len(parts) >= 8:
                    tracks.setdefault(parts[1], []).append(
                        (float(parts[0]), float(parts[6]), float(parts[7])))
    except FileNotFoundError:
        pass
    actor_motion_end = None
    for rows in tracks.values():
        fx, fy = rows[-1][1], rows[-1][2]
        for t, x, y in reversed(rows):
            if math.hypot(x - fx, y - fy) > FINAL_DIST_EPS:
                if actor_motion_end is None or t > actor_motion_end:
                    actor_motion_end = t
                break
    return {'min_clearance': min_clearance, 'collisions': collisions,
            'actor_motion_end': actor_motion_end}


def parse_states_csv(path):
    """→ {states(순서, 중복 제거), stop_entries, travel, collision_events,
    first_stop_t(첫 STOP 진입 sim 시각, 없으면 None)}."""
    states = []
    travel = 0.0
    stop_entries = 0
    collision_events = 0
    first_stop_t = None
    with open(path) as f:
        f.readline()
        for line in f:
            if line.startswith('#'):
                for key, cast in (('travel', float), ('stop_entries', int),
                                  ('collision_events', int)):
                    m = re.search(rf'{key}=([\d.]+)', line)
                    if m:
                        if key == 'travel':
                            travel = cast(m.group(1))
                        elif key == 'stop_entries':
                            stop_entries = cast(m.group(1))
                        else:
                            collision_events = cast(m.group(1))
                continue
            parts = line.strip().split(',')
            if len(parts) == 2 and (not states or states[-1] != parts[1]):
                states.append(parts[1])
                if parts[1] == 'STOP' and first_stop_t is None:
                    first_stop_t = float(parts[0])
    return {'states': states, 'stop_entries': stop_entries,
            'travel': travel, 'collision_events': collision_events,
            'first_stop_t': first_stop_t}


def evaluate(oracle, driver, criteria):
    """criteria 딕셔너리 평가 → (passed, [실패 사유])."""
    failures = []

    max_collisions = criteria.get('max_collisions')
    if max_collisions is not None and oracle['collisions'] > max_collisions:
        failures.append(
            f"collisions {oracle['collisions']} > {max_collisions}")

    min_clr = criteria.get('min_clearance_gt')
    if min_clr is not None and not (oracle['min_clearance'] > min_clr):
        failures.append(
            f"min_clearance {oracle['min_clearance']:.3f} <= {min_clr}")

    for st in criteria.get('require_states', []) or []:
        if st not in driver['states']:
            failures.append(f"state {st} never entered "
                            f"(saw {'/'.join(driver['states'])})")

    for st in criteria.get('forbid_states', []) or []:
        if st in driver['states']:
            failures.append(f"forbidden state {st} entered")

    max_stops = criteria.get('max_stop_entries')
    if max_stops is not None and driver['stop_entries'] > max_stops:
        failures.append(
            f"stop_entries {driver['stop_entries']} > {max_stops}")

    min_travel = criteria.get('min_travel_m')
    if min_travel is not None and driver['travel'] < min_travel:
        failures.append(f"travel {driver['travel']:.2f}m < {min_travel}m")

    # 상대속도 경로 게이트: STOP이 actor 이동 "중"(종료 0.5s 이전)에 발화해야 함.
    # 타이밍 퇴화(주행 시작 전에 actor 보행이 끝나는 안무)를 FAIL로 잡는다.
    if criteria.get('require_stop_during_actor_motion'):
        stop_t = driver.get('first_stop_t')
        motion_end = oracle.get('actor_motion_end')
        if stop_t is None:
            failures.append('no STOP entry (stop_during_actor_motion)')
        elif motion_end is None:
            failures.append('actor never moved (stop_during_actor_motion)')
        elif stop_t >= motion_end - 0.5:
            failures.append(
                f'STOP t={stop_t:.1f} not during actor motion '
                f'(motion ended t={motion_end:.1f})')

    return (len(failures) == 0), failures
