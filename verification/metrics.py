"""메트릭 파싱 + 임계 평가.

oracle CSV(clearance 시계열)와 driver states CSV(FSM 전이/이동거리)에서
trial 메트릭을 뽑아 시나리오 criteria와 비교, PASS/FAIL을 판정한다.
"""

import re


def parse_oracle_csv(path):
    """→ {min_clearance, collisions}. 파일 없음/행 없음 = actor 없는 시나리오."""
    min_clearance = float('inf')
    collisions = 0
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
    except FileNotFoundError:
        pass
    return {'min_clearance': min_clearance, 'collisions': collisions}


def parse_states_csv(path):
    """→ {states(순서, 중복 제거), stop_entries, travel, collision_events}."""
    states = []
    travel = 0.0
    stop_entries = 0
    collision_events = 0
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
    return {'states': states, 'stop_entries': stop_entries,
            'travel': travel, 'collision_events': collision_events}


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

    return (len(failures) == 0), failures
