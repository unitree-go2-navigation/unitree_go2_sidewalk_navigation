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
    """→ {min_clearance, collisions, actor_motion_end, clearance_series, cpa_t}.

    파일 없음/행 없음 = actor 없는 시나리오.
    actor_motion_end = 어떤 actor든 최종 위치에서 FINAL_DIST_EPS 밖에 있던
    마지막 sim 시각 (전혀 이동 안 했으면 None).
    cpa_t = center_dist가 최소인 sim 시각 (최근접 통과 시점 — P4 자전거 판정).
    """
    min_clearance = float('inf')
    collisions = 0
    tracks = {}                      # name → [(t, x, y), ...]
    clearance_series = []            # [(t, clearance), ...]
    robot_series = []                # [(t, x, y)] (t당 1개 — P5 계측)
    cpa_t = None
    cpa_dist = float('inf')
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
                    clearance_series.append(
                        (float(parts[0]), float(parts[2])))
                if len(parts) >= 4 and float(parts[3]) < cpa_dist:
                    cpa_dist = float(parts[3])
                    cpa_t = float(parts[0])
                if len(parts) >= 8:
                    tracks.setdefault(parts[1], []).append(
                        (float(parts[0]), float(parts[6]), float(parts[7])))
                    t = float(parts[0])
                    if not robot_series or robot_series[-1][0] != t:
                        robot_series.append(
                            (t, float(parts[4]), float(parts[5])))
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
            'actor_motion_end': actor_motion_end,
            'clearance_series': clearance_series, 'cpa_t': cpa_t,
            'robot_series': robot_series}


def parse_states_csv(path):
    """→ {states(순서, 중복 제거), stop_entries, travel, collision_events,
    first_stop_t(첫 STOP 진입 sim 시각, 없으면 None),
    first_resume_t(첫 STOP 이후 첫 RESUME 진입 시각, 없으면 None)}."""
    states = []
    travel = 0.0
    stop_entries = 0
    collision_events = 0
    first_stop_t = None
    first_resume_t = None
    first_yield_wait_t = None
    resume_after_yield_t = None
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
                if (parts[1] == 'RESUME' and first_stop_t is not None
                        and first_resume_t is None):
                    first_resume_t = float(parts[0])
                if parts[1] == 'YIELD_WAIT' and first_yield_wait_t is None:
                    first_yield_wait_t = float(parts[0])
                if (parts[1] == 'RESUME' and first_yield_wait_t is not None
                        and resume_after_yield_t is None):
                    resume_after_yield_t = float(parts[0])
    return {'states': states, 'stop_entries': stop_entries,
            'travel': travel, 'collision_events': collision_events,
            'first_stop_t': first_stop_t, 'first_resume_t': first_resume_t,
            'first_yield_wait_t': first_yield_wait_t,
            'resume_after_yield_t': resume_after_yield_t}


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

    # P4 조기 STOP 게이트: 첫 STOP 진입 시점의 oracle clearance가 임계 초과.
    # (고속 물체는 원거리에서 멈춰야 함 — run 전체 min_clearance는 이후의
    #  측방 통과(CPA)로 작아지므로 "STOP 순간" 값을 본다)
    clr_at_stop_gt = criteria.get('min_clearance_at_stop_gt')
    if clr_at_stop_gt is not None:
        stop_t = driver.get('first_stop_t')
        series = oracle.get('clearance_series') or []
        if stop_t is None:
            failures.append('no STOP entry (clearance_at_stop)')
        elif not series:
            failures.append('no oracle rows (clearance_at_stop)')
        else:
            clr = min(series, key=lambda r: abs(r[0] - stop_t))[1]
            if not clr > clr_at_stop_gt:
                failures.append(
                    f'clearance at STOP {clr:.2f} <= {clr_at_stop_gt}')

    # P4 CPA 유지 게이트: 최근접 통과(cpa_t) 전에는 RESUME 금지.
    if criteria.get('require_no_resume_before_cpa'):
        resume_t = driver.get('first_resume_t')
        cpa_t = oracle.get('cpa_t')
        if cpa_t is None:
            failures.append('no oracle rows (no_resume_before_cpa)')
        elif resume_t is not None and resume_t < cpa_t:
            failures.append(
                f'RESUME t={resume_t:.1f} before CPA t={cpa_t:.1f}')

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

    # ── P5 소셜 계측 ─────────────────────────────────────────────────
    robot = oracle.get('robot_series') or []

    # 통과 속도 상한: 사람 인접(clearance < 1.2) 구간의 로봇 속도.
    # 0.5s 창 평활 — 샘플 간(0.1s) 순간속도는 gait sway 첨두(±0.15)가 얹혀
    # 명목(명령 캡 0.3)과 정합하지 않음 (스모크 실측 0.61 스파이크).
    pass_v_lt = criteria.get('max_pass_speed_lt')
    if pass_v_lt is not None:
        series = oracle.get('clearance_series') or []
        near = {round(t, 2) for t, c in series if c < 1.2}
        worst = 0.0
        j = 0
        for i in range(len(robot)):
            t1, x1, y1 = robot[i]
            while robot[j][0] < t1 - 0.6:
                j += 1
            t0, x0, y0 = robot[j]
            if t1 - t0 < 0.4:
                continue
            if round(t0, 2) in near or round(t1, 2) in near:
                worst = max(worst, math.hypot(x1 - x0, y1 - y0) / (t1 - t0))
        if not robot:
            failures.append('no robot rows (max_pass_speed)')
        elif worst >= pass_v_lt:
            failures.append(f'pass speed {worst:.2f} >= {pass_v_lt}')

    # 횡변위 하한: 회피/양보가 실제로 일어났는가
    lat_shift = criteria.get('min_lateral_shift_m')
    if lat_shift is not None:
        if not robot:
            failures.append('no robot rows (lateral_shift)')
        else:
            y0 = robot[0][2]
            shift = max(abs(y - y0) for _, _, y in robot)
            if shift < lat_shift:
                failures.append(
                    f'lateral shift {shift:.2f}m < {lat_shift}m')

    # 밴드 이탈 금지: robot_y가 [min, max] 안 (polygon 이탈 0)
    y_rng = criteria.get('robot_y_within')
    if y_rng is not None and robot:
        lo, hi = y_rng
        worst_y = [y for _, _, y in robot if not (lo <= y <= hi)]
        if worst_y:
            failures.append(
                f'robot_y out of band [{lo},{hi}]: {worst_y[0]:.2f} '
                f'({len(worst_y)} rows)')

    # YIELD 재개 시한: 대기 진입 → RESUME까지
    y2r = criteria.get('max_yield_to_resume_s')
    if y2r is not None:
        yw = driver.get('first_yield_wait_t')
        rs = driver.get('resume_after_yield_t')
        if yw is None:
            failures.append('no YIELD_WAIT entry (yield_to_resume)')
        elif rs is None:
            failures.append('no RESUME after YIELD_WAIT (yield_to_resume)')
        elif rs - yw > y2r:
            failures.append(
                f'yield→resume {rs - yw:.1f}s > {y2r}s')

    return (len(failures) == 0), failures
