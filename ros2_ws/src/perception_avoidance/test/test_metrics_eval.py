"""metrics.py 파싱/판정 테스트 — 특히 require_stop_during_actor_motion.

이 criteria는 head_on의 타이밍 퇴화(로봇 주행 전에 actor 보행이 끝나
STOP이 정지 표적 상대로 발화하는 안무)를 FAIL로 잡기 위한 것이다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[4] / 'verification'))
import metrics  # noqa: E402


def write_oracle(tmp_path, rows):
    p = tmp_path / 'oracle.csv'
    lines = ['t,actor,clearance,center_dist,robot_x,robot_y,actor_x,actor_y']
    lines += [','.join(str(v) for v in r) for r in rows]
    p.write_text('\n'.join(lines) + '\n')
    return str(p)


def write_states(tmp_path, rows, travel=5.0):
    p = tmp_path / 'states.csv'
    lines = ['t,state'] + [f'{t},{s}' for t, s in rows]
    lines.append(f'# summary travel={travel} stop_entries=1 '
                 f'collision_events=0 states=x')
    p.write_text('\n'.join(lines) + '\n')
    return str(p)


def test_actor_motion_end_detected(tmp_path):
    # t=1~3 이동(0.5m/샘플, 최종 위치 5.5로 접근), t=4~5 유사정지(hold)
    rows = [(t, 'ped', 3.0, 3.5, 15.0, 5.2, 4.0 + 0.5 * t, 5.2)
            for t in range(1, 4)]
    rows += [(t, 'ped', 2.0, 2.5, 14.0, 5.2, 5.5, 5.2) for t in (4, 5)]
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    # 최종 위치(5.5)에서 0.05m 밖이던 마지막 시각 = t=2 (x=5.0)
    assert oracle['actor_motion_end'] == 2


def test_actor_motion_end_none_when_static(tmp_path):
    rows = [(t, 'ped', 3.0, 3.5, 15.0, 5.2, 10.0, 5.2) for t in range(1, 5)]
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    assert oracle['actor_motion_end'] is None


def test_first_stop_t_parsed(tmp_path):
    driver = metrics.parse_states_csv(write_states(
        tmp_path, [(10.0, 'SLOW_DOWN'), (12.5, 'STOP'), (20.0, 'WAIT')]))
    assert driver['first_stop_t'] == 12.5


def test_stop_during_motion_passes(tmp_path):
    oracle = {'min_clearance': 1.0, 'collisions': 0, 'actor_motion_end': 18.0}
    driver = {'states': ['SLOW_DOWN', 'STOP'], 'stop_entries': 1,
              'travel': 4.0, 'collision_events': 0, 'first_stop_t': 15.0}
    passed, failures = metrics.evaluate(
        oracle, driver, {'require_stop_during_actor_motion': True})
    assert passed, failures


def test_stop_after_motion_end_fails(tmp_path):
    # head_on 타이밍 퇴화 재현: actor는 t=12에 멈췄는데 STOP은 t=34
    oracle = {'min_clearance': 1.0, 'collisions': 0, 'actor_motion_end': 12.0}
    driver = {'states': ['SLOW_DOWN', 'STOP'], 'stop_entries': 1,
              'travel': 4.0, 'collision_events': 0, 'first_stop_t': 34.0}
    passed, failures = metrics.evaluate(
        oracle, driver, {'require_stop_during_actor_motion': True})
    assert not passed
    assert any('not during actor motion' in f for f in failures)


def test_no_stop_fails_when_required(tmp_path):
    oracle = {'min_clearance': 1.0, 'collisions': 0, 'actor_motion_end': 18.0}
    driver = {'states': ['SLOW_DOWN'], 'stop_entries': 0,
              'travel': 4.0, 'collision_events': 0, 'first_stop_t': None}
    passed, failures = metrics.evaluate(
        oracle, driver, {'require_stop_during_actor_motion': True})
    assert not passed


# ---- P4 자전거 판정: clearance@STOP + no-RESUME-before-CPA ----

def test_cpa_t_and_clearance_series_parsed(tmp_path):
    # center_dist가 t=3에서 최소(1.0) → cpa_t=3
    rows = [(1, 'bike', 5.0, 5.5, 15, 5.2, 9.5, 5.2),
            (2, 'bike', 2.0, 2.5, 15, 5.2, 12.5, 5.2),
            (3, 'bike', 0.6, 1.0, 15, 5.2, 15.0, 6.2),
            (4, 'bike', 3.0, 3.5, 15, 5.2, 18.5, 6.6)]
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    assert oracle['cpa_t'] == 3
    assert oracle['clearance_series'][0] == (1, 5.0)


def test_first_resume_t_parsed_only_after_stop(tmp_path):
    driver = metrics.parse_states_csv(write_states(
        tmp_path, [(10.0, 'SLOW_DOWN'), (12.0, 'STOP'),
                   (14.0, 'WAIT'), (15.5, 'RESUME')]))
    assert driver['first_resume_t'] == 15.5


def test_clearance_at_stop_gate(tmp_path):
    oracle = {'min_clearance': 0.9, 'collisions': 0,
              'clearance_series': [(10.0, 6.0), (12.0, 4.5), (14.0, 0.9)]}
    driver = {'states': ['STOP'], 'stop_entries': 1, 'travel': 5.0,
              'collision_events': 0, 'first_stop_t': 12.1}
    # STOP 시점(t≈12) clearance 4.5 > 2.0 → PASS
    passed, failures = metrics.evaluate(
        oracle, driver, {'min_clearance_at_stop_gt': 2.0})
    assert passed, failures
    # 늦은 STOP(t≈14, clearance 0.9) → FAIL
    driver['first_stop_t'] = 14.2
    passed, failures = metrics.evaluate(
        oracle, driver, {'min_clearance_at_stop_gt': 2.0})
    assert not passed
    assert any('clearance at STOP' in f for f in failures)


def test_no_resume_before_cpa_gate(tmp_path):
    oracle = {'min_clearance': 0.9, 'collisions': 0, 'cpa_t': 13.2}
    driver = {'states': ['STOP', 'WAIT', 'RESUME'], 'stop_entries': 1,
              'travel': 5.0, 'collision_events': 0,
              'first_stop_t': 11.0, 'first_resume_t': 12.5}
    passed, failures = metrics.evaluate(
        oracle, driver, {'require_no_resume_before_cpa': True})
    assert not passed
    assert any('before CPA' in f for f in failures)
    # CPA 이후 RESUME → PASS. RESUME 없음(계속 대기)도 PASS.
    driver['first_resume_t'] = 14.5
    assert metrics.evaluate(
        oracle, driver, {'require_no_resume_before_cpa': True})[0]
    driver['first_resume_t'] = None
    assert metrics.evaluate(
        oracle, driver, {'require_no_resume_before_cpa': True})[0]


# --- P5 소셜 계측 -----------------------------------------------------------

def test_pass_speed_gate(tmp_path):
    # 사람 인접(clr<1.2) 구간에서 0.5m/s 이동 → 상한 0.35 위반
    # (판정은 0.5s 창 평활 — oracle 샘플레이트 10Hz로 생성)
    rows = [(round(k * 0.1, 1), 'ped', 1.0, 1.2,
             15.0 - 0.05 * k, 5.2, 10.0, 5.2) for k in range(50)]
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    ok, fails = metrics.evaluate(
        oracle, {'states': [], 'stop_entries': 0, 'travel': 5},
        {'max_pass_speed_lt': 0.35})
    assert not ok and 'pass speed' in fails[0]
    ok, _ = metrics.evaluate(
        oracle, {'states': [], 'stop_entries': 0, 'travel': 5},
        {'max_pass_speed_lt': 0.6})
    assert ok


def test_lateral_shift_gate(tmp_path):
    rows = [(t, 'ped', 3.0, 3.0, 15.0, 5.2 - 0.2 * t, 10.0, 5.2)
            for t in range(5)]
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    ok, _ = metrics.evaluate(
        oracle, {'states': []}, {'min_lateral_shift_m': 0.5})
    assert ok
    ok, fails = metrics.evaluate(
        oracle, {'states': []}, {'min_lateral_shift_m': 1.5})
    assert not ok and 'lateral shift' in fails[0]


def test_band_stay_gate(tmp_path):
    rows = [(0, 'ped', 3.0, 3.0, 15.0, 5.2, 10.0, 5.2),
            (1, 'ped', 3.0, 3.0, 14.5, 6.9, 10.0, 5.2)]   # 밴드 밖
    oracle = metrics.parse_oracle_csv(write_oracle(tmp_path, rows))
    ok, fails = metrics.evaluate(
        oracle, {'states': []}, {'robot_y_within': [3.85, 6.5]})
    assert not ok and 'out of band' in fails[0]


def test_yield_to_resume_gate(tmp_path):
    driver = metrics.parse_states_csv(write_states(
        tmp_path, [(10.0, 'YIELD_MOVE'), (12.0, 'YIELD_WAIT'),
                   (16.0, 'RESUME'), (18.0, 'NOMINAL')]))
    ok, _ = metrics.evaluate(
        {'min_clearance': 9, 'collisions': 0, 'robot_series': []},
        driver, {'max_yield_to_resume_s': 5.0})
    assert ok
    ok, fails = metrics.evaluate(
        {'min_clearance': 9, 'collisions': 0, 'robot_series': []},
        driver, {'max_yield_to_resume_s': 3.0})
    assert not ok and 'yield→resume' in fails[0]
