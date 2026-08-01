# verification/ — 자동화 시나리오 러너

"검증 공통 원칙"의 집행자. 수동 teleop 육안 확인은 통과 근거가 아니다 —
이 러너가 남기는 CSV가 근거다.

## 사용법

```bash
cd ~/unitree_go2_sidewalk_navigation
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 전체 회귀 (scenarios/ 전부 × 5회)
python3 verification/run_scenario.py --all -n 5 --out verification/phase3_regression.csv

# 단일 시나리오
python3 verification/run_scenario.py -s verification/scenarios/static_stop.yaml -n 1

# 센서 열화 프로파일 (sim2real 게이트)
python3 verification/run_scenario.py -s verification/scenarios/static_stop.yaml \
    --degrade verification/profiles/real_l1.yaml -n 3
```

exit code 0 = 전부 PASS. trial별 산출물(월드/oracle CSV/상태 로그/launch 로그)은
`verification/runs/<시나리오>_<trial>_<ts>/` (gitignore됨).

## 구조

- `run_scenario.py` — trial마다 fresh headless Gazebo → 드라이버 실행 → 메트릭 판정
- `worlds.py` — 기본 월드(small_city.sdf)의 actor를 시나리오 명세로 교체한 변형 생성
- `cmd_publisher.py` — 드라이버: 준비 대기(/odom+/obstacles/lidar) → settle →
  cmd 프로파일 재생(sim time) → /safety/state 전이·이동거리 기록
- `metrics.py` — oracle CSV + 상태 로그 파싱, criteria 평가
- `scenarios/*.yaml` — 시나리오 정의 (actor 안무, cmd 프로파일, PASS 기준)
- `profiles/*.yaml` — 센서 열화 프로파일 (real_l1, worst_case)

## 시나리오 작성 규칙

- **이동 보행자는 사람 actor** — 단 안전 레시피를 생성기가 강제한다:
  `loop=true` + `tension="1.0"` + `interpolate_x` 미사용 + `type="route"`.
  `<loop>false</loop>`는 headless(-s) 센서 렌더링 초기화와 충돌해 sim 루프를
  wedge시킴 (증상: controller_manager 서비스 무응답 → readiness timeout).
- **정지 보행자는 `static: true`** (서 있는 사람 메시 모델 + oracle
  actor_radius와 일치하는 충돌 실린더 r=0.3). 제자리/초저속
  세그먼트(이동 <0.05m 또는 <0.02m/s)는 생성기가 거부한다.
- 이동 후 정지는 `loop: false`의 마지막 waypoint로 표현 (그 자리에 멈춤).
- criteria 키: `max_collisions`, `min_clearance_gt`, `require_states`,
  `forbid_states`, `max_stop_entries`, `min_travel_m`,
  `require_stop_during_actor_motion`(STOP이 actor 이동 중에 발화 — 접근
  시나리오의 타이밍 퇴화 방지),
  `min_clearance_at_stop_gt`(P4: 첫 STOP 진입 순간의 clearance — 고속 물체
  조기 정지 게이트; run 전체 min과 달리 이후의 측방 통과에 오염되지 않음),
  `require_no_resume_before_cpa`(P4: 최근접 통과(center_dist 최소 시각) 전
  RESUME 금지 — CPA 통과까지 STOP 유지 검증).
- `criteria_degraded:` — `--degrade` 런에서 기존 criteria 위에 겹치는 오버레이.
  센서 열화로 물리적으로 달성 불가한 항목(조기정지 사치 마진)만 명시적으로
  재정의하는 용도 — 안전 의미론 항목(충돌/최소 이격/상태)은 오버레이 금지.
- **고속 actor(자전거 대역)**: 보행자 actor를 고속 waypoint로 구동 (임시 표현 —
  최종적으로 사람+자전거 메시로 교체 예정). 안무는 접근 → 정지한 로봇을
  피해 가는 veer → 후방 이탈 hold 형태로, 로봇이 어떤 게이트 버전에서 멈춰도
  충돌하지 않게 설계한다 (`bike_head_on.yaml` 참조).

## Headless 주의

- `gui:=false`는 gz 서버 전용(-s)으로 실행하며 센서는 DISPLAY(GLX)로 렌더링.
  **`--headless-rendering`(EGL)은 이 머신(NVIDIA)에서 서버 행 유발 — 사용 금지.**
  진짜 무디스플레이 환경(CI)에서는 Xvfb 또는 EGL 설정 정비 필요.
- trial 간 Gazebo 프로세스를 완전히 재시작한다 (결정성 우선, ~60-90s/trial).
