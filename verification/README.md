# verification/ — 자동화 시나리오 러너

"검증 공통 원칙"의 집행자. 수동 teleop 육안 확인은 통과 근거가 아니다 —
이 러너가 남기는 CSV가 근거다.

## 사용법

```bash
cd ~/unitree_go2_sidewalk_navigation
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 전체 회귀 (시나리오 4종 × 5회)
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

- **gz `<actor>`는 사용 금지** — 스킨·속도 무관하게 headless(-s) 센서 렌더링
  초기화 시 sim 루프를 wedge시킴 (증상: controller_manager 서비스 무응답 →
  readiness timeout). 생성기가 이동 보행자를 WaypointMover 플러그인 구동
  실린더 model로 만들어주므로 시나리오 YAML 형식은 그대로다.
- **정지 보행자는 `static: true`** (정적 실린더 모델). 제자리/초저속
  세그먼트(이동 <0.05m 또는 <0.02m/s)는 생성기가 거부한다.
- 이동 후 정지는 `loop: false`의 마지막 waypoint로 표현 (그 자리에 멈춤).
- criteria 키: `max_collisions`, `min_clearance_gt`, `require_states`,
  `forbid_states`, `max_stop_entries`, `min_travel_m`.

## Headless 주의

- `gui:=false`는 gz 서버 전용(-s)으로 실행하며 센서는 DISPLAY(GLX)로 렌더링.
  **`--headless-rendering`(EGL)은 이 머신(NVIDIA)에서 서버 행 유발 — 사용 금지.**
  진짜 무디스플레이 환경(CI)에서는 Xvfb 또는 EGL 설정 정비 필요.
- trial 간 Gazebo 프로세스를 완전히 재시작한다 (결정성 우선, ~60-90s/trial).
