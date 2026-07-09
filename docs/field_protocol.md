# 실기체 전이 프로토콜 (Phase 8~10)

Go2 EDU + 외장 PC(Ethernet+CycloneDDS) 기준. **각 단계의 게이트를 통과하기 전에는
다음 단계로 진입하지 않는다.** 모든 자율주행 세션에 리모컨 e-stop 담당 1인 필수.

인쇄해서 현장 체크리스트로 사용할 것. 산출물은 `verification/phase8~10_*.csv` / `verification/field/`.

---

## Phase 8 (H0): 벤치 브링업 — 로봇 스탠드/하네스 위

**목적**: 통신·제어 경로 검증 + 시뮬에서 가정한 파라미터를 실측으로 교체.

준비물: Go2 EDU, 외장 PC, Ethernet 케이블, 스탠드(발이 땅에 닿지 않게), 리모컨.

- [ ] `go2_real_bringup` 패키지: unitree_ros2 설치, CycloneDDS 설정 (**WiFi 금지** — CMU 보고 >1s 지연)
- [ ] `sport_cmd_adapter` 확인: `/cmd_vel_safe`(Twist) → sport mode velocity 명령
- [ ] `/utlidar/cloud` → 기존 perception 파이프라인 remap 확인 (frame TF 포함)
- [ ] 오도메트리 소스 선정: `/utlidar/robot_pose`(lidar-inertial) vs `/sportmodestate`(leg+IMU)

**게이트 (전부 수치 기록, N≥5)**:

| # | 항목 | 기준 | 측정 방법 |
|---|---|---|---|
| G8-1 | cmd→다리 반응 latency | ≤ 0.2s | cmd 발행 시각 ↔ `/lowstate` 관절 변화 시각 |
| G8-2 | L1 rate 안정성 | ≥ 10Hz, 10분, gap < 1% | `ros2 topic hz` 로그 |
| G8-3 | SW e-stop | zero 도달 ≤ 0.3s | 게이트 ESTOP 강제 후 lowstate 확인 |
| G8-4 | HW e-stop (리모컨) | 동작 확인 | damping 명령 |
| G8-5 | **제동 감가속 실측** | 값 기록 (추정 2~3 m/s²) | 지상, 0.5m/s에서 StopMove, odom 미분, N≥5 |
| G8-6 | 파이프라인 latency 실측 | 값 기록 (시뮬 가정 0.15s 대체) | 센서 stamp ↔ cmd 발행 wall time |

G8-5/G8-6 실측값을 `safety_stop.yaml`(stop 거리·TTC)과 occlusion 파라미터(Phase 7)에
**역반영**하고 시뮬 회귀를 재실행한다.

---

## Phase 9 (H1): Teleop 전용 센서 검증 — 자율주행 OFF

**목적**: 실제 L1 데이터로 perception 재튜닝. 실 환경 최대 리스크(비반복·희소 스캔) 해소.

- [ ] 자율 기능 전부 OFF (`safety_gate:=false`, 게이트는 기록만)
- [ ] 실제 인도 ≥ 3환경(넓은/좁은/경사·잔디 인접)에서 teleop 주행, 총 ≥ 30분 rosbag
  (`/utlidar/cloud`, `/odom`, `/tf`, 카메라, 보행자 통행 포함)
- [ ] **bag 재생으로 시뮬 튜닝 perception 오프라인 검증** — `real_l1` 프로파일 값에서 출발해
  `cluster_min_points`, `vel_alpha`, `assoc_max_dist` 재튜닝. 델타를 기록
  (델타가 작으면 열화 하네스가 유효했다는 증거).

**게이트**:

| # | 항목 | 기준 |
|---|---|---|
| G9-1 | 보행자 감지율 | ≥ 5m 거리에서 프레임 ≥ 90% (bag 재생) |
| G9-2 | STOP급 false cluster | ≤ 1회 / 5분 |
| G9-3 | 지면 제거 | 경사/잔디 경계에서 지면 point가 obstacle로 승격 0 |
| G9-4 | 재튜닝 회귀 | 재튜닝 값으로 **시뮬 전체 회귀 재통과** (러너, real_l1 프로파일) |

---

## Phase 10 (H2): 단계적 필드 자율주행

공통 안전 조치 (모든 세션):
- [ ] 리모컨 e-stop 담당 1인 (로봇 5m 이내 유지)
- [ ] 지오펜스 폴리곤 활성 (이탈 시 자동 정지)
- [ ] 속도 캡 0.3~0.5 m/s
- [ ] 세션마다 시작 전 G8-3/G8-4 e-stop 재확인
- [ ] 전 세션 rosbag 기록 + 개입(intervention) 로그

**단계별 게이트 (순서 고정, 각 단계 통과 후 다음 진입)**:

| 단계 | 환경 | 기준 |
|---|---|---|
| 10a | 빈 개활지 | 50m 직진 ×5회: 개입 총 ≤1, 전도 0, false stop ≤1/회 |
| 10b | + 정적 장애물·연석·지면 장애물 | 접촉 0/5, 하강 단차 앞 정지 5/5 |
| 10c | + 배우 보행자 (팀원, 시나리오 연기) | 통과/양보 판단 정답 ≥ 9/10, 사람과 거리 항상 ≥ 0.45m, 충돌 0 |
| 10d | 연속 운용 | 15분 연속 ×3회, 개입/km 기록 (추세 하강 확인) |

10c 시나리오 (시뮬 회귀와 동형): 정면 접근, 횡단, 좁은 gap(통과 불가 → YIELD), 자전거 통과
(안전 위해 배우는 자전거를 **끌고** 지나감).

**중단 규칙**: 충돌/전도/지오펜스 이탈/예상외 거동 발생 시 세션 즉시 중단 →
bag 분석 → 원인 수정 → **시뮬 회귀 재통과 후** 해당 단계 재시작.
