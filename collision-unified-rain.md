# Unitree Go2 인도 장애물 회피 시스템 — 마스터 계획 (v2)

## Context

Unitree Go2의 인도(sidewalk) 자율주행을 위한 장애물 회피 시스템을 개발한다.
v1 계획(Phase 0~2.5)의 반응형 안전 게이트는 구현 완료됐고, v2는 다음을 추가한다:

- **A. 군중/사회적 회피**: 보행자 사이 gap 통과 판단, 통과 불가 시 인도 가장자리 양보(YIELD), 정적 장애물 밀착 통과
- **B. 고속 장애물**: 자전거/킥보드(2~7m/s) 속도 적응 대응 + latency 보상
- **C. 발 근처 장애물**: L1이 못 보는 0.3m 미만 지면 장애물 (D435i 근접장)
- **D. 돌발 출현**: 골목 튀어나옴/굴러오는 공/넘어지는 사람 — 폐색 인지 감속 + 긴급 대응
- **E. sim2real**: Go2 EDU + 외장 PC, sport mode 고수준 제어로의 검증 가능한 단계적 전이

확정 사항: 실기체 제어 = **sport mode velocity 명령**(unitree_ros2, Ethernet+CycloneDDS),
하드웨어 = **Go2 EDU + 외장 PC**, 발 근처 = **인지 후 조향회피+감속**(foothold 계획 없음),
인도 경계 = **시뮬 정적 polygon → 팀원 segmentation 스왑**(계약: `docs/interfaces.md`),
Nav2 = **필수 별도 트랙**(회피 스택 sim2real을 막지 않음).

---

## 핵심 설계 원칙

1. **회피는 Gazebo ground-truth에 의존하지 않는다.** Actor pose는 평가 oracle 전용.
   회피 판단은 실제 로봇 센서 토픽(`/unitree_lidar/points`, `/d435i/*`)만 사용.
   이 분리가 sim2real의 전제다.
2. **거리/TTC는 LiDAR 단독.** 카메라(class)는 임계 *보수화*에만 사용 — 완화 금지.
   자전거 대응에 필요한 감지 거리(~13m)는 D435i(10m)를 초과, L1(30m)만 가능.
3. **verified-gate 아키텍처**: 어떤 상위 레이어(YIELD 계획, Nav2, 학습 모듈)든
   기존 safety_stop 게이트 *위*에 얹힌다. 게이트의 STOP/ESTOP 의미론이 최종 권한.
4. **속도는 근접도의 단조 비증가 함수** (ISO 13482 정신) + protective stop 래치.
5. **고속 물체에는 회피(swerve)가 아니라 조기 정지.** latency 수학상 dodge는
   오히려 진로로 들어갈 수 있다 (Falanga RA-L'19). swerve는 측면 gap이 확인된
   경우만 (RSS Rule 5).
6. **얼어붙지 않는다** (freezing robot problem): 통과 가능한 gap이면 감속 통과,
   불가면 흐름 한가운데가 아니라 **가장자리로 이동 후 대기**. 과잉 조심은 그
   자체로 위험 (Trautman IJRR'15).

## 카메라 스펙 주의

시뮬은 D435i로 모델링돼 있으나 실물 교체 가능성 높음. 카메라 파라미터(intrinsics,
depth range, fov)는 YAML/CameraInfo로 분리, 알고리즘에 hardcode 금지. 팀원 depth
estimation 노드는 D435i depth 토픽과 drop-in 호환(`docs/interfaces.md` §3).

---

## 상태 원장 (Phase 0~2.5 = v1 완료분)

상세 as-built 기록: `docs/changelog/phase0-2.5_asbuilt.md`

| Phase | 내용 | 코드 | 검증 |
|---|---|---|---|
| 0 | 센서/브리지 (D435i+IMU, Velodyne 제거, L1 10Hz/360°×90°/0.8~30m) | ✅ | 러너 회귀로 대체 (Phase 3) |
| 0.5 | 평가 oracle (`actor_pose_publisher` plugin + `collision_oracle_node`, 다중 actor·model, per-run CSV) | ✅ | pytest + 러너 |
| 1 | `lidar_obstacle_node`(ROI/self-filter/클러스터/속도추적) + `safety_stop_node`(직사각 footprint, 상대속도 TTC, emergency backstop 0.25m) | ✅ | pytest + 러너 |
| 2 | FSM 7상태 (NOMINAL/SLOW/STOP/WAIT/RESUME/STUCK/ESTOP), TTC 히스테리시스 | ✅ | pytest + 러너 |
| 2.5 | STUCK: 창(window) net 변위 감지 → 회전 복구 → ESTOP 래치 | ✅ | pytest + 라이브 E2E |

cmd_vel 흐름: `/cmd_vel` → safety_stop(게이트) → `/cmd_vel_safe` → CHAMP → joint 명령.
(잔재 `/cmd_vel`→gz Twist 브리지는 제거됨 — 게이트 우회 경로 봉쇄)

---

## 검증 공통 원칙 (모든 phase)

1. **정량 PASS/FAIL 임계** — 애매하면 통과 아님.
2. **반복 N≥5** — 확률적 항목은 통계로 판정.
3. **자동화 시나리오 러너** — `verification/run_scenario.py`. 수동 teleop 육안 금지.
   trial마다 fresh headless Gazebo → 드라이버가 cmd 프로파일 재생 → oracle CSV +
   상태 로그 수집 → 임계 비교 → `verification/phaseN_*.csv` 집계.
4. **회귀** — 새 phase 진입 시 이전 phase 게이트 전부 재실행.
5. **산출물 의무화** — phase별 metrics CSV 없이는 통과 없음.
6. **pytest** — Gazebo 불필요한 순수 로직은 `perception_avoidance/test/`
   (`colcon test`). YAML↔기본값 정합도 테스트로 강제.
7. **열화 게이트 (신규, sim2real 인터리브)** — 매 phase에 `real_l1` 프로파일
   (`verification/profiles/`) 하 축소 게이트(N≥3) 포함. 실제 L1의 희소·노이즈
   특성에서 파라미터가 깨지는지 그때그때 확인 — 종단에 몰지 않는다.

⚠ **시나리오 월드 주의**: 정지 보행자는 actor가 아니라 static 실린더 모델로 표현
(`worlds.py`의 `static: true`). **퇴화 actor 궤적(제자리/초저속 세그먼트)은 센서
렌더링 활성 시 gz sim 스레드를 wedge**시킴 (2026-07-09 확인, 전 토픽/서비스
무응답 + `SceneBroadcaster: Timed out waiting for state` 증상). 생성기가 거부하며
pytest로 고정됨. 종료 후 정지는 `loop=false`의 마지막 waypoint로 표현.

---

## 시스템 아키텍처 (v2)

```text
[Gazebo Harmonic]  /  [실기체: unitree_ros2]
    │
    ├── /unitree_lidar/points  ──(옵션: degrade_pointcloud_node)──┐
    ├── /d435i/* (RGB, depth, IMU)                                │ perception 입력
    ├── /odom                                                     │
    │                                                            ─┘
    ├── /world/default/actor_pose/info (plugin)  → 평가 전용
    ▼
┌── EVALUATION ──────────────────────────────────────────┐
│ collision_oracle_node: clearance 시계열 CSV + 이벤트    │  ※ cmd_vel에 무영향
└─────────────────────────────────────────────────────────┘
┌── PERCEPTION ──────────────────────────────────────────┐
│ lidar_obstacle_node: 클러스터 + v_rel (Detection3DArray)│
│ ground_obstacle_node (P6): D435i 근접장 지면 편차       │
│ occlusion_speed_node (P7): free-space → /safety/speed_limit │
│ yolo_class_node + class_fusion_node (Track V): class 보수화 │
│ sidewalk_polygon_node (P5): 정적 YAML → PolygonStamped  │
│   (팀원 segmentation과 동일 토픽 — remap 스왑)          │
└─────────────────────────────────────────────────────────┘
┌── DECISION ────────────────────────────────────────────┐
│ social_gap_node (P5): gap 판단 → 통과 bias / YIELD 목표 │
│ safety_stop_node: 게이트 FSM                            │
│   NOMINAL/SLOW/STOP/WAIT/RESUME/STUCK/ESTOP             │
│   + YIELD_MOVE/YIELD_WAIT (P5)                          │
│   + CPA·속도조건부 임계·latency 보상 (P4)               │
│   + speed_limit 스케일 (P7), 지면 장애물 소스 (P6)      │
└─────────────────────────────────────────────────────────┘
    ▼ /cmd_vel_safe → CHAMP (시뮬) / sport_cmd_adapter (실기체, P8)
    ※ twist_mux + e-stop lock은 다중 소스(Nav2) 도입 시 (Track N)
```

---

## 핵심 기술 결정 (v2 추가분)

| 결정 | 선택 | 근거 |
|---|---|---|
| 반응 트리거 | TTC + **CPA(최근접점) 미스거리** | TTC만으론 옆으로 지나는 자전거에 false stop. `d_CPA < 유효반폭+0.2+0.4·t_CPA` AND `t_CPA<4s`일 때만 반응 |
| 고속 물체 임계 | \|v_obs\|>2m/s → stop_ttc 2.0s, slow_ttc 4~5s, **CPA 통과까지 STOP 유지** | ISO 22839 AEB 관행 + 자전거 latency 수학 |
| latency 보상 | 추적 장애물을 파이프라인 latency만큼 CVM 전방 전파 | 실측 latency(P8 G8-6)로 갱신 |
| 예측 | Constant Velocity 1~3s + 반경 팽창 0.4m/s·t | CVM이 학습 예측기와 대등 (Schöller RA-L'20). DRL 배제 |
| gap 판단 | 시도 ≥0.7m / 기본 ≥1.0m / 움직이는 보행자 사이 ≥1.2~1.4m | Go2 유효폭 0.5m(gait sway 포함) + 사회적 여유 (Kirby, Pacchierotti) |
| 사람 옆 통과 | 측면 ≥0.45m + 통과 중 ≤0.3m/s, 우측 통과 관습 | proxemics 친밀권 경계 + legibility |
| YIELD | 가장자리(연석 0.1~0.2m 여유)로 이동→정지→통과 후 재개 | Starship 운영 관행, freezing 방지 |
| 발 근처 | <0.10m 무시 / 0.10~0.30m 조향회피+감속 / **하강 단차 ≥0.12m 무조건 STOP** | Go2 맹목 한계 ~0.12m(스펙 0.15~0.16), L1 감지 하한 ~0.3m (CMU) |
| 폐색 | RSS Rule 4 팬텀(1.5m/s, 자전거 구역 2m/s) → `v_max=√(2·a·(d_vis−margin))`, 코너 상한 ~0.5m/s | occluded crosswalk 속도 제어 연구 |
| 열화 하네스 | `degrade_pointcloud_node` (밀도 1/5~1/10, σ2~5cm, dropout 5~20%, 지연 50~200ms) | 실제 L1 = 비반복 스캔 21,600pts/s — 시뮬 균일 링과 최대 격차 |
| 실기체 제어 | sport mode velocity (unitree_ros2), **Ethernet+CycloneDDS** | WiFi >1s 지연 보고 (CMU autonomy_stack_go2) |
| 시뮬 headless | `gui:=false` → `-s` 서버 전용 (GLX 렌더) | `--headless-rendering`(EGL)은 본 머신에서 서버 행 |

## 리서치 근거 (요약)

- **Freezing robot**: Trautman & Krause IROS'10 / IJRR'15 — 협조 모델 없인 군중에서 정지, 과잉 조심이 더 위험. → gap 통과 가능 시 감속 통과, 불가 시 가시적 edge-yield.
- **Proxemics/gap**: Hall 친밀권 0.45m; Kirby CMU'10, Pacchierotti '06 — 통과 여유 0.4~0.6m. → gap/측면 임계.
- **CVM**: Schöller RA-L'20 — 1~5s 지평선에서 상수속도 모델이 Social-LSTM/GAN과 대등. → 학습 예측기 불채택.
- **VO/CPA**: Fiorini & Shiller '98 — 이동 장애물엔 속도 공간 추론. → CPA 검사.
- **CBF**: Molnar RA-L'22 (Unitree A1 실증) — cmd_vel 레벨 unicycle CBF 실용. → P4 선택 항목.
- **Latency**: Falanga RA-L'19 — 인지 지연이 안전 속도 상한을 직접 결정. → 자전거 20km/h에 감지 13m 필요, 조기 STOP 원칙.
- **폐색**: RSS (Shalev-Shwartz '17) Rule 4/5, occluded crosswalk speed control (arXiv:1802.06314), OA-MPC (Stanford MSL). → 팬텀 에이전트 속도 거버너.
- **발 근처/전이**: CMU autonomy_stack_go2 — L1 감지 하한 0.3m 명문화, WiFi 지연 경고; ANYmal perceptive locomotion (Science Robotics '22) = foothold는 저수준 제어 필요 → 범위 외 확정.
- **표준**: ISO 13482 (protective stop, 근접 속도 제한), UL 4600 (시나리오별 증거 = phase-gate CSV 방식).

---

## v2 로드맵

재배치: 구 Phase 3(lateral) → **Phase 5에 흡수** · 구 Phase 4(YOLO) → **Track V** ·
구 Phase 5(Nav2) → **Track N (필수, 병렬)**.

### Phase 3: 검증 부채 상환 + 시나리오 러너 ✅ (2026-07-09)

v1 완료분의 검증 인프라를 계획 원칙에 맞게 구축. **모든 후속 phase의 전제.**

완료 내용:
- 커밋 정리 (Phase 0.5~2.5 전체가 논리 단위로 커밋됨) + 드리프트 4건 수정
  (docstring, roi_z_max, emergency_clearance, 잔재 cmd_vel 브리지)
- oracle per-run CSV (`csv_path`/launch `oracle_csv`)
- pytest 40개 (`colcon test`): TTC/corridor/FSM/stuck 창/추적/oracle/YAML 정합/열화 변환/월드 생성 가드
- 시나리오 러너 `verification/run_scenario.py` + 시나리오 4종
  (static_stop / head_on / crossing / empty) + `worlds.py`(actor 교체 월드 생성)
  + `cmd_publisher.py`(드라이버) + `metrics.py`
- 열화 하네스 `degrade_pointcloud_node` + `real_l1`/`worst_case` 프로파일
- `docs/interfaces.md`(팀원 계약), `docs/field_protocol.md`(P8~10 체크리스트)
- launch: `gui:=false` headless 지원, `oracle_csv`/`degrade_profile` 인자

게이트: pytest 전부 통과 · 4 시나리오 × N=5 회귀 (충돌 0, false stop 게이트 포함)
· `real_l1` 정적 정지 N≥3 · 산출물 `verification/phase3_regression.csv`

### Phase 4: 속도 적응형 게이트 — 요구 B (4~5일)

기존 노드 순수 업그레이드 (신규 인지원 없음):
1. CPA 계산 + 미스거리 트리거 (위 표) — false stop 제거
2. 속도 조건부 임계 (fast class \|v\|>2m/s) + **CPA 통과까지 STOP 유지**
3. latency 보상: 파이프라인 latency 측정·stamp → CVM 전방 전파 후 TTC/CPA
4. (선택) 이산 unicycle CBF로 `_scale_for_slow` 대체 — 부드러운 단조 속도
5. 월드: 자전거 actor (정면 5m/s, 측방 통과 1.5m 오프셋) — 13m 감지 확인,
   원거리 희소 대비 프레임 누적 검토

verify (N≥5, `phase4_metrics.csv`): 정면 자전거 5m/s STOP + oracle clearance ≥2.0m,
충돌 0/5 · 측방 통과 자전거 false stop 0/5 · CPA 전 RESUME 0/5 · 합성 트랙 pytest
· `real_l1` 정면 4m/s 3/3 · **Phase 3 회귀 전체 재통과**

파일: `safety_stop_node.py`, `lidar_obstacle_node.py`, `config/safety_stop.yaml`,
자전거 시나리오 YAML/월드

### Phase 5: 소셜 내비게이션 — 요구 A (8~10일, 5a/5b)

**5a — polygon + gap 판단 + 측면 통과**:
1. `sidewalk_polygon_node`: 정적 YAML(`sidewalk_boundaries.yaml`) →
   `/perception/sidewalk/boundary` (PolygonStamped) — 팀원 segmentation과 동일
   토픽/타입 (스왑 = remap 1줄)
2. `social_gap_node`: 클러스터 + polygon 단면에서 gap 계산 → 통과/YIELD 판단
   (0.7/1.0/1.2m 규칙, 속도 추정 반영)
3. 통과 bias: polygon 내 lateral 오프셋, 우측 관습, 사람 측면 ≥0.45m + ≤0.3m/s,
   **정적-보행자 사이 통과 시 정적 쪽 밀착**
4. 게이트는 filter 유지 — gap/YIELD 계획은 별도 노드가 cmd bias로 공급

**5b — YIELD 기동**: FSM에 YIELD_MOVE(가장자리로 저속 이동, 조기·가시적 커밋) →
YIELD_WAIT(정지, 통과 대기) → RESUME. 다중 보행자 월드 (2~3 actor: 마주 오는
흐름/추월/서 있는 그룹 — 서 있는 그룹은 static 실린더).

verify (N≥5, `phase5_metrics.csv`): gap 판단 pytest · 정지 보행자 2명 gap 1.0m →
통과, 각 clearance ≥0.2m, 충돌 0/5 · gap 0.5m + 마주 오는 보행자 → YIELD 도달,
polygon 이탈 0, 통과 후 ≤5s 재개 5/5 · 단일 통과 측면 ≥0.45m AND ≤0.3m/s 5/5 ·
정적 밀착 (d_static < d_ped) 5/5 · anti-freeze: 흐름 중앙 비양보 정지 ≤1/군중 run ·
회귀 + `real_l1`

### Phase 6: 발 근처 장애물 — 요구 C (5~7일)

1. `ground_obstacle_node`: D435i depth → 근접장(0.3~2.5m) 지면평면 편차 그리드
   (카메라 파라미터 `config/camera.yaml` 분리). 최소구현 우선; FastDEM/
   elevation_mapping_cupy는 성능 미달 시 업그레이드 경로
2. 출력 `/obstacles/ground` (Detection3DArray — LiDAR와 동일 계약, 게이트가
   두 소스를 동일 로직으로 소비)
3. 정책: 높이 <0.10m 무시 / 0.10~0.30m → Phase 5 조향회피 + ≤0.3m/s /
   **하강 단차 ≥0.12m → 무조건 STOP** (음의 편차 플래그)
4. L1 근거리 블라인드(0.8m) 부분 보완 효과
5. 월드: 0.15m 박스, 0.25m 돌, 공, 인도 가장자리 하강 연석 + oracle 정적 소품
   ground-truth (plugin `<model_name>` 다중 지원 — worlds.py가 자동 주입)

verify (N≥5, `phase6_metrics.csv`): 0.15m 박스 ≥1.5m에서 감지 5/5, 높이 오차
≤0.05m (합성 depth pytest 포함) · 평지 30s false positive 0 · 0.25m 돌 조향회피
접촉 0/5 + 회피 중 ≤0.3m/s · 하강 단차 전 정지 5/5 · 0.08m 클러터 무반응 5/5 ·
회귀 + depth 노이즈 프로파일

### Phase 7: 폐색 인지 감속 + 돌발 출현 — 요구 D (4~5일)

1. `occlusion_speed_node`: LiDAR free-space 폴리곤에서 폐색 경계 추출 → 각
   경계에 팬텀(1.5m/s, 파라미터) 가정 → `v_max=√(2·a_brake·(d_visible−margin))`
   → `/safety/speed_limit` (Float32). a_brake는 P8 실측값으로 갱신
2. 게이트가 speed_limit을 추가 스케일 항으로 소비 (최솟값 결합)
3. 돌발 출현 = 기본 긴급정지 (P4 기계 재사용: 고속 진입 물체는 fast-class 임계).
   swerve는 P5 gap 확인 시만
4. 월드: 블라인드 코너 벽, 지연 출발 actor(폐색에서 1.5m/s 출현), 굴러오는 공

verify (N≥5, `phase7_metrics.csv`): 블라인드 코너 0.8m/s 명령 → 코너 1m 내 실측
≤0.5m/s 5/5 · 폐색 출현 actor 충돌 0/5 + clearance >0.2m · 개활 직선 거버너
비활성 (평균 ≥0.95×명령) 5/5 — 과잉 감속 방지 게이트 · 공 굴러옴 정지 5/5 ·
폐색 추출 pytest · 회귀 + `real_l1`

### Phase 8~10: sim2real 하드웨어 트랙

체크리스트 정본: `docs/field_protocol.md`

- **P8 벤치 (3~5일, Phase 3 이후 로봇 확보 시 병렬 가능)**: `go2_real_bringup`
  신규 패키지 (어댑터/launch만, 알고리즘 0줄 — sim2real 경계). unitree_ros2,
  `sport_cmd_adapter`(Twist→sport velocity), `/utlidar/cloud` remap.
  게이트: cmd→다리 latency ≤0.2s · L1 ≥10Hz 10분 · SW e-stop ≤0.3s + HW e-stop ·
  **제동 감가속 실측 N≥5** → `safety_stop.yaml`/P7 파라미터 역반영 ·
  파이프라인 latency 실측 → P4 보상값 갱신
- **P9 teleop 센서 검증 (4~6일)**: 자율 OFF, 실제 인도 ≥3환경 ≥30분 bag →
  **bag 재생 오프라인 재튜닝** (`real_l1` 튜닝값에서 출발 — 델타 기록이 열화
  하네스 유효성 증거). 게이트: 보행자 ≥5m 감지율 ≥90% · STOP급 false cluster
  ≤1/5분 · 경사/잔디 지면 제거 유지 · 재튜닝 값으로 시뮬 전체 회귀 재통과
- **P10 단계적 필드 (2~3주)**: 지오펜스 + 속도캡 0.3~0.5m/s + 리모컨 e-stop 상주.
  (a) 빈 개활지 50m×5 개입≤1 전도0 → (b) +정적/연석/지면 장애물 접촉 0/5 →
  (c) +배우 보행자: 통과/양보 정답 ≥9/10, 사람 거리 항상 ≥0.45m, 충돌 0 →
  (d) 15분 연속×3, 개입/km 추세. 중단 규칙: 사고성 이벤트 → bag 분석 → 수정 →
  **시뮬 회귀 재통과 후** 재시작

### Track V (팀원 vision, 병렬)

- 계약: `docs/interfaces.md` (지금부터 유효). 스텁 퍼블리셔로 통합 사전 테스트.
- 경량 YOLO + `class_fusion_node` (구 Phase 4 내용): bbox↔클러스터 angular 매칭,
  class별 임계 **보수화만** (unknown ≥ person 보수). Phase 4 이후 아무 때나 통합.
  verify: 매칭율 ≥90%, false association ≤5%, **임계 완화 없음 pytest 강제**,
  안전 회귀 재통과.
- segmentation → P5 polygon 토픽 스왑. depth estimation → D435i drop-in.

### Track N (Nav2, 필수·병렬)

P10(b) 이후 권장. 구 Phase 5 내용 유지:
- LiDAR SLAM 2D 맵 + Nav2 (MPPI/DWB, 직사각 footprint 0.70×0.31)
- **정적/동적 분리 원칙**: 정적(건물·기둥)은 map known → planner가 정상 속도
  우회 ("과감"의 안전한 근거는 map이지 class가 아님 — 서 있는 사람=기둥 구분
  불가). 동적은 반응층 전담. freezing 방지 근거.
- safety_stop은 Nav2 출력 *아래* 최상위 안전 레이어 유지. `nav2_collision_monitor`
  와 A/B 비교 후 대체 여부 결정.
- **twist_mux + e-stop lock을 이 시점에 도입** (다중 cmd 소스 발생).
- P5의 lateral bias는 Nav2 활성 시 기본 OFF (cmd 충돌 방지).
- 개발 스캐폴딩: actor ground-truth → costmap 직접 입력 디버그 모드 (기본 OFF,
  sim2real 무영향).

---

## 시나리오 테스트 (누적)

| 시나리오 | Phase | 핵심 메트릭 |
|---|---|---|
| 정적 정지 / 정면 접근 / 횡단+재개 / 빈 인도 | 3 (러너 구축) | clearance, STOP 반응, false stop 0, 재개 |
| 자전거 정면/측방 통과 | 4 | 13m 감지, STOP 마진 ≥2m, false stop 0, CPA 유지 |
| 보행자 2명 gap (1.0m 통과 / 0.5m YIELD) | 5 | gap 판단 정답, polygon 이탈 0, 재개 시간 |
| 정적+보행자 밀착 통과 | 5 | d_static < d_ped |
| 지면 장애물 (0.08/0.15/0.25m, 하강 단차) | 6 | 과잉/과소 반응 0, 접촉 0 |
| 블라인드 코너 + 폐색 출현 + 공 | 7 | 코너 감속, 충돌 0, 개활 비감속 |
| 실기체 10a~10d | 8~10 | 개입/km, 사람 거리, 충돌 0 |

공통 통과 기준: 충돌 0 · false stop ≤1/시나리오 · sensor timeout 정지 <0.5s ·
stuck 자동 복구 · 각 phase 고유 임계는 해당 절 참조.

---

## 레이턴시 분석 (v2)

- L1 스캔 0.1s + 클러스터/판단 0.01~0.02s + cmd→CHAMP 0.005s ≈ **0.12~0.15s**
  (+ 열화 프로파일 지연 주입 0.05~0.2s로 상한 검증; 실기체 실측으로 대체 — P8 G8-6)
- 자전거 20km/h(5.6m/s), 로봇 0.4m/s → v_rel 6m/s: latency 이동 0.9m,
  stop_ttc 2s 기준 **필요 감지 거리 ≈ 13m** → L1 전용. 원거리 점밀도 희박 시
  2~3프레임 누적(+0.1~0.2s) 예산 포함.
- 로봇 제동(0.4m/s, 추정 2~3m/s²) ≈ 0.05~0.1m — **지배항은 latency + 장애물
  속도이지 로봇 제동이 아님**. 급정지 pitch 진동 방지: 긴급 외 STOP은 0.2~0.3s
  감속 램프.

## 리스크와 대응 (v2 추가/갱신)

| 리스크 | 대응 |
|---|---|
| **실제 L1 비반복·희소 스캔으로 클러스터/속도추정 파손** | 열화 게이트 인터리브 (매 phase) + P9 bag 재튜닝. 최대 단일 리스크 |
| 옆으로 지나는 자전거 false stop | P4 CPA 미스거리 트리거 |
| 자전거 STOP 후 조기 재개 → 진로 진입 | CPA 통과까지 STOP 유지 |
| 좁은 gap 무리한 통과 | 0.7m 하한 + YIELD 폴백 (P5) |
| YIELD 중 curb 이탈 | polygon 제약 + 연석 여유 0.1~0.2m + P6 하강 단차 STOP |
| L1이 낮은 장애물 못 봄 (<0.3m) | P6 D435i 근접장 노드 (L1에 이 역할 요구 금지) |
| 하강 단차(내려가는 연석) 낙하 | 음의 편차 ≥0.12m 무조건 STOP (상승보다 위험) |
| 폐색 과잉 감속 (개활지 기어감) | P7 개활 비감속 게이트 (평균 ≥0.95×명령) |
| class 오인식으로 임계 완화 | 완화 금지 불변식 + pytest 강제 (Track V) |
| WiFi 지연 >1s | Ethernet+CycloneDDS 고정 (P8) |
| 시뮬 제동/latency 가정 ≠ 실기체 | P8 실측 → 파라미터 역반영 → 시뮬 회귀 재실행 |
| **퇴화 actor 궤적이 sim wedge** | worlds.py 생성 가드(ValueError) + pytest. 정지 보행자는 static 모델 |
| Gazebo headless EGL 행 | `-s`(GLX)만 사용, `--headless-rendering` 금지 (본 머신) |
| Nav2·YIELD cmd 충돌 | Track N 진입 시 P5 bias 기본 OFF + twist_mux |
| (v1 유지) 자기 다리 오인식/false stop/stuck/후진 미감지 등 | v1 대응 유지 (self-filter, 히스테리시스, STUCK, 후진 차단) |

---

## 파일 지도

```text
ros2_ws/src/perception_avoidance/
  perception_avoidance/
    collision_oracle_node.py      # 평가 (CSV)
    lidar_obstacle_node.py        # LiDAR 클러스터+속도 [P4: latency stamp, 누적]
    safety_stop_node.py           # 게이트 FSM [P4: CPA/속도임계, P5: YIELD, P7: speed_limit]
    degrade_pointcloud_node.py    # 열화 하네스
    (P5) sidewalk_polygon_node.py, social_gap_node.py
    (P6) ground_obstacle_node.py
    (P7) occlusion_speed_node.py
    (V)  yolo_class_node.py, class_fusion_node.py
  config/  safety_stop.yaml, self_filter.yaml,
           (P5) sidewalk_boundaries.yaml, (P6) camera.yaml, ground_obstacle.yaml
  test/    pytest (colcon test)
ros2_ws/src/go2_simulation/       # 월드/launch/plugin
(P8) ros2_ws/src/go2_real_bringup/  # 실기체 어댑터 전용 (알고리즘 0)
verification/                     # 러너/시나리오/프로파일/메트릭 CSV
docs/interfaces.md                # 팀원 계약 (정본)
docs/field_protocol.md            # P8~10 체크리스트 (정본)
docs/changelog/                   # as-built 상세
```

## 실행 순서

1. ✅ Phase 3 (검증 인프라) — 완료, `verification/phase3_regression.csv`
2. Phase 4 (자전거/CPA) → 회귀
3. Phase 5 (gap/YIELD) → 회귀 — 팀원 계약은 이미 유효, segmentation 병렬 개발
4. Phase 6 (발 근처) → Phase 7 (폐색) → 회귀
5. P8 벤치는 로봇 확보 즉시 병렬 착수 가능 (Phase 3만 전제)
6. P9~P10 필드 → Track N (Nav2)
