# L1 → L2 델타 (Phase 3.5 하드웨어 정합 + 회귀 재기준화)

계획: `collision-unified-rain-v3.md` (v3.1.1) §1.1~1.4 · §6.1~6.6 · §9.1
날짜: 2026-08-04 · 브랜치: `worktree-phase3-5-l2-rebaseline`

> **이 문서의 목적** (§9.1): "L1에서 통과했던 값이 L2에서 얼마나 달라지는가"를
> 기록한다. 이 델타가 열화 하네스의 유효성과 이후 sim2real 예측 신뢰도의 근거다.
> 값 자체보다 **무엇이 무엇에서 유도되었는지**가 본문이다 (원칙 8).

---

## 0. 요약 — 이번 재기준화에서 실제로 바뀐 것

| 구분 | 내용 |
|---|---|
| 센서 모델 | 시뮬 라이다를 L2 공칭으로 전환, `lidar_gen:=l1\|l2` 스위치로 L1 재현 보존 |
| 재유도 | 사각 의존 파라미터 2종 · 프레임 주기 의존 파라미터 5종 |
| 🔴 **결함 2건 발견** | ① 회귀 하네스가 시행 간 프로세스를 정리하지 못해 판정 오염 ② base 월드에 RTF 캡이 없어 L2 운영점에서 게이트가 성립하지 않음 |
| 대장 | `sensor_generation: L1` 레코드 **잔여 0**, 신규 레코드 4종 등재 |

**②는 안전 의미론에 직접 닿는다**: 캡 없이는 정지 보행자에 SLOW_DOWN 자체가
발생하지 않고 0.316 m로 스쳐 지나갔다 (§3 참조).

---

## 1. 센서 스펙 델타 (§1.1)

| 항목 | v2 시뮬 (L1 설정) | v3 시뮬 (L2 설정) | 비고 |
|---|---|---|---|
| `update_rate` | 10 Hz | **5.55 Hz** | T_frame 0.10 → **0.18 s** |
| `range/min` | 0.8 m | **0.05 m** | ⚠ 0.8은 URDF 설정값 — L1 데이터시트도 0.05 (v3.1 정정) |
| `range/max` | 30 m | 30 m | 저반사율 15 m 제약은 세대 무관 (v2가 놓친 기존 제약) |
| 수평 samples | 600 | **480** | |
| 수직 samples | 30 | **24** | 수직 FOV 90° 유지 (96° negative angle 모드는 미도입) |
| 유효 포인트율 | 180,000 pts/s | **≈63,936 pts/s** | 480×24×5.55, 데이터시트 유효 64,000 근사 |
| 프레임당 점 수 | 18,000 | **11,520** | 실측 확인 |

구현: `unitree_go2_description/urdf/lidar_4D_lidar.xacro` 를 xacro 인자화.
기본 `lidar_gen:=l2`, `lidar_gen:=l1` 로 v2 설정을 그대로 재현한다 (§6.6 v3.1.1 권고).
launch(`unitree_go2_launch_small_city.py`)에 동일 인자 노출.

> 비반복 스캔 패턴·반사율별 dropout·FOV 밀도 불균일은 gz-sim 기본 라이다로
> 표현할 수 없다 → 열화 노드(`real_l2` 프로파일)가 담당한다 (§6.5).

---

## 2. 파라미터 재유도 델타

### 2.1 근거리 사각 의존 (🔴 유도 전제가 바뀐 항목)

v2의 유도식은 `min_range 0.8 − robot_half_length 0.35 + margin 0.15 = 0.60` 이었다.
L2에서 `min_range` 가 0.05가 되면서 **사각의 실효 경계가 센서에서 ROI 크롭으로 이동**한다.
즉 상수가 아니라 **유도식의 입력 심볼이 교체**된 경우다.

```
v2:  boundary = min_range(0.8)
v3:  boundary = max(min_range 0.05, self_x_max 0.45, roi_x_min 0.5) = 0.5
     clearance 기준 = 0.5 − 0.35 + 0.15 = 0.30
```

| 파라미터 | v2 | v3 | status |
|---|---|---|---|
| `emergency_clearance` | 0.60 | **0.30** | INVALID → PROVISIONAL |
| `blind_hold_clearance` | 0.60 | **0.30** | INVALID → PROVISIONAL |

원칙(정지 결정 경계는 사각 밖) 은 불변이고, 구조적 동형도 유지된다:
v2 `0.60+0.35=0.95 ≥ 0.8+0.15` ↔ v3 `0.30+0.35=0.65 ≥ 0.5+0.15`.

> ⚠ 완화 방향 변경이므로 **head_on·static_stop N≥5 재통과로만 유효화**된다
> (대장 note에 조건 명기). 실기 `min_range` 0.05 확정 시 blind-hold 메커니즘
> 존속 자체를 재평가한다.

### 2.2 프레임 주기(T_frame) 의존

| 파라미터 | v2 (T_frame 0.10) | v3 (T_frame 0.18) | 유도 |
|---|---|---|---|
| `publish_rate` | 10.0 | **5.55** | `= f_scan` |
| `assoc_max_dist` | 1.6 | **2.2** | `v_max 7.0 × T_frame + 여유 0.9` (프레임당 변위 0.75 → 1.26 m) |
| `vel_max_dt` | 0.5 | **0.8** | `(track_coast_frames 3 + 1) × 0.18 = 0.72` + 여유 |
| `sensor_timeout` | 0.5 | **0.9** | `5 × T_frame` (구판은 L2에서 3프레임 미만 → 오탐 STOP) |
| `cvm_latency_max` | 0.3 | **0.5** | `T_frame 0.18 + 열화 0.10 + 처리` — 구판은 실측 지연을 클립 |
| `track_coast_frames` | 3 | 3 (유지) | 프레임 단위이므로 코스팅 0.30 → **0.54 s** 로 실효 변화 |
| `fast_trig_frames` | 4 | 4 (유지) | 확정 지연 0.40 → **0.72 s** (§1.4 표 첫 행과 일치) |

`vel_max_dt` 는 v2에 잠재한 정합 결함이기도 했다: 코스팅 상한 직후 재연관 시
`dtm = 4 × T_frame = 0.72 s` 가 상한 0.5를 넘어 **속도 갱신이 조용히 스킵**되고
스테일 속도가 유지된다. L2에서 이 구간이 실사용 대역으로 들어와 명시 수정했다.

### 2.3 self-filter (시뮬 판, §6.1)

| 파라미터 | v2 | v3 | 유도 |
|---|---|---|---|
| `self_x_min/max` | ∓0.40 | **∓0.45** | `robot_half_length 0.35 + 다리 전후 스윙·마운트 0.10` |
| `self_y_*`, `self_z_*` | 유지 | 유지 | 측·수직은 시뮬에서 자기 유입 미관측 |

전제 자체가 바뀌었다. v2 헤더는 *"L1 min range 0.8이 몸체 대부분을 제거하므로
이 박스는 안전망"* 이었다 — L2(0.05 m)에서는 **박스가 1차 방어선**이 된다.
시뮬 G0는 `empty` 시나리오(보행 중 자기 유래 클러스터 0)로 판정한다.
실기체 판(gait 의존 다리 스윙 영역)은 P8'에서 측정 9D로 재조정한다.

### 2.4 열화 프로파일 (§6.5)

| 프로파일 | keep | dropout | noise | latency | status |
|---|---|---|---|---|---|
| `real_l1` | 0.20 | 0.10 | 0.03 | 0.10 | **DEPRECATED** (v2 재현 전용) |
| `worst_case` | 0.10 | 0.20 | 0.05 | 0.20 | **DEPRECATED** |
| **`real_l2`** | **1.0** | 0.10 | 0.02 | 0.10 | PROVISIONAL |
| **`worst_case_l2`** | 0.50 | 0.20 | 0.05 | 0.20 | PROVISIONAL |

`real_l2` 의 `density_keep_ratio` 가 1.0인 것이 핵심 델타다. `real_l1` 의 0.20은
*시뮬 균일 링(180,000 pts/s)을 L1 유효율로 깎는* 계수였다. 시뮬 센서가 이미
L2 공칭 포인트율로 내려온 지금 같은 계수를 얹으면 **이중 열화**가 된다.
따라서 `real_l2` 는 gz 라이다가 표현하지 못하는 것(비반복 스캔 커버리지 요동,
저반사 dropout, 측정 정확도, 파이프라인 지연)만 담당한다.

> ⚠ `real_l1` 은 반드시 `lidar_gen:=l1` 과 함께 써야 한다 (프로파일 헤더에 명기).

---

## 3. 🔴 재기준화 중 발견된 결함 2건

### 3.1 회귀 하네스가 시행 간 프로세스를 정리하지 못함

`kill_gazebo_leftovers()` 가 `gz sim` 과 `parameter_bridge` 만 종료하고
**파이썬 노드(`lidar_obstacle_node`/`safety_stop_node`/`collision_oracle_node`)와
`quadruped_controller_node` 는 남겼다.** launch가 SIGINT에 완전히 죽지 않으면
이들이 살아남아 다음 시행과 같은 ROS 그래프에 계속 발행한다.

실측 (2026-08-04): 유령 노드가 **34분 생존**. 이 상태에서의 판정은 전부 무의미했다.

```
오염된 실행:  static_stop  travel 0.00m  STUCK/NOMINAL 수백 회 → ESTOP
             (변경 전 코드 대조군도 동일: travel 1.70m, STOP 진입 145회)
정리 후:      static_stop  travel 10.20m  SLOW_DOWN/NOMINAL  PASS
```

→ `LEFTOVER_PATTERNS` 로 스택 전체를 종료하도록 수정
(`verification/run_scenario.py`). 드라이버(`cmd_publisher`)는 실행 중 자기 자신을
죽이므로 제외한다.

> 이 결함은 L2와 무관하며 **v2 baseline 수치의 신뢰도에도 소급 적용**된다.
> 과거 회귀에 산발적으로 나타난 설명되지 않는 STUCK/STOP 이상치의 유력한 후보다.

### 3.2 base 월드에 RTF 캡이 없어 L2 운영점이 성립하지 않음

회귀 하네스의 base 월드(`small_city.sdf`)에는 RTF 캡이 없다. 캡이 명시된 것은
`small_city_test*.sdf`(별도 경로)뿐이었다. 머신이 한가하면 RTF가 0.50까지 떠오르고,
요구 벽시계 렌더율이 GPU 라이다 상한(~2.5 Hz)을 넘어 **심시간 라이다 주기가 붕괴**한다.

```
실측 (2026-08-04):
  캡 없음(RTF 0.50)  심시간 라이다 주기 중앙값 0.72~0.90 s (≈1.4 Hz)
  캡 0.25            심시간 라이다 주기 중앙값 0.180 s   (설정값과 정합)
```

L1 시절에는 풀스택 부하가 RTF를 0.13~0.33으로 눌러줘 우연히 성립했기 때문에
캡이 base 월드에 명시되지 않았다 — **암묵적 전제였던 것이 L2에서 드러났다.**

A/B (클린 환경, `static_stop`):

| | 결과 | 상태 | 최소 이격 |
|---|---|---|---|
| 캡 0.25 | **PASS** | SLOW_DOWN 진입 | 0.409 m |
| 캡 없음 | **FAIL** | SLOW_DOWN 미진입 | **0.316 m** (보행자를 스쳐 지나감) |

→ `verification/worlds.py::generate_world` 가 월드 생성 시 캡을 주입하도록 수정.
**RTF 캡은 성능 편의가 아니라 게이트 성립 조건**이며, 이것이 §6.6 v3.1 항목
"RTF 운영점 재유도"의 답이다.

---

## 4. 대장(`params_ledger.yaml`) 상태 전이

| 레코드 | v2 status | v3 status |
|---|---|---|
| `emergency_clearance` | INVALID (L1) | PROVISIONAL (L2) |
| `blind_hold_clearance` | INVALID (L1) | PROVISIONAL (L2) |
| `roi_x_max` | INVALID (L1) | PROVISIONAL (L2) — 구속 미해소는 note에 명시 |
| `self_filter_box` | INVALID (L1) | PROVISIONAL (L2) |
| `cvm_latency_max` | PROVISIONAL (L1) | PROVISIONAL (L2), 0.3→0.5 |
| `assoc_max_dist` | PROVISIONAL (L1) | PROVISIONAL (L2), 1.6→2.2 |
| `track_coast_frames` | PROVISIONAL (L1) | PROVISIONAL (L2), 값 유지 |
| 신규 `sim_lidar_config` | — | PROVISIONAL (L2) |
| 신규 `degrade_profile` | — | PROVISIONAL (L2) |
| 신규 `sensor_timeout` | — | PROVISIONAL (L2) |
| 신규 `publish_rate` | — | PROVISIONAL (L2) |

`sensor_generation: L1` **잔여 0**. `INVALID` 잔여는 `cpa_growth` 1건이며 이는
센서가 아니라 C1(CP 분위수 교체) 대기 항목이라 Phase 3.5 범위 밖이다.

> `roi_x_max` 는 값(15.0)을 유지했다. §1.4 재유도로는 17.6 m가 필요하지만 가용
> 거리(저반사율 15 m)를 넘는다 — **구속 위반은 해소되지 않았고**, 해소 수단은
> `v_robot_max` 하향(§1.4 ③)이다. 시뮬은 PROVISIONAL 값으로 진행한다는
> §9.2 항목 6의 결정을 따르며, 이 사실을 대장 note에 남겼다.

---

## 5. 회귀 재기준화 결과

산출물: `verification/phase3.5_rebaseline.csv`
(시나리오 12종 × N=5 + `real_l2` 열화 세트)

pytest: **187/187 통과**. 임계값 변경에 맞춰 기준값을 갱신한 테스트:
blind-hold 8건(0.45→0.25 기준), 추적 2건(`assoc_max_dist`/`vel_max_dt`),
latency 클램프 1건(1.8→1.6 s).

> 결과 표는 아래 §5.1에 채운다.

### 5.1 시나리오별 결과

*(회귀 실행 완료 후 기재)*
