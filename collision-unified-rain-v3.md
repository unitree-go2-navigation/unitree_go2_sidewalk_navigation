# Unitree Go2 인도 장애물 회피 시스템 — 마스터 계획 (v3.1)

> **v2 대체 문서.** v2의 Phase 0~3 as-built 기록, 검증 공통 원칙, 시나리오 러너 구조는
> 그대로 유효하며 여기서 반복하지 않는다 (`collision-unified-rain.md` v2 §"상태 원장",
> §"검증 공통 원칙" 참조). 이 문서는 **v2 대비 변경분과 그 근거, 그리고 그로 인해
> 수정해야 하는 코드**를 정본으로 기록한다.

**개정 사유 3가지**

| # | 사유 | 성격 |
|---|---|---|
| H | **하드웨어 변경**: LiDAR L1 → **L2**, 기체 Go2 EDU → **Go2 X** | 외부 사실 변경 → v2 유도값 다수 무효 |
| S | **소프트웨어 환경 확정**: ROS 2 **Jazzy** | v2에 미기재. Nav2 가용 기능 제약 |
| R | **최신 연구 반영**: 예측 불확실성 정량화, 4족 CBF 필터, 조향 권한 단일화 | 구조 개선 (값 무관) |

> **v3.1 (2026-08-01) 정정 개정.** 검토(문헌·데이터시트·Nav2 소스·리포 실사) 반영:
> ① P5·Track N **기구현** 사실 반영 — C3는 신규 설계가 아니라 이관 (§3.3, §6.7, §9.3)
> ② "L1 근거리 블라인드 0.8 m" 전제 정정 — L1 데이터시트도 0.05 m, 0.8은 URDF 값 (§1.1~1.2)
> ③ 저반사율 15 m 제약은 L1(RM)도 동일 — 신규 제약이 아니라 기존에 놓친 제약 (§1.1)
> ④ 재기준화 범위에 P4·P5 게이트 포함 (§1.3, §9.1)
> ⑤ D435i → **ToF 카메라** 확정(2026-07-10) 반영 (§3.1, §5.4, §9.4, §9.7)
> ⑥ 대장에 `emergency_clearance`/`blind_hold_clearance`/ToF 사양 추가 (§4.3)
> ⑦ L2 드라이버 타임스탬프 버그 검증 추가 (§8.1), 배포판 명칭 Lyrical Luth 확정 (§1.7)
> ⑧ CP 1차를 클래스 통합(pooled)으로 완화 + ACI 승격 경로 (§7.4), 계층 2 활성 순서 명문화 (§3.2)
> ⑨ 고유수용성(족저력·IMU) ESTOP 입력 신설 (§6.11), Phase 3.5에 RTF 운영점 재유도 추가 (§6.6)

---

## §0. v3 변경 요약

### 0.1 유지하는 것 (v2의 강점 — 건드리지 않는다)

1. **회피는 Gazebo ground-truth에 의존하지 않는다.** Actor pose는 평가 oracle 전용.
2. **verified-gate 아키텍처**: 모든 상위 레이어는 `safety_stop` 게이트 *위*에 얹힌다.
3. **고속 물체에는 swerve 금지, 조기 STOP.**
4. **열화 게이트 인터리브**: 매 phase에 실센서 열화 프로파일 축소 게이트 포함.
5. **정량 PASS/FAIL + N≥5 + 자동 러너 + phase별 CSV 산출물 의무화.**
6. **class는 임계 보수화에만 사용, 완화 금지 불변식.**
7. **정적/동적 분리 원칙**: "과감함"의 근거는 map이지 class가 아니다.

이 7개는 v3에서 그대로 정본이다. 특히 4·5는 이번 L1→L2 사고에서 **피해를 국지화시킨
장치**였으므로 강화한다 (§1.4).

### 0.2 무효화되는 것 (하드웨어 변경)

| v2 항목 | 상태 | 사유 |
|---|---|---|
| 필요 감지 거리 ≈ 13 m | **INVALID** | 스캔 주기 10 Hz 가정이 5.55 Hz로 변경 (§1.1) |
| 레이턴시 합계 0.12~0.15 s | **INVALID** | 동일 |
| `real_l1` 열화 프로파일 (21,600 pts/s, 균일 링 대비) | **INVALID** | 포인트율·스캔 패턴 변경 (§1.1) |
| L1 근거리 블라인드 0.8 m | **정정 (v3.1)** | 0.8 m는 URDF/시뮬 설정값 — L1 데이터시트도 0.05 m. **self-filter 전면 재작성 필요**는 유효하되 세대 무관 실기체 리스크 (§1.2) |
| L1 지면 감지 하한 0.3 m (CMU) | **UNVERIFIED** | L2 negative angle 모드로 완화 가능 → Phase 6 존재 이유 재평가 |
| 시뮬 라이다 센서 설정 (10 Hz / 90° / 0.8~30 m) | **INVALID** | L2로 재설정 → **Phase 1~5 게이트 전부 재실행 필요** (P4 46종·P5 소셜 포함, §1.3) |
| 실기체 제어 = unitree_ros2 Ethernet+CycloneDDS | **AT RISK** | Go2 X SKU의 DDS 접근 권한 미확인 (§1.5) |

### 0.3 새로 들어가는 것 (구조 개선)

| # | 변경 | 근거 (상세는 §7) |
|---|---|---|
| C1 | 예측 반경 `0.4 m/s · t` 고정상수 → **conformal prediction 분위수** | Lindemann RA-L'23 |
| C2 | 확정 프레임 수 `N=4` 고정 → **클래스별 파라미터 `N_fast` / `N_ped`** | 센서 스캔 주기 변경 대응 |
| C3 | `social_gap_node`가 cmd bias 대신 **goal(PoseStamped)** 생산 | 조향 권한 단일화 |
| C4 | 안전 계층을 **3계층으로 명시** (계획 비용 / 제어 필터 / 센서 직결 게이트) | One Filter (Go1), Molnar RA-L'22 |
| C5 | footprint를 등방·고정 직사각 → **속도 의존 비등방** | Sankaranarayanan '26 (Spot), Nav2 VelocityPolygon |
| C6 | `Detection3DArray`에 **`TrackedObstacle` 병행 토픽** 추가 | navigation2_dynamic 정렬 |
| C7 | **파라미터 대장** 도입 + 근거 층위에 **④ 센서 사양 의존** 추가 | 이번 사고의 재발 방지 장치 |
| C8 | fast-class는 **costmap 투영 금지** (Track N 사전 결정) | 좁은 인도 데드락 회피 |
| C9 | **벤더 회피 가드 노드** + `ObstaclesAvoidClient` 금지 CI | 이중 회피 차단 |
| C10 | 측정 프로토콜 9종 + **진입 게이트 G1~G5** 신설 | PENDING 값의 확정 경로 명문화 |

### 0.4 v2 서술 정정 1건

v2 Track N에 이미 **"P5의 lateral bias는 Nav2 활성 시 기본 OFF (cmd 충돌 방지)"**가
명기되어 있다 (v2 line 288). 이 원칙은 v3에서 **더 강하게** 재정의된다 — OFF 스위치가
아니라 **bias를 만들지 않는 구조로 이관**한다 (C3, §3.3 — v3.1: 기구현 vy 주입에서의
이관임에 유의).

---

## §1. 하드웨어 변경 영향 분석 (H)

### 1.1 LiDAR L1 → L2: 스펙 대조

| 항목 | v2 전제 (L1) | L2 실측/카탈로그 | 영향 등급 |
|---|---|---|---|
| 수평 스캔 주기 | 10 Hz (시뮬), 11 Hz (실기) | **5.55 Hz** | 🔴 치명 |
| 근거리 블라인드 | 0.8 m (⚠ URDF 설정값 — L1 데이터시트는 0.05 m) | **0.05 m** | 🔴 치명 — 단 **세대 무관** (v3.1 정정, §1.2) |
| 유효 포인트율 | 21,600 pts/s | **64,000 pts/s** (샘플 128,000) — *측정 5로 확정* | 🟠 중 |
| 수직 FOV | 360° × 90° | 360° × 90° 기본 / **96° negative angle 모드** | 🟠 중 |
| 최대 거리 | 30 m | **30 m @90% / 15 m @10% 반사율** — v3.1: L1(RM)도 동일 30/15 | 🔴 치명 (신규 제약이 아니라 v2가 놓친 기존 제약) |
| 수직 스캔 | 180 Hz | 216 Hz | 🟢 경미 (개선) |
| 측정 정확도 | ±2 cm | 2 cm / 거리 해상 4.5 mm | 🟢 경미 |
| 점군 밀도 분포 | 균일 가정 | **FOV 영역별 불균일, 중심부 고밀도** | 🟠 중 |

> ⚠ 제품 페이지 표기 불일치: Go2 제품 설명은 "21,600 samples/s"라고 기재하지만
> L2 단품 데이터시트는 유효 64,000 pts/s를 명시한다. 21,600은 L1 값(43,200)의 정확히
> 절반이라 **스펙 블록이 오래된 값을 상속했을 가능성**이 높다. (v3.1: 현행 공식
> 페이지는 L2 표기로 갱신 확인 — 21,600은 L1 유효율이었다.) 측정 5에서
> `/utlidar/cloud` 실측으로 확정하기 전에는 어떤 값도 계획에 넣지 않는다.

### 1.2 🔴 근거리 유입: self-filter가 깨진다 (v3.1 정정 — 세대 무관)

**v2에서 가장 간과되기 쉬운 리스크.** ⚠ v3.1 정정: "L1은 0.8 m 이내를 보지 않는다"는
전제는 **틀렸다** — L1 공식 매뉴얼도 근거리 블라인드 0.05 m를 명시하며, 0.8 m는 우리
URDF/시뮬 설정값이다 (`safety_stop.yaml` 주석의 "L1 min range 0.8m(URDF)"가 출처를
스스로 기록하고 있다). 즉 이 리스크는 L2 전환 때문이 아니라 **어떤 세대든 실기체 최초
기동 시** 발현될 문제였고, 시뮬만 0.8 m 블라인드 덕에 조용했던 것이다. Go2 몸통 반폭
0.155 m, 보행 중 다리 스윙 실효폭 0.45~0.5 m는 전부 실기체 가시 영역 안에 있다.

예상 증상:
- 자기 다리 점군이 매 프레임 대량 유입 → 클러스터 폭증
- 다리가 주기적으로 스윙하므로 **속도를 가진 근접 장애물로 오추적** → 즉시 STOP 래치
- 몸통 하부·배터리·마운트 반사 → 상시 emergency backstop(0.60 m) 침범

**이것은 실기체 최초 기동 시 즉시 발현되고, 로봇이 아예 움직이지 못하는 형태로
나타날 것이다.** `self_filter.yaml`을 L2 기하로 전면 재작성하는 것이 P8' 최우선 작업.

동시에 v2 Phase 6의 목적 중 하나였던 "L1 근거리 블라인드(0.8 m) 부분 보완"은
**스펙 오인에 기반한 목적이었을 가능성**이 크다 (실기 근거리 특성은 측정 5·8로 확정).
Phase 6의 존속 여부는 남은 목적(하강 단차 검출)만으로 판정한다 (§9.4).

또한 v3.1 추가: `emergency_clearance`(0.60)·`blind_hold_clearance`(0.60)는 yaml 주석에
유도식(`min_range 0.8 − half_length 0.35 + margin 0.15`)이 명시된 **min_range 의존
파라미터**다 — 실기 min_range 확정 시 재유도하며, 0.05 m가 확인되면 blind-hold
메커니즘 자체의 존속을 재검토한다 (§4.3).

### 1.3 🔴 시뮬 센서 재설정 → 통과한 게이트가 무효화된다

v2 Phase 0은 시뮬 라이다를 `10 Hz / 360°×90° / 0.8~30 m`로 모델링했다. 이를 L2로
바꾸면(5.55 Hz / 96° / 0.05~30 m) **점군의 시간·공간 특성이 모두 달라지므로**,
그 위에서 통과한 Phase 1~5 게이트(v3.1: P4 46종·P5 소셜 포함)는 증거로서의 효력을 잃는다.

```
영향 받는 게이트:
  Phase 1  클러스터/속도추적, safety_stop footprint, emergency backstop 0.60 m
  Phase 2  TTC 히스테리시스 (프레임 주기 의존)
  Phase 2.5 STUCK 창(window) 변위 감지 (프레임 주기 의존)
  Phase 3  4 시나리오 × N=5 회귀 전체
  Phase 4  46 게이트 전부 — fast_trig_frames·CPA·latency 보상이 프레임 주기 의존
           (phase4_metrics.csv 기준선 무효)                        ← v3.1
  Phase 5  소셜/YIELD 게이트 — 접근 판정 프레임 수·양보 속도 창이 주기 의존
           (phase5_metrics.csv 기준선 무효)                        ← v3.1
```

→ **Phase 3.5 신설** (§9.1): 하드웨어 정합 + 회귀 재기준화. 이걸 건너뛰고 Phase 4로
가면 이후 모든 측정이 오염된 baseline 위에 쌓인다.

### 1.4 🔴 감지 거리 재유도: 13 m 요건이 성립하지 않는다

v2 §레이턴시 분석의 재계산:

```
v2:  L1 스캔 0.10 s + 판단 0.015 s + cmd 0.005 s          ≈ 0.12 s
     v_rel 6 m/s × (stop_ttc 2.0 + 0.15)                  ≈ 12.9 m → "13 m"

v3:  L2 스캔 0.18 s + 판단 0.015 s + cmd/전송 T_transport  ≈ 0.20 s + T_transport
     확정 지연 N_fast × 0.18 s 를 명시적으로 포함
     T_total = N_fast·T_frame + T_perception + T_transport + T_actuation
```

| N_fast | T_confirm | T_total (추정) | 필요 감지 거리 |
|---|---|---|---|
| 4 (v2 기본) | 0.72 s | ~0.94 s | **17.6 m** |
| 3 | 0.54 s | ~0.76 s | 16.6 m |
| 2 | 0.36 s | ~0.58 s | **15.5 m** |
| 1 | 0.18 s | ~0.40 s | 14.4 m |

**가용 거리는 저반사율(10 %) 표적에서 15 m가 카탈로그 상한**이고, 클러스터를 형성해
속도를 추정할 수 있는 실제 거리 `R_confirm`은 그보다 짧다. 어두운 옷을 입은 자전거
탑승자가 최악 케이스다.

→ 세 자유변수 중 하나를 양보해야 한다:

```
① N_fast 하향       (오발률 상승 감수)  ← 측정 6C 로 결정
② stop_ttc 하향     (안전 마진 축소)
③ v_robot_max 하향  (v_rel 축소)       ← 권장. 유일하게 안전 방향

   v_robot_max = R_confirm_p10 / (stop_ttc + T_total) − v_bike_assumed
```

**③이 v3의 새 구속조건이다.** v2에는 "로봇 속도 상한이 센서 검출 거리에서 유도된다"는
연결이 없었다. 이 식이 §4 파라미터 대장의 핵심 항목이 된다.

### 1.5 🟠 Go2 EDU → Go2 X: 확정 사항이 흔들린다

v2 line 12·14의 확정 사항 두 개가 SKU 변경으로 재검증 대상이 된다.

```
v2 확정: 하드웨어 = Go2 EDU + 외장 PC
         실기체 제어 = sport mode velocity (unitree_ros2, Ethernet + CycloneDDS)
```

Go2 X는 EDU와 다른 SKU다. v3.1 확인: 공식 Go2 스펙표는 Secondary Development를
**AIR ○ / PRO ○ / X ◕(부분) / EDU ●(전체)**로 표기하며 **◕의 정의는 공개돼 있지
않다.** 커뮤니티 자료는 AIR/PRO의 DDS 기본 잠김만 언급하고 X는 다루지 않는다.
→ **측정 3(로봇 도착 후)까지 기다리지 말고 구매처/Unitree에 서면 확인을 지금
요청한다** — G2 실패의 파급(계획 전면 재검토)을 고려하면 가장 싼 보험이다.
검증해야 할 것:

| 검증 항목 | 실패 시 영향 |
|---|---|
| Ethernet + CycloneDDS로 `unitree_ros2` 접속 가능? | ❌ → WebRTC/WiFi만 남음. **v2가 이미 기록한 "WiFi >1 s 지연" 리스크가 기본 경로로 승격** |
| `/utlidar/cloud` DDS 직접 구독 가능? | ❌ → WebRTC 경유 점군 = 지연·손실 급증, fast-class 불가 |
| `SportClient` velocity 명령 API 접근 가능? | ❌ → 제어 경로 없음. 계획 전면 재검토 |
| P8 게이트 `cmd→다리 latency ≤ 0.2 s` 달성 가능? | ❌ → §1.4의 T_actuation이 커져 필요 감지 거리가 더 늘어남 |

**저수준 관절 제어는 필요 없다** (v2에서 foothold 계획을 범위 외로 확정했으므로).
따라서 EDU 전용 기능 중 우리가 잃는 것은 없고, **문제는 DDS/Ethernet 접근 권한 하나로
국지화된다.** 이것이 측정 프로토콜 3번(전송 경로 판정)의 최우선 판정 항목이다.

### 1.6 🟠 벤더 회피와의 이중화 (해소됨)

Go2에는 `obstacles_avoid` 서비스와 `ObstaclesAvoidClient`가 있고, 이는 온보드 센서로
장애물을 감지해 이동 명령을 자동 조정한다. v3.1 확인: API ID 1001/1002는 unitree_sdk2
헤더(`obstacles_avoid_api.hpp`)에서 실물 확인했다. 단 "동시 사용 금지" 경고문은 공식
문서에서 원문 인용을 확보하지 못했다(커뮤니티 구현들은 배타 사용으로 취급) — C9
가드의 근거는 **액추에이터 단일 소유 원칙**으로 충분하다.

```
✅ /cmd_vel_safe → sport_cmd_adapter → SportClient.Move()      ← 회피 우회
❌ /cmd_vel_safe → ObstaclesAvoidClient.Move()                 ← 이중 회피 (금지)
```

즉 v2가 이미 선택한 sport mode 경로가 구조적으로 옳다. 다만 스위치 상태는 로봇에
지속되는 상태이므로 **명시적 OFF + 검증 + 기록**이 필요하다 (C9, §6.11).

```
ObstaclesAvoidClient.SwitchSet(False)   # API ID 1001, {"enable": false}
ObstaclesAvoidClient.SwitchGet()        # API ID 1002 → 검증
```

### 1.7 ROS 2 Jazzy 확정 (S)

| Nav2 기능 | Jazzy | v3 대응 |
|---|---|---|
| Collision Monitor `VelocityPolygon` | ✅ | **C5(속도 의존 footprint)의 구현 수단으로 채택** |
| Collision Monitor Stop/Slowdown/Limit/Approach | ✅ | 기존 게이트와 A/B 비교 (Track N) |
| MPPI `OmniMotionModel` | ✅ | `vy_max ≪ vx_max` 제약 필수 (측정 9C) |
| MPPI Eigen 재구현 (+40~50 %, ARM) | ❌ Kilted 이후 | 측정 4에서 Jetson 처리량 실측 후 백포트 판단 |
| MPPI `model_delay_*` (축별 지연 보상) | ❌ Lyrical Luth(2026-05, LTS) 이후 | **v2의 CVM 전방전파를 그대로 유지** (자체 구현이 정답) |
| SpeedFilter `enable_path_lookahead` | ❌ Lyrical Luth 이후 | **v2의 `occlusion_speed_node`를 그대로 유지** |
| `unitree_ros2` 공식 지원 배포판 | Foxy / Humble만 | 🔴 Jazzy 미검증 + CycloneDDS 0.10.2 핀 문제 (측정 3) |

**결론: Jazzy 제약이 v2 설계를 정당화한다.** 지연 보상과 폐색 감속을 자체 노드로
구현해 둔 v2의 선택이 Jazzy에서는 유일한 선택지다. 반면 드라이버 계층은 마찰이 있다.
(v3.1: 후속 배포판은 Lyrical Luth로 확정 출시됐으나(2026-05-22) Ubuntu 24.04 유지가
전제인 한 Jazzy 고수가 옳다.)

---

## §2. 개정된 핵심 설계 원칙

v2의 원칙 1~6은 유지. 다음을 **수정·추가**한다.

### 원칙 2 (수정) — 거리/TTC는 LiDAR 단독

> ~~자전거 대응에 필요한 감지 거리(~13 m)는 D435i(10 m)를 초과, L1(30 m)만 가능.~~
>
> **개정**: 필요 감지 거리는 `v_rel × (stop_ttc + T_total)`로 유도되며, T_total은
> 센서 스캔 주기에 의존한다. **가용 거리는 카탈로그 최대 거리가 아니라 저반사율
> 표적의 트랙 확정 거리 `R_confirm_p10`이다** (측정 7). 두 값이 충돌하면 로봇 속도
> 상한을 낮춘다.

### 원칙 5 (강화) — 고속 물체에는 조기 정지

> 추가: **fast-class 장애물의 예측 궤적을 costmap에 정적 lethal로 투영하지 않는다.**
> 5 m/s 표적의 2 s 예측을 투영하면 인도를 가로지르는 10 m 길이의 벽이 생겨 가용
> 자유공간이 소멸하고, 이는 협소 통로에서 데드락을 유발한다 (Sankaranarayanan '26이
> 등방 거리 제약으로 실증한 것과 동일한 메커니즘). fast-class는 **게이트 전담**,
> 예측 투영은 pedestrian-class 한정.

### 원칙 7 (신설) — 조향 권한 단일화

> **정상 주행 중 횡방향 명령을 만드는 주체는 항상 하나뿐이다.**
> 회피 결정 모듈(`social_gap_node`)은 **목표(goal)** 를 생산하고, 그 목표를 속도로
> 바꾸는 것은 단일 추종기(초기: 얇은 bias 추종기, Track N 이후: MPPI)가 담당한다.
> 안전 계층(게이트·CBF·Collision Monitor)은 **감속·정지·상한만** 하고 조향하지 않는다.
>
> 허용 개입: 속도 상한 / 일시 정지 / 모드 전환 요청 / 최소 침습 CBF 보정
> 금지 개입: 독립 lateral planner / 장시간 누적되는 `vy` bias / 매 프레임 회피 방향 재결정

### 원칙 8 (신설) — 근거 층위에 ④ 센서 사양 의존을 둔다

> 근거 문서의 기존 3층위(①문헌 앵커 ②물리·기하 유도 ③시뮬 실측 보정)에
> **④ 센서 사양 의존**을 추가한다. ④에 속한 값은 모든 레코드에
> `sensor_generation` 필드를 필수로 갖고, 센서 세대가 바뀌면 **전수 재계산 대상으로
> 자동 표시된다.** 유도 결과(스칼라)만 기록하지 않고 **유도식과 입력 심볼을 함께**
> 기록한다.

이번 L1→L2 사고의 근본 원인은 `ROI = 15 m`, `필요 감지 거리 = 13 m` 같은 **유도
결과만 문서에 박고 유도 과정을 흘린 것**이다. 원칙 8은 그 재발을 구조적으로 막는다.

### 원칙 9 (신설) — PENDING 파라미터로는 기동하지 않는다

> 파라미터 대장에서 `status: PENDING`인 값을 사용하는 노드는 **런타임에 기동을
> 거부**한다 (실기체 안전 파라미터에 한함). 시뮬 개발 시에는 `allow_pending: true`로
> 우회하되, 그 실행의 모든 산출물 CSV에 `pending_params: [...]` 열이 기록된다.

---

## §3. 목표 아키텍처 (v3)

### 3.1 전체 파이프라인

```text
[Gazebo Harmonic (L2 모델)]        /        [실기체: Go2 X + 외장 PC]
    │                                            │
    ├── /unitree_lidar/points                    ├── /utlidar/cloud
    │     └(옵션) degrade_pointcloud_node        │
    ├── /d435i/* (시뮬 잔존 — 교체 예정)          ├── /tof/* (ToF 카메라 — D435i 미사용 확정, 스펙 미정)
    ├── /odom                                    ├── /lowstate, /sportmodestate
    └── /world/.../actor_pose/info → 평가 전용   └── /vendor_avoid_state ← C9 신규
    ▼
┌── EVALUATION ────────────────────────────────────────────────────┐
│ collision_oracle_node: clearance 시계열 CSV + 이벤트              │ ※ cmd에 무영향
│   + pending_params 열, vendor_avoid_state 열  ← 신규              │
└──────────────────────────────────────────────────────────────────┘
┌── PERCEPTION ────────────────────────────────────────────────────┐
│ lidar_obstacle_node                                              │
│   · self-filter (L2 근접 기하로 재작성)          ← H 수정         │
│   · 클러스터 + v_rel + covariance + confidence   ← C6 확장        │
│   · N_fast/N_ped 클래스별 확정                   ← C2             │
│   · 프레임별 조건 만족 이력 로깅                 ← 측정 6C 전제    │
│   · 출력: /obstacles (Detection3DArray, 유지)                     │
│           /obstacles/tracked (TrackedObstacleArray) ← C6 신규     │
│ prediction_node  ← 신규 (C1)                                     │
│   · CVM 전방전파 (T_total 만큼)                                  │
│   · CP 분위수 조회 → 클래스별·지평별 예측 반경                    │
│   · 출력: /prediction/tubes                                       │
│ ground_obstacle_node (P6', 조건부 존속)                          │
│ occlusion_speed_node (P7) → /safety/speed_limit                  │
│ yolo_class_node + class_fusion_node (Track V): 보수화만           │
│ sidewalk_polygon_node (P5) → /perception/sidewalk/boundary        │
└──────────────────────────────────────────────────────────────────┘
┌── DECISION (계층 1: 계획) ───────────────────────────────────────┐
│ social_gap_node (P5')                                            │
│   · gap 판단 → PASS / YIELD / WAIT 결정                          │
│   · 출력: /social/decision (모드) + /social/goal (PoseStamped)    │
│   · ⚠ vy bias를 직접 생성하지 않음               ← C3 구조 변경   │
│ goal_follower  ← 신규 (P5' 임시), Track N 에서 MPPI 로 교체       │
│   · /social/goal → cmd_vel                                       │
└──────────────────────────────────────────────────────────────────┘
┌── SAFETY (계층 2: 제어 필터, 선택) ──────────────────────────────┐
│ cbf_filter_node (P4 선택 항목, v2에서 이미 예정)                  │
│   · 최소 침습 QP: min‖u − u_nom‖ s.t. CBF 조건                    │
│   · 비등방 타원 안전영역 (C5)                                     │
└──────────────────────────────────────────────────────────────────┘
┌── SAFETY (계층 3: 센서 직결 게이트 — 최종 권한) ─────────────────┐
│ safety_stop_node                                                 │
│   NOMINAL/SLOW/STOP/WAIT/RESUME/STUCK/ESTOP                      │
│   + YIELD_MOVE/YIELD_WAIT (P5, goal 기반으로 재구현)              │
│   + CPA·클래스별 임계·latency 보상 (P4)                           │
│   + CP 분위수 기반 예측 반경 (C1)                                 │
│   + 속도 의존 비등방 footprint (C5)                               │
│   + speed_limit 스케일 (P7), 지면 장애물 소스 (P6')               │
│   + PENDING 파라미터 가드 (원칙 9)                                │
└──────────────────────────────────────────────────────────────────┘
    ▼ /cmd_vel_safe
    ├── (시뮬) CHAMP → joint
    └── (실기체) sport_cmd_adapter → SportClient.Move()
                 ↑ go2_vendor_avoid_guard 가 SwitchGet()==False 를 보증  ← C9
```

### 3.2 3계층 안전 게이트의 역할 분리 (C4)

| 계층 | 노드 | 하는 일 | 하지 않는 일 |
|---|---|---|---|
| 1 계획 | `social_gap_node`, (Track N) Nav2 planner/BT | 어디로 갈지, 통과할지 양보할지 | 속도 명령 직접 생성 |
| 1' 추종 | `goal_follower` → MPPI | **유일한 조향 주체** | 안전 판정 |
| 2 제어 필터 | `cbf_filter_node` (선택) | 최소 침습 보정 | 목표 변경, 대규모 우회 |
| 3 센서 직결 | `safety_stop_node`, (비교) Collision Monitor | 감속·정지·상한·래치 | **조향** |

계층 3이 최종 권한이라는 v2 원칙은 유지. 계층 2는 v2 Phase 4의 "(선택) 이산 unicycle
CBF"를 정식 계층으로 승격한 것이다.

**활성 순서 (v3.1)**: 다단 필터 상호 간섭(계층 2가 계층 3의 트리거 영역으로 조향해
들어가는 경우) 문헌 경고에 따라, 계층 2는 **계층 1+3 베이스라인(Phase 4'·5' 게이트
통과) 확립 전에는 활성화하지 않는다.**

### 3.3 조향 권한 단일화의 구현 (C3) — 기구현 vy 주입에서의 이관 설계 (v3.1 정정)

⚠ v3.1 정정: P5는 계획이 아니라 **기구현 상태**다. 소셜/양보 레이어는
`safety_stop_node` 내부에 vy 크랩 주입으로 구현·튜닝 완료됐고 (P5a/5b/P5-4), Track N
v1/v2 시뮬 통합에서 **Nav2와 vy 주입이라는 두 횡방향 제어기가 실제로 충돌해 nav
모드에서 주입을 껐다** — 이 실측이 C3의 방향을 정당화한다. 문제는 v2가 우려한 "두 번
만들기"가 아니라, **이미 vy 쪽에 쌓인 검증된 로직(래치·회전 차단·월드 앵커 목표)을
잃지 않고 goal 구조로 옮기는 것**이다 (보존 목록은 §6.7).

v3는 **인터페이스를 먼저 고정**한다:

```
Phase 5' (Nav2 없음):
  social_gap_node → /social/goal (PoseStamped) → goal_follower(얇음) → cmd_vel

Track N (Nav2 도입):
  social_gap_node → /social/goal → [BT 노드가 소비] → MPPI → cmd_vel
                                   goal_follower 삭제
```

`goal_follower`는 의도적으로 얇게 만든다 (goal 방향으로 P 제어 + 속도 캡, 100줄 이내).
**버릴 코드임을 처음부터 전제한다.** `social_gap_node`의 판단 로직·히스테리시스·워치독은
goal 생산 쪽에 있으므로 그대로 살아남는다.

이 설계의 이점: Track N을 앞당기지 않아도 되므로 **v2의 "Nav2는 sim2real을 막지
않는다" 원칙을 지킬 수 있다.**

### 3.4 Track N 대비 사전 결정 사항 (C8)

Track N 진입 시 재논의하지 않도록 지금 확정한다.

| 항목 | 결정 |
|---|---|
| fast-class 예측 궤적 → costmap 투영 | **금지.** 게이트 전담 (원칙 5) |
| pedestrian-class 예측 → costmap | 허용. 단 lethal 아니라 **시간 감쇠 cost** |
| 조향 담당 | MPPI (`OmniMotionModel`, `vy_max ≪ vx_max`) |
| YIELD 담당 | BT 서브트리 + `/social/goal` |
| 최종 게이트 | `safety_stop_node` 유지 → Collision Monitor와 A/B 후 결정 |
| footprint | Collision Monitor `VelocityPolygon` (속도 구간별, 측정 9D 값) |
| 다중 cmd 소스 | twist_mux + e-stop lock (v2 계획 유지) |
| 동적 장애물 critic | Phase N3 상당 — 시간 정렬 비용. **방법 A(시간축 없는 투영) 건너뜀** |

---

## §4. 파라미터 대장 (C7)

정본은 기계가 읽는 `config/params_ledger.yaml` (별도 파일). 이 절은 사람이 읽는 요약이다.

### 4.1 상태 정의

| status | 의미 | 사용 가능성 |
|---|---|---|
| `CONFIRMED` | 근거가 갖춰지고 검증됨 | 사용 가능 |
| `PENDING` | 유도식은 있으나 입력값 미측정 | 실기체 사용 금지 (원칙 9) |
| `INVALID` | 근거가 무효화됨 (센서/환경 변경) | 사용 금지, 재유도 필요 |
| `PROVISIONAL` | 카탈로그값으로 임시 채움, 측정으로 대체 예정 | 시뮬만 |

### 4.2 근거 층위 (원칙 8)

```
① 문헌 앵커        기존 연구·표준이 제시하는 값/범위
② 물리·기하 유도   로봇 치수에서 계산
③ 시뮬 실측 보정   시뮬 관측으로 결정 (문헌 근거 없음)
④ 센서 사양 의존   센서 스펙에 의존 — 세대 변경 시 전수 재계산  ← v3 신설
```

### 4.3 ④ 센서 사양 의존 항목 (전부 재유도 대상)

| 파라미터 | v2 값 | v3 status | 유도식 / 의존 측정 |
|---|---|---|---|
| `T_frame` | 0.10 s | **PROVISIONAL** 0.18 s | `1 / f_scan` · 측정 6A |
| `T_perception` | 0.015 s | PROVISIONAL | 측정 3 (풀스택 동거 조건) |
| `T_transport` | 0.005 s | **PENDING** | 측정 3 (경로별 p95) |
| `T_actuation` | (미분리) | **PENDING** | 측정 9A (스텝 응답 데드타임 L) |
| `T_total` | 0.15 s | **PENDING** | `N_fast·T_frame + T_perception + T_transport + T_actuation` |
| `ROI_fast` | 15 m | **INVALID** | `v_rel × (stop_ttc + T_total)`, 구속: `≤ R_confirm_p10` |
| `R_confirm_p10` | — | **PENDING** | 측정 7B (저반사율 자전거 실루엣) |
| `v_robot_max` | 0.4 m/s (암묵) | **PENDING** | `R_confirm_p10/(stop_ttc+T_total) − v_bike` |
| `stop_ttc` | 2.0 s | **PROVISIONAL** | ① ISO 22839 관례. 위 구속 위반 시 재협상. ⚠ v3.1: 코드 키는 `fast_stop_ttc`(코리도 `stop_ttc`는 1.0으로 별개) |
| `self_filter.*` | L1 기하 | **INVALID** | 실기 근거리 유입 (세대 무관, §1.2) → 전면 재작성 · 측정 5 |
| `emergency_clearance` | 0.60 m | **INVALID** (v3.1) | `min_range − half_length + margin` (yaml 주석 명시 유도식) · 측정 5 |
| `blind_hold_clearance` | 0.60 m | **INVALID** (v3.1) | 동일 유도식. min_range 0.05 확정 시 blind-hold 메커니즘 존속 자체 재검토 |
| ToF 카메라 사양 | — | **PENDING** (v3.1) | 선정 대기 (D435i 미사용 확정). Phase 6' 규모·근접장 담당 범위 판정의 입력 (§5.4, §9.4) |
| `roi_z_max`, 지면 정책 임계 | 0.10 / 0.30 / 0.12 m | **UNVERIFIED** | 측정 8B/8C |
| `degrade_profile: real_l1` | — | **INVALID** | → `real_l2` · 측정 5, 6A |

### 4.4 ③ 시뮬 실측 보정 항목 (재보정 필요)

| 파라미터 | v2 값 | v3 status | 사유 |
|---|---|---|---|
| `N_fast` | 4 (클래스 무구분) | **PENDING** | C2 분리. 측정 6C (오발률 트레이드오프) |
| `N_ped` | 4 | PROVISIONAL 4 | 유지. 재확인만 |
| 예측 반경 팽창 | `0.4 m/s · t` 상수 | **INVALID** | C1 → CP 분위수로 교체 |
| `cp_quantile[class][horizon]` | — | **PENDING** | 시뮬 GT 잔차로 1차 산출 → 실기체 재캘리브레이션 |
| 양보 진입 접근속도 창 | (−2.0, −0.5) | **PENDING** | 클러터 노이즈가 L2 밀도에서 변화. 측정 6C 병행 |
| 틈 시도 하한 | 0.7 → 0.8 m | **PENDING** | C5 비등방 footprint 도입으로 **재하향 가능** |
| 기동 커밋 히스테리시스 | 0.15 / hold 3 s | PROVISIONAL | goal 생산 쪽으로 이전 (C3). 값은 유지 |
| 양보 워치독 | 8 s / 이동 시한 10 s | CONFIRMED | 센서 무관 |
| 사람 인접 속도 캡 반경 | 2.5 m | PROVISIONAL | 유지 |
| `a_brake` | 2~3 m/s² 추정 | **PENDING** | 측정 9A + P8 제동 실측 |

### 4.5 ①② 항목 (대체로 유지)

| 파라미터 | 값 | status | 비고 |
|---|---|---|---|
| 고속 경계 \|v\|>2.0 m/s | 2.0 | CONFIRMED | ① Bohannon '97 + RSS. **단 단일 조건 → 다변수화** (§9.2) |
| 사람 측면 hard / comfort | 0.45 / 0.6 m | CONFIRMED | ① proxemics. **정당화는 SSM 참조 구조로 보강** (§7) |
| 로봇 몸통 반폭 | 0.155 m | CONFIRMED | ② 실측 |
| 다리 스윙 실효폭 | 0.45~0.5 m | PROVISIONAL | ② + ③. 측정 9D로 실기체 확정, 축별 분리 |
| footprint 반장축 `a_x` | (신규) | **PENDING** | ② Go2 길이 0.70 m 기준 ≈ 0.55~0.62 추정. 측정 9D |
| footprint 반단축 `a_y(v)` | (신규) | **PENDING** | ② + 측정 9D (vx/vy/wz별) |
| TTC 감속/정지 4 s / 2 s | 4 / 2 | PROVISIONAL | ① ISO 22839. §1.4 구속 하에 재검토 |
| 폐색 팬텀 속도 | 1.5 / 2.0 m/s | CONFIRMED | ① RSS Rule 4 |
| 하강 단차 STOP | ≥0.12 m | UNVERIFIED | ② Go2 등판 한계. **검출 가능성 미확인** (측정 8C) |

> 📌 v2 근거 문서와 v3 계획서 사이에 **AEB 표준 인용 불일치**가 있다: 근거 문서는
> ISO 15623(FCW), 계획서는 ISO 22839(AEB)를 인용한다. 둘 다 차량 표준이고 4~2 s
> 대역 관례라는 결론은 같으나, **정본을 ISO 22839로 통일**하고 도메인 부적합성을
> ISO 4448 계열로 보완 표기한다 (§7).

---

## §5. 인터페이스 변경

### 5.1 `TrackedObstacleArray` 신설 (C6)

`Detection3DArray`는 **유지**한다 (게이트가 두 소스를 동일 로직으로 소비하는 v2 계약을
깨지 않기 위해). 예측·불확실성이 필요한 소비자를 위해 병행 토픽을 추가한다.

```
/obstacles          Detection3DArray        (유지 — 게이트, ground_obstacle_node 공통 계약)
/obstacles/tracked  TrackedObstacleArray    (신규)
/prediction/tubes   PredictionTubeArray     (신규)
```

`TrackedObstacle` 필드는 `nav2_dynamic_msgs/Obstacle`에 **정렬**한다 (uuid, position,
velocity, size 등). 그 위에 우리 필요분만 확장:

```
TrackedObstacle
  # nav2_dynamic_msgs/Obstacle 정렬분
  uuid, position, velocity, size, score
  float64[9] position_covariance   # v3.1: 원본 msg에 이미 존재 → 정렬분으로 이동
  float64[9] velocity_covariance   #   track quality 게이팅용 (예측 반경용 아님)
  # v3 확장
  string   obstacle_class          # person / bicycle / animal / unknown / static
  float64  class_confidence
  builtin_interfaces/Duration track_age
  uint32   consecutive_hits        # N 확정용 — 측정 6C 가 이 필드를 소비
  bool     is_fast_class
```

> ⚠ **covariance는 예측 반경 산출에 쓰지 않는다.** KF 공분산은 가정한 모델 내부의
> 값이므로 모델 오정합(직진 가정 CVM에 회전하는 자전거) 하에서 실제 오차를 체계적으로
> 과소평가한다. 예측 반경은 CP 분위수(C1)가 담당하고, covariance는 **트랙 품질
> 게이팅**(신뢰도 낮은 트랙 배제)에만 쓴다.

`nav2_dynamic_msgs`는 릴리스가 없는 연구 저장소이므로(커밋 44개, 바이너리 없음)
**의존하지 않고 필드 정의만 차용**한다. 나중에 `kf_hungarian_tracker`나 커뮤니티 도구를
가져올 때 어댑터가 얇아진다.

### 5.2 `/social/goal` 신설 (C3)

```
/social/decision  std_msgs/String        # PASS / YIELD_MOVE / YIELD_WAIT / WAIT
/social/goal      geometry_msgs/PoseStamped
                  # YIELD_MOVE 시 가장자리 목표점
                  # PASS 시 gap 중앙 통과점
                  # 프레임: odom (커밋 후 재계산 금지 — 히스테리시스가 여기 적용)
```

`goal_follower`(임시)와 BT 노드(Track N)가 이 토픽의 두 소비자다. **동일 토픽·타입이므로
스왑은 launch 한 줄** — v2가 `sidewalk_polygon_node`에서 이미 쓴 패턴 그대로.

### 5.3 `/vendor_avoid_state` 신설 (C9)

```
/vendor_avoid_state  std_msgs/Bool   # ObstaclesAvoidClient.SwitchGet() 결과, 1 Hz
```

`collision_oracle_node`가 이 값을 CSV 열로 기록한다. True로 관측된 구간이 있는 실험은
**무효 처리**한다.

### 5.4 `docs/interfaces.md` 갱신 필요 항목

팀원 계약 문서이므로 다음을 반영해야 한다:

- §3 depth estimation 계약: 32FC1+CameraInfo 계약은 유지하되 실기 센서는 **D435i가
  아니라 ToF 카메라** (v3.1 — 2026-07-10 확정, 스펙 미정 → 대장 PENDING). 단
  **L2가 0.05 m부터 보므로 근접장 담당 범위가 축소될 수 있음**을 주석 (측정 8 대기)
- segmentation → polygon 토픽 계약: 변경 없음
- `TrackedObstacleArray` 정의 추가 (팀원 YOLO 노드가 class를 여기에 채울 수 있도록)

---

## §6. 코드 수정 목록 (하드웨어·계획 변경 반영)

> 🔴 = 실기체 기동 전 필수 · 🟠 = Phase 4 진입 전 필수 · 🟡 = 해당 Phase에서

### 6.1 🔴 `config/self_filter.yaml` — 전면 재작성 (H)

**사유**: 실기 근거리 유입 — 시뮬 URDF min_range 0.8이 가려온 세대 무관 리스크
(§1.2, v3.1 정정)

```
변경:
  · 자기 몸통 배제 영역: L2 원점 기준 3D 박스로 재정의
    (몸통 0.70×0.31, 마운트/배터리 돌출 포함)
  · 다리 스윙 배제 영역: 속도·gait 의존 원통 또는 4개 부채꼴
    → 측정 9D 실효폭을 입력으로 사용
  · 근접 임계 min_range: 0.8 → 0.05 (또는 self-filter 통과 후 0.05 유지)
검증:
  · 정지 상태 30 s: self-filter 후 잔여 점 0
  · 보행 중(0.3/0.6/1.0 m/s) 30 s: 자기 유래 클러스터 0
  · 이 게이트를 통과하지 못하면 다른 어떤 측정도 진행 불가
```

### 6.2 🔴 `lidar_obstacle_node.py` — 대폭 수정

| 변경 | 사유 | 우선 |
|---|---|---|
| ROI 재계산 (`ROI_fast` 대장 참조로 변경, 하드코딩 제거) | H §1.4 | 🔴 |
| 프레임 누적 로직 재작성 — 5.55 Hz 기준, 누적창을 파라미터화 | H §1.1 | 🔴 |
| `consecutive_hits` 필드 유지 + **프레임별 조건 만족 이력 로깅** | 측정 6C 전제 | 🔴 |
| `N`을 클래스별(`N_fast`/`N_ped`)로 분리 | C2 | 🟠 |
| covariance·confidence 산출 → `TrackedObstacleArray` 병행 퍼블리시 | C6 | 🟠 |
| 점군 밀도 불균일(중심부 고밀도) 반영한 클러스터 최소 점수 조건 | H §1.1 | 🟠 |
| latency 보상 상한 재검토 — v3.1: 코드는 이미 메시지별 실측 지연 + `cvm_latency_max` 0.3 상한("단일 상수 0.15 제거"는 낡은 서술). 상한을 `T_total` 분해 실측으로 재유도 | H §1.4 | 🟠 |

### 6.3 🔴 `config/safety_stop.yaml` — 스키마 변경

```
변경 전:  stop_ttc: 2.0
          cpa_expansion_rate: 0.4
          roi_max: 15.0

변경 후:  stop_ttc:
            value: 2.0
            status: PROVISIONAL
            basis_tier: 1
            source: "ISO 22839 AEB 관례"
          roi_fast:
            value: null
            status: PENDING
            basis_tier: 4
            sensor_generation: L2
            formula: "v_rel * (stop_ttc + T_total)"
            constraint: "<= R_confirm_p10"
            depends_on: [measurement_03, measurement_05, measurement_06, measurement_07, measurement_09]
          # cpa_expansion_rate 삭제 → cp_quantile 테이블 참조로 대체
```

pytest의 YAML↔기본값 정합 테스트(v2 Phase 3)에 **status 필드 검사**를 추가한다.

### 6.4 🟠 `safety_stop_node.py` — 게이트 로직 수정

| 변경 | 사유 |
|---|---|
| 예측 반경 `0.45 + 0.4·t` → `footprint_halfwidth(v) + cp_quantile[class][t]` | C1, C5 |
| 클래스별 `N` 적용 | C2 |
| 속도 의존 비등방 footprint 조회 (`a_x`, `a_y(vx,vy,wz)`) | C5 |
| PENDING 파라미터 가드: 실기체 모드에서 PENDING 사용 시 기동 거부 | 원칙 9 |
| `YIELD_MOVE` 상태에서 **vy 생성 제거** → `/social/goal` 도달 여부만 판정 | C3 |
| `T_total` 분해값으로 latency 보상 | H §1.4 |
| fast-class는 costmap/투영 경로를 타지 않음을 assert로 고정 | C8, 원칙 5 |

### 6.5 🔴 `degrade_pointcloud_node.py` + `verification/profiles/`

**사유**: `real_l1` 프로파일이 L1 특성(21,600 pts/s, 11 Hz, 0.8 m 블라인드) 기준 (H)

```
신규 프로파일 real_l2:
  · 포인트율      측정 5 실측값 (64,000 또는 21,600)
  · 수평 회전     5.55 Hz (측정 6A 로 확정)
  · 비반복 스캔   L2 패턴 — 프레임 간 방위각 위치가 다름
  · 근접 유입     0.05 m 부터 반환 (자기 유래 점 포함 모드 옵션)
  · 반사율 의존   저반사율 표적의 거리별 dropout 곡선 (측정 7A)
  · 밀도 불균일   FOV 영역별 가중 (중심부 고밀도)
  · 지연          측정 3 실측 p95

real_l1 은 삭제하지 않고 status: DEPRECATED 로 남긴다 (v2 회귀 결과 재현용).
worst_case 프로파일도 L2 기준으로 재작성.
```

### 6.6 🔴 `go2_simulation` 라이다 센서 설정 + 회귀 재기준화

```
변경: gz-sim lidar 플러그인 파라미터
  update_rate      10   → 5.55
  vertical fov     90°  → 90° (기본) / 96° (negative angle 모드 실험용 별도 프로파일)
  min_range        0.8  → 0.05
  samples          L2 포인트율에 맞춰 재계산
  (비반복 스캔은 gz-sim 기본 라이다로 표현 불가 → degrade 노드가 담당)

⚠ RTF 운영점 재유도 (v3.1): GPU 라이다 벽시계 상한 ~2.5 Hz(월드 무관 실측)·현행
  test 월드 RTF 0.25 캡 위에서 5.55 Hz 설정이 심시간 프레임 주기와 정합하는지
  재확인 — sensor_timeout·프레임 누적 창이 이 운영점에 튜닝돼 있다

파급: Phase 1 / 2 / 2.5 / 3 / 4 / 5 게이트 전부 재실행 (§1.3)
      → Phase 3.5 의 주 작업
```

### 6.7 🟡 `social_gap_node.py` — 기구현 로직의 추출·이관 (v3.1 정정)

~~구현 전이므로 수정 비용 0.~~ **P5는 `safety_stop_node` 내부에 vy 주입으로
구현·튜닝 완료 상태다** (P5a/5b/P5-4 게이트 통과). 따라서 이 작업은 신규 작성이
아니라 **검증된 판단 로직을 추출해 goal 생산으로 이관**하는 마이그레이션이다.

```
출력:  /social/decision (String), /social/goal (PoseStamped)
이관:  gap 판단(social_nav.py 순수 모듈은 그대로), 우측 통과 관습, 정적 밀착,
       히스테리시스 0.15/hold 3 s, 양보 워치독 8 s / 이동 시한 10 s
금지:  vy / wz 직접 산출
보존해야 할 튜닝 불변식 (이관 시 소실 금지 — 최근 커밋들의 산물):
  · 양보 래치 요 불변화 + 월드 앵커 목표 단조화 (6ee19b2, 3090672)
  · 도달 판정 = 월드 y 실변위 증거 — 회전 통과 오판 차단 (c0cd11c)
  · 통과 판정 긍정 증거화 — 전방 소실 3 s 유예 (964a751)
  · 발동 시간 기준화 x/closing ≤ 7 s (37582d8)
  · lane_clear(막혔을 때만 조향) · cut-in lock · edge|edge gap 커밋 금지
검증:  이관 전후 P5 게이트 회귀 동등성 (동일 시나리오 세트 PASS 유지)
```

### 6.8 🟡 `goal_follower.py` — 신규, 의도적으로 얇게 (P5')

```
100줄 이내. goal 방향 P 제어 + 속도 캡 + goal 도달 판정.
Track N 진입 시 삭제 예정임을 파일 헤더에 명시.
여기에 로직을 추가하려는 충동이 들면 social_gap_node 로 올린다.
```

### 6.9 🟡 `prediction_node.py` — 신규 (C1)

```
입력:  /obstacles/tracked
처리:  CVM 전방전파(T_total) + CP 분위수 조회 → 클래스별·지평별 반경
출력:  /prediction/tubes
부속:  scripts/cp_calibration.py
         · 시뮬: GT vs CVM 잔차를 지평 bin 으로 수집 → 분위수 산출
           (v3.1: 1차는 클래스 통합 pooled — 클래스 조건부는 bin당 표본 기준
            충족 시에만, §7.4 단서)
         · 실기체: P9 bag 으로 재캘리브레이션 (델타를 기록 = 열화 하네스 유효성 증거)
         · 산출물: config/cp_quantiles.yaml (status 필드 포함)
```

### 6.10 🟡 `ground_obstacle_node.py` (P6 미구현) — 존속 여부 조건부

```
v2 목적 2개 중:
  ① L1 근거리 블라인드(0.8 m) 보완  → 🟢 목적 소멸 — 단 v3.1 정정: 0.8 m는 URDF
     값이었고 L1 실기도 0.05 m (스펙 오인 기반 목적, §1.2). 측정 5·8로 확정
  ② L1 지면 감지 하한 0.3 m 보완 (하강 단차 포함) → ❓ 측정 8B/8C 결과에 달림

분기:
  8C 낙차 검출 가능      → Phase 6 축소 (하강 단차 STOP만 L2 기반으로 게이트에 편입)
  8C 낙차 검출 불가      → Phase 6 존속 + 우선순위 상승 (§9.4)
  8B 0.10 m 상승 미검출  → Phase 6 존속
```

### 6.11 🔴 `go2_real_bringup` (P8 미구현) — 신규 항목 추가

```
신규 노드 go2_vendor_avoid_guard.py:
  · 기동 시 ObstaclesAvoidClient.SwitchSet(False) → SwitchGet() 검증
  · 30 s 주기 재확인 (주기는 측정 1 Step4 결과로 확정)
  · /vendor_avoid_state 퍼블리시
  · True 관측 시 safety_stop ESTOP 트리거 + bag 무효 플래그

sport_cmd_adapter.py:
  · SportClient 만 사용. ObstaclesAvoidClient import 금지
  · 축간 커플링 보상 (측정 9C 결과에 따라)
  · vy 제한: vy_max = vx_max × (K_vy/K_vx)  ← 측정 9A

proprio_safety_node.py (v3.1 신규 — 고유수용성 안전 입력):
  · /lowstate 족저력 + IMU 자세·충격 이상 (전도/슬립/비정상 접촉) → ESTOP 트리거
  · 외수용(라이다) 게이트와 상보 — 연석 낙하·미끄러짐은 충돌만큼 안전 치명적인데
    현행 게이트는 외수용 전용. sport 모드에서도 /lowstate·IMU 는 수신 가능
  · P8' 벤치에서 임계 실측 (오발률 게이트 포함)

CI 게이트 (신규 pytest):
  · grep 으로 ObstaclesAvoidClient 사용 검출 → guard 노드 외에서 발견되면 FAIL
```

### 6.12 🟡 `worlds.py` / 시나리오 — 신규 월드 요소

```
측정용 월드 (verification/measurement_worlds/):
  · 반사율 표적 3종 (90/40/10 %) × 실루엣 2종 (보행자/자전거)  ← 측정 7
  · 계단 블록 세트 0.05/0.10/0.15/0.20/0.30 m × 거리 7단        ← 측정 8B
  · 인공 낙차 0.12/0.20/0.30 m                                   ← 측정 8C
  · 방위각 재방문 측정용 좁은 폴 (0.2 m)                          ← 측정 6A
  · 클러터 오발률 월드 (주차 자전거·화분·벤치·벽 요철)            ← 측정 6C

기존 월드: L2 센서 설정으로 재실행 (내용 변경 없음)
정지 보행자 = static 모델 유지 (퇴화 actor 가드 유지)
```

### 6.13 🟡 `collision_oracle_node.py` — 소폭

```
CSV 열 추가:  pending_params (list), vendor_avoid_state (bool), sensor_generation (str)
알고리즘 영향 없음 (평가 전용 유지)
```

### 6.14 신규 파일 총괄

```
config/params_ledger.yaml          파라미터 대장 (정본, 기계 판독)
config/cp_quantiles.yaml           CP 분위수 테이블
perception_avoidance/
  prediction_node.py               C1
  goal_follower.py                 C3 (임시)
  cbf_filter_node.py               C4 (P4 선택 항목)
scripts/cp_calibration.py          CP 분위수 산출
go2_real_bringup/
  go2_vendor_avoid_guard.py        C9
measurement/                       측정 프로토콜 스크립트 9종 (§8)
  m01_vendor_avoid.py  m03_transport.py  m04_mppi_throughput.py
  m05_point_rate.py    m06_revisit.py    m07_track_range.py
  m08_fov_ground.py    m09_step_response.py  common.py
docs/measurement_protocol.md       측정 절차 정본
```

---

## §7. 근거 추적표 — 어떤 논문의 어떤 아이디어를 썼는가

`채택` = 알고리즘에 직접 반영 · `참조` = 설계 정당화/로드맵 후단 · `배제` = 검토 후 불채택

### 7.1 v2에서 유지 (재확인)

| 출처 | 채택한 아이디어 | 적용 위치 | 수준 |
|---|---|---|---|
| Trautman & Krause, *Unfreezing the Robot*, IROS 2010 / IJRR 2015 | 밀집 군중에서 과잉 조심으로 멈추는 것 자체가 위험. 의도가 가시적인 회피 행동이 상호 협조를 유도 | `social_gap_node` YIELD_MOVE→WAIT 설계, anti-freeze 게이트 | 채택 |
| Schöller et al., *What the CVM Can Teach Us*, RA-L 2020 | 1~5 s 지평에서 등속 모델이 Social-LSTM/GAN과 대등 | `prediction_node` 기본 예측기 = CVM | 채택 |
| Falanga, Kim, Scaramuzza, *How Fast Is Too Fast?*, RA-L 2019 | 인지 지연이 안전 속도 상한을 직접 결정. 지연 중 상대가 이동해 회피 창이 소멸 | 원칙 5 (swerve 금지), `T_total` 유도식, `v_robot_max` 구속 | 채택 |
| Fiorini & Shiller, *Motion Planning Using Velocity Obstacles*, IJRR 1998 | 이동 장애물은 속도 공간에서 추론. 최근접점(CPA) 프레임 | `safety_stop_node` CPA 미스거리 트리거 | 채택 |
| Shalev-Shwartz et al., *RSS*, arXiv:1708.06374, 2017 | Rule 4: 폐색 뒤 팬텀 에이전트 가정 → 속도 상한. Rule 5: 돌발 출현 = 긴급정지, 조향은 측면 안전 확인 시만 | `occlusion_speed_node`, 원칙 5 | 채택 |
| Hall, *The Hidden Dimension*, 1966 / Kirby, PhD CMU 2010 / Pacchierotti et al., RO-MAN 2006 | 친밀권 경계 ≈0.45 m, 통과 여유 0.4~0.6 m, 사회적 비용 함수 구성 | 사람 측면 hard 0.45 / comfort 0.6, gap 임계 합성 | 채택 |
| Molnar et al., RA-L 2022 (Unitree A1 실증) | `cmd_vel` 레벨 unicycle CBF가 4족에서 실용적 | `cbf_filter_node` (계층 2) | 채택 |
| ISO 13482 | protective stop, 근접 시 속도 제한 | 원칙 4 (속도 = 근접도의 단조 비증가) + 래치 | 채택 |
| ISO 22839 (AEB) | 경고 ~4 s / 개입 ~2 s 시간 척도 | `slow_ttc` 4 s, `stop_ttc` 2 s | 채택 (단 §7.4 단서) |
| UL 4600 | 시나리오별 증거 기반 안전 논증 | phase-gate CSV 산출물 의무화 | 채택 |
| Rosén & Sander, AAP 2009 | 피해 위험은 충돌 속도에 단조·급경사 증가 → 감속은 판단이 틀려도 손해가 없는(no-regret) 조치 | SLOW→STOP 2단 구조의 정당화 | 채택 |
| CMU `autonomy_stack_go2` | L1 지면 감지 하한 0.3 m 문서화, WiFi 지연 경고 | Phase 6 존재 근거, Ethernet 고정 | **부분 무효** (§7.5) |
| ANYmal perceptive locomotion, Science Robotics 2022 | foothold 계획은 저수준 제어 접근 필요 | 범위 외 확정 (유지) | 참조 |
| Bohannon, *Age and Ageing*, 1997 | 성인 선호 보행속도 1.2~1.4 m/s, 빠른 보행자 ~1.5 | 고속 경계 2.0 m/s의 하한 근거 | 채택 |

### 7.2 v3 신규 채택

| 출처 | 채택한 아이디어 | 적용 위치 | v2 대비 |
|---|---|---|---|
| **Lindemann, Cleaveland, Shim, Pappas**, *Safe Planning in Dynamic Environments using Conformal Prediction*, RA-L 8(8):5116, 2023 | 예측 궤적과 함께 **불확실성을 정량화한 예측 영역**을 conformal prediction으로 얻어 MPC에 확률적 안전 보장을 부여. 오프라인 궤적 데이터만 있으면 성립 | **C1**: `0.4 m/s·t` 고정상수 → `cp_quantile[class][horizon]`. `prediction_node` + `cp_calibration.py` | **교체** |
| **Lin, Peng, Bansal**, *One Filter to Deploy Them All*, arXiv:2412.09989 (2024) / IEEE T-RO 42:545–560 | LiDAR 관측으로 안전 영역을 동적 구성하고 외란 추정으로 미모델 동역학을 반영. **명목 4족 제어기를 사전 지식 없이 필요할 때만 오버라이드**. Go1 실기체 검증 | **C4**: 계층 2를 정식화. locomotion 정책을 건드리지 않고 필터만 얹는 구조 | **신규** |
| **Sankaranarayanan, Viswanathan, Saradagi, Satpute, Nikolakopoulos**, *Reactive Robot-Centric Safety for Autonomous Navigation in Constrained and Dynamic Environments*, arXiv:2605.15782 (Spot 4족 실증) | 안전 영역을 **몸통 프레임 축정렬 타원**으로 정의 → 등방 거리 제약보다 덜 보수적·방향 구분. 등방 0.8 m 제약은 통과 가능한 문틈을 안전영역 중첩으로 막아 **데드락 유발**을 실증. 로봇 회전이 세계 프레임에서 시간가변 제약을 유도 | **C5**: 비등방 footprint. 원칙 5의 "예측 투영이 데드락을 만든다" 논거. 틈 하한 0.8 m 재하향 가능성의 근거 | **신규** (단 §7.4 단서) |
| **Molnar & Ames**, *Composing Control Barrier Functions for Complex Safety Specifications*, IEEE L-CSS 7:3615, 2023 | log-sum-exp soft-min으로 다수 점별 제약을 단일 매끄러운 배리어로 합성 → 수백~천 개 제약을 제어 주기로 처리 | `cbf_filter_node` 구현 방식 (계층 2) | 신규 |
| **Sathyamoorthy, Patel, Guan, Manocha**, *Frozone*, RA-L 5(3):4352, 2020 | freezing-free + 보행자 친화 내비게이션 | Trautman의 현대 앵커. anti-freeze 게이트 근거 보강 | 신규 |
| **ISO/TS 15066 + ISO 10218-1/2:2025 Annex L** (SSM) | 보호 이격거리 = 사람 지향속도 항 + 로봇 정지거리 항 + **측정 불확실성·시스템 지연·제어 주기**를 모두 포함. 현재 이격거리에서 최대 안전속도를 역산 | 사람 측면 0.45 m와 CPA 팽창항을 **공학적 보호거리 모델의 참조 구조**로 재정당화. `v_robot_max` 역산 구조의 원형 | 신규 (§7.4 단서) |
| **ISO/TR 4448-1:2024 · ISO/WD TS 4448-16** | PMR(공공영역 이동로봇) 정의가 **"바퀴 또는 다리(보행) 기반"으로 4족을 명시 포함**. Part 16이 보도 로봇 안전·신뢰성을 다룸 | 도메인 표준 앵커. ISO 22839(차량)의 도메인 부적합을 보완 | 신규 |
| **Nav2 Collision Monitor 문서** (`VelocityPolygon`, Stop/Slowdown/Limit/Approach) | 명령 속도에 따라 보호 폴리곤을 전환. Approach 모델은 현재 속도로 TTC를 추정해 항상 M초 여유 유지. costmap·플래너를 우회한 센서 직결 비상정지 | **C5 구현 수단** + 계층 3 A/B 비교 대상. Approach 모델이 v2 게이트와 개념적으로 동형임을 확인 | 신규 |
| **`ros-navigation/navigation2_dynamic`** (`nav2_dynamic_msgs`, `kf_hungarian_tracker`) | 동적 장애물 추적 메시지 스키마 (uuid/position/velocity/size) + KF+Hungarian 추적 | **C6** 필드 정의 차용 (의존은 하지 않음 — 릴리스 없음) | 신규 |

### 7.3 참조 (로드맵 후단 / 설계 정당화)

| 출처 | 아이디어 | 왜 지금 채택하지 않는가 |
|---|---|---|
| **Samavi, Han, Shkurti, Schoellig**, *SICNav*, T-RO 41:801, 2025 / *SICNav-Diffusion*, RA-L 2025 / *Deploying SICNav in the Field*, arXiv:2506.08851 | 보행자가 ORCA를 따른다고 모델링해 그 모델을 로봇 MPC의 제약으로 심는 bilevel 최적화 → **로봇 계획과 사람 예측이 구성적으로 충돌 없음**. 필드 7 km/2 h 검증 | 우리 YIELD FSM은 이 결합형의 **규칙 기반 근사**임을 명시. 실시간 solver(Acados)·예측 모델 복잡도가 Jetson 예산 미검증. **Track N 이후 비교 대상** |
| **DR-MPC**, RA-L 2025 | 명목 MPC 경로추종 위에 실세계 군중 데이터로 학습한 residual policy를 얹음 → end-to-end RL보다 해석 가능·안정 | "규칙 기반 명목 + 보정" 구조의 문헌 정당화로만 사용. 실제 사람 데이터 필요 |
| Dixit et al., L4DC 2023 (adaptive CP) / Yao et al., arXiv:2508.05634 | 온라인 적응 CP, 불확실성 추정을 관측에 덧붙여 제약 RL | C1의 3차 개선 경로. 1차는 split CP로 충분 |
| Strawn, Ayanian, Lindemann, RA-L 2023 | conformal predictive safety filter | 계층 2의 대안 형태 |
| **HuNavSim**, RA-L 8(11):7130, 2023 | SFM 기반 보행자 + 로봇 존재에 대한 반응 세트 + 사회적 내비게이션 지표 | ⚠ **래퍼가 Gazebo Classic 11 / Fortress / Webots 3종뿐 — Harmonic 래퍼 없음.** Phase 5' 검증 시나리오는 스크립트 궤적 actor로 충분하므로 포팅은 후순위 |
| **Arena 4.0**, ICRA 2025 | 25종 이상 SOTA 플래너 + 표준 지표 평가 파이프라인. 플래너를 nav2 플러그인으로 통합하는 래퍼 제공 | Track N에서 우리 구성을 nav2 플러그인으로 만들면 추가 비용 거의 없이 외부 비교 가능 |
| arXiv:2504.19193 (Nav2 controller plugin + Mahalanobis 제약 MPC) | 장애물 미래 위치를 정규분포로 표현, Mahalanobis 거리 기반 확률 제약, Nav2 스택 무수정 통합 | ⚠ **한계 확인 필수**: 추적을 구현하지 않고 시뮬 내 로봇 간 위치 통신으로 대체 · IPOPT 450 ms 상한으로 **2 Hz**(데스크톱 5800X) · 로봇·장애물을 원형 근사(r+r_d=1.5 m) · 정적 장애물 미처리 · 실기체 없음. **개념 참조만** |

### 7.4 채택 시 명시해야 하는 단서

| 출처 | 단서 |
|---|---|
| Sankaranarayanan '26 (타원 CBF) | **동적 예측기가 아니다.** 논문 본문이 "장애물은 정적이라 가정하며 time-varying의 출처는 로봇 회전"임을 명시. 실험의 동적 장애물은 **의자와 문**이지 5 m/s 자전거가 아니다. → 계층 3 필터 근거로만 쓰고 fast-class 근거로 쓰지 않는다. 또한 `a_x=0.9 / a_y=0.45`는 **Spot 치수·환경 튜닝값**이며 저자들도 넓은 복도에서 `a_y`를 0.6으로 바꿨다. Go2 값은 측정 9D로 새로 유도한다 |
| ISO/TS 15066 SSM | **규범적 적합성 표준이 아니다.** 스코프는 산업용 로봇이며 야외 보도 4족은 범위 밖. **공학적 보호거리 모델의 참조 구조**로만 활용한다. 도메인 표준은 ISO 4448 계열 |
| ISO 22839 / ISO 15623 | 차량 표준. 상대속도가 차량급이라는 이유로 시간 척도를 차용하지만, **PMR 도메인 표준(4448-16)이 확정되면 우선한다.** 근거 문서의 ISO 15623 표기는 ISO 22839로 통일 |
| One Filter to Deploy Them All | 서지 정보를 안전하게 표기: `arXiv:2412.09989 (2024); IEEE T-RO vol.42, pp.545–560 (early access 2025)`. 연도 단정 회피 |
| Nav2 최신 기능 | `model_delay_*`(Lyrical Luth), SpeedFilter path lookahead(Lyrical Luth), MPPI Eigen 재구현(Kilted)은 **Jazzy에 없다.** 자체 구현 유지 (§1.7) |
| Lindemann CP (v3.1) | 원법은 **지평별** 분위수(union bound) — 클래스×지평 분할은 우리 확장이며, 자전거 트랙 표본 부족 시 캘리브레이션 파편화로 분위수가 노이지·과보수화된다. **1차는 클래스 통합(pooled) 분위수**, 클래스 조건부는 bin당 표본 기준 충족 시. 또한 로봇이 보행자 행동에 영향을 주면 교환가능성 가정이 훼손된다 → 적응형 CP(ACI, §7.3 Dixit)를 3차가 아닌 **2차 개선으로 승격** |
| Nav2 VelocityPolygon (v3.1) | holonomic 모드는 vy를 독립 축이 아니라 **크기+방향 비닝**으로 처리하고, 미매칭 시 폴백 폴리곤 없이 마지막 매칭 폴리곤을 유지한다 — C5 구현 시 서브폴리곤이 명령 속도공간 전체를 덮도록 설계 |

### 7.5 배제 및 무효

| 항목 | 판정 | 사유 |
|---|---|---|
| DRL / Transformer / diffusion 예측기 | **배제 유지** | Schöller '20 근거. 연산량·데이터셋 편향·실외 일반화. v2 판단 유효 |
| KF covariance 기반 예측 반경 | **배제 (신규 판정)** | 모델 내부 값이라 모델 오정합 시 실제 오차를 체계적으로 과소평가. "통계적으로 유도됨"이라는 외양 때문에 고정상수보다 위험할 수 있음. → CP로 대체(C1), covariance는 트랙 품질 게이팅만 |
| fast-class 예측을 costmap에 시간축 없이 투영 | **배제 (신규 판정)** | 5 m/s × 2 s = 10 m 벽 → 좁은 인도에서 자유공간 소멸 → 데드락. 현재 fast-class STOP 규칙이 더 낫다 |
| CMU `autonomy_stack_go2`의 "L1 지면 감지 하한 0.3 m" | **부분 무효** | L1 관측이며 L2 negative angle 모드(96°)로 완화 가능. 측정 8B/8C로 재확인 전까지 Phase 6 근거로 인용 금지 |
| `ObstaclesAvoidClient` 사용 | **금지** | 이중 회피. CI로 강제 (§6.11) |
| foothold 계획 | **범위 외 유지** | ANYmal 근거. v2 판단 유효 |

---

## §8. 측정 프로토콜과 진입 게이트

상세 절차는 `docs/measurement_protocol.md`. 여기서는 의존 관계와 산출 파라미터만 정리한다.

### 8.1 측정 목록

| # | 측정 | 하드웨어 | 산출 파라미터 |
|---|---|---|---|
| 1 | 벤더 회피 OFF 검증 (positive control 포함) | 실기체 | `vendor_avoid_bypass_confirmed`, `switch_reassert_period_s` |
| 2 | `ObstaclesAvoidClient` 금지 CI | — | (구현 항목) |
| 3 | 전송 경로 판정 (Jazzy 네이티브 / Humble 컨테이너 / WebRTC) | 실기체 | `T_transport` p95, 손실률, 지터 |
| 4 | Jetson MPPI 처리량 (전력 모드 고정, 풀스택 동거) | Jetson | `controller_frequency_achieved`, `batch_size`, `time_steps` |
| 5 | `/utlidar/cloud` 실제 포인트율 + **L2 타임스탬프 무결성** (v3.1: unilidar_sdk2에 "시간축이 실제 경과의 절반" 버그 사례 — stamp 기반 latency 보상과 측정 3을 직접 오염) | 실기체 | `points_per_sec`, `frame_hz`, `accumulation_window_ms`, `timestamp_integrity_ok` |
| 6 | 방위각 재방문 간격 (A) / 트랙 갱신 간격 (B) / N 대 오발률 (C) | 실기체 | `T_frame`, `N_fast`, `N_ped`, 양보 진입 창 |
| 7 | 반사율별 트랙 형성 거리 (A 정적 / B 동적 접근) | 실기체 | `R_detect`, `R_cluster`, **`R_confirm_p10`** |
| 8 | 수직 FOV 실측 (A) / 지면 상승 (B) / 낙차 (C) — 정지·보행 양쪽 | 실기체 | `negative_angle_available`, 지면 정책 임계, `dropoff_detectable` |
| 9 | 스텝 응답 (A) / 주파수 응답 (B) / 축간 커플링 (C) / 실효폭 (D) | 실기체 | `T_actuation`, `a_brake`, `vy_max`, `model_dt`, `a_x`/`a_y(v)` |

### 8.2 실행 순서 (의존성)

```
[하드웨어 없이]
  3 (경로 판정) ─┐   ← Go2 X DDS 권한 판정 포함 (§1.5). 최우선 리스크
  4 (MPPI 처리량)┤
                 │
[실기체 도착]     │
  6.1 self-filter 재작성 검증  ← §6.1. 이걸 통과하지 못하면 아래 전부 불가
  1 (벤더 회피 검증)  ← 최초 30분
       ↓
  5 (포인트율) ──┐
       ↓         │
  6 (재방문, N) ─┤
       ↓         │
  9 (응답 특성) ─┤   ← T_actuation 이 7 의 계산 입력
       ↓         │
  7 (트랙 형성) ←┘   ← 3,5,6,9 결과를 모두 소비
       ↓
  stop_ttc / v_robot_max / ROI_fast 확정 → 대장 PENDING 해제
       ↓
  8 (지면·낙차) — 병렬 가능. Phase 6' 존속 판정
```

**7번이 마지막인 이유**: 5(밀도)·6(확정 지연)·3(전송)·9(액추에이션)를 모두 입력으로
받아야 재유도식이 닫힌다. 순서를 바꾸면 두 번 측정해야 한다.

### 8.3 진입 게이트

| 게이트 | 조건 | 실패 시 |
|---|---|---|
| **G0** | self-filter 후 정지 30 s 잔여 점 0, 보행 중 자기 유래 클러스터 0 | 다른 모든 측정 중단 |
| **G1** | 벤더 회피 우회 확인 **또는** OFF 검증 완료 + 지속성 파악 | 실기체 자율 주행 금지 |
| **G2** | 전송 경로 확정, 지터 p95 < 50 ms, 손실 < 1 % (풀스택 동거) | Go2 X SKU 권한 문제 → 계획 재검토 (§1.5) |
| **G3** | MPPI가 풀스택 동거에서 ≥15 Hz, CPU 헤드룸 ≥25 % | Track N 컨트롤러 재선정 (Vector Pursuit 등) |
| **G4** | `v_robot_max ≥ 0.5 m/s` | 🔴 **아키텍처 분기** — §9.7 대안 B로 전환 |
| **G5** | 낙차 검출 가능 여부 확정 | 🔴 **아키텍처 분기** — Phase 6'을 앞으로 이동 |

**G4·G5만 결과에 따라 아키텍처가 바뀐다.** 나머지 게이트는 파라미터 값만 흔든다.

### 8.4 통계 규칙 (v2 검증 원칙에 추가)

- 임계값 산출 측정은 **n ≥ 10**, 보고는 **평균이 아니라 보수적 분위수**(p10/p90).
- 모든 측정에 함께 기록: 배터리 %, 조도(lux), 기온, 바닥 재질, gait 모드/자세 높이,
  전력 모드(Jetson), `sensor_generation`.
- 시계 동기: 가능하면 **Jetson 단일 클록 왕복**으로 측정. 불가한 항목은
  `ros2 topic delay`의 min을 baseline으로 먼저 추정. **잔차 > 20 ms면 측정 9 무효.**

---

## §9. 개정 로드맵

### 9.1 Phase 3.5 (신설) — 하드웨어 정합 + 회귀 재기준화 (3~4일)

**Phase 4 진입의 전제. 건너뛰면 이후 모든 게이트가 오염된 baseline 위에 쌓인다.**

1. 시뮬 라이다를 L2로 재설정 (§6.6)
2. `real_l2` / `worst_case_l2` 열화 프로파일 신설, `real_l1`은 DEPRECATED (§6.5)
3. `params_ledger.yaml` 도입 + `safety_stop.yaml` 스키마 변경 (§6.3) + pytest status 검사
4. `self_filter.yaml` 재작성 **시뮬 판** (실기체 판은 P8'에서 재조정)
5. RTF 운영점 재유도 (§6.6 v3.1 항목)
6. **Phase 1~5 게이트 전부 재실행** (v3.1: P4 46종 + P5 소셜 포함) →
   `verification/phase3.5_rebaseline.csv`

게이트: 시나리오 세트 전체(P3 4종 · P4 7종+열화 · P5 5종) × N≥5 재통과 ·
`real_l2` 정적 정지 N≥3 · pytest 전부 통과 ·
**v2 baseline과의 델타를 문서화** (`docs/changelog/l1_to_l2_delta.md`)

> 델타 문서화가 중요하다. "L1에서 통과했던 값이 L2에서 얼마나 달라지는가"가
> 열화 하네스의 유효성과 이후 sim2real 예측의 신뢰도를 결정한다.

### 9.2 Phase 4' — 속도 적응형 게이트 (요구 B) (5~7일, v2 대비 +2일)

v3.1: P4는 완료 상태(46/46, `phase4_metrics.csv`) — 이 phase는 신규 구현이 아니라
**L2 재기준화 위 재검증 + 구조 교체(C1·C2·C6)**다. v2 항목 유지 + 다음 변경:

1. CPA 트리거의 팽창항을 **CP 분위수**로 교체 (C1) → `prediction_node` 신설
2. `cp_calibration.py`로 시뮬 GT 잔차에서 1차 분위수 산출 (`cp_quantiles.yaml`)
3. `N`을 `N_fast`/`N_ped`로 분리 (C2), 프레임별 조건 만족 이력 로깅 추가
4. **fast-class 판정을 다변수화** (v2의 `|v|>2.0` 단일 조건 → 아래)

```
fast_class = (|v_obs| > 2.0)
             AND (예측 경로 교차 or d_CPA < footprint_halfwidth(v) + cp_quantile)
             AND (t_CPA < slow_ttc)
             AND (track_confidence ≥ θ)

관찰만:  고속 + 경로 비교차 → 속도 캡만, STOP 없음
보수화:  고속 + 예측 불확실(cp_quantile 큼) → SLOW 선행
```

5. `TrackedObstacleArray` 병행 퍼블리시 (C6)
6. `ROI_fast`, `stop_ttc`, `T_total`은 **PENDING 유지** — 시뮬은 PROVISIONAL 값으로
   진행하되 산출물 CSV에 `pending_params` 기록
7. (선택) `cbf_filter_node` — 계층 2 정식화

verify (N≥5, `phase4_metrics.csv`): v2 게이트 전부 + **CP 커버리지 검증**
(예측 tube가 실제 GT 위치를 명목 신뢰수준만큼 덮는지, 클래스×지평별) ·
`real_l2` 정면 4 m/s 3/3 · Phase 3.5 회귀 재통과

### 9.3 Phase 5' — 소셜 내비게이션 (요구 A) (8~10일, v2와 동일)

v3.1: P5a/5b는 완료 상태 — 이 phase의 실작업은 **C3 이관**(§6.7)과 L2 재기준화
재검증이다.
**5a** — 기구현 유지. `social_gap_node`로 판단 로직 추출, 출력을 **goal**로 이관 (C3, §6.7).
**5b** — 기구현 YIELD를 goal 생산 + 도달 판정으로 이관. `goal_follower` 신설 (§6.8).

verify: v2 게이트 전부 유지 + **추가 게이트 2개**

```
· 조향 단일성 불변식: 게이트/supervisor 가 vy 를 생성하는 코드 경로 0 (pytest)
· goal 커밋 안정성: 동일 장면에서 goal 이 3 s 내 좌우 반전 0/5  ← 채터 회귀
```

⚠ Harmonic 보행자 시뮬: HuNavSim 래퍼가 없으므로 **스크립트 궤적 actor로 진행**.
반응하는 보행자는 Track N 이후 필요 시 포팅 (§7.3).

### 9.4 Phase 6' — 발 근처 장애물 (요구 C) — 조건부 (0~7일)

**측정 8 결과에 따라 규모가 결정된다.** 실기체 전에는 착수하지 않는다.

| 측정 8 결과 | Phase 6' 규모 |
|---|---|
| 8C 낙차 검출 가능 + 8B 0.10 m 검출 가능 | **축소**: `ground_obstacle_node` 불필요. 하강 단차 STOP을 L2 기반으로 게이트에 편입. 1~2일 |
| 8C 가능 / 8B 불가 | **중간**: 상승 장애물만 ToF 보완 (v3.1 — 스펙 확정 전제, §4.3). 3~4일 |
| 8C 불가 | **존속 + 우선순위 상승**: v2 계획대로 7일. G5 실패이므로 Phase 5' 앞으로 이동 검토 |

어느 경우든 **정책 임계(0.10 / 0.30 / 0.12 m)는 측정 8B/8C 실측으로 재유도**한다.

### 9.5 Phase 7' — 폐색 인지 감속 (요구 D) (4~5일, v2와 동일)

v2와 동일. 변경점만:
- `a_brake`는 PENDING → 측정 9A + P8 실측으로 확정
- Jazzy SpeedFilter path-lookahead가 없으므로 **자체 노드 유지가 정답**임을 명기 (§1.7)
- 팬텀 속도 1.5 / 2.0 m/s는 CONFIRMED 유지 (RSS Rule 4)

### 9.6 Phase 8'~10' — sim2real 하드웨어 트랙

체크리스트 정본: `docs/field_protocol.md` (갱신 필요)

**P8' 벤치 (5~7일, v2 대비 +2일)** — `go2_real_bringup` 신규 패키지 (알고리즘 0줄 유지)

```
v2 항목 + 신규:
  · go2_vendor_avoid_guard (C9) + ObstaclesAvoidClient 금지 CI
  · self_filter.yaml 실기체 판 재조정 → G0
  · 측정 1, 3, 5, 6, 7, 8, 9 전부 실행
  · 대장 PENDING → CONFIRMED 전환 + 시뮬 회귀 재실행

게이트: G0~G5 전부 + v2 게이트(cmd→다리 latency, L1 10 Hz 10분 → L2 5.55 Hz 기준 재정의,
       SW/HW e-stop, 제동 감가속 N≥5) + proprio 안전 입력 임계 실측·오발률 (§6.11 v3.1)
       + 대장에 PENDING 잔여 0
```

> ⚠ v2 게이트 "L1 ≥10 Hz 10분"은 L2에서 **≥5.5 Hz**로 재정의된다. 그리고 이것이
> 성능 저하가 아니라 **센서 사양**임을 명시해야 한다 (측정 5·6으로 확정).

**P9' teleop 센서 검증 (4~6일)** — v2와 동일 + 신규:
- bag 재생 오프라인 재튜닝의 출발점을 `real_l2` 튜닝값으로
- **CP 분위수 실기체 재캘리브레이션** (시뮬 대비 델타 기록 = 열화 하네스 유효성 증거)
- 게이트: v2 항목 + `R_confirm` 실환경 재확인 (자전거 실측 ≥10회)

**P10' 단계적 필드 (2~3주)** — v2와 동일. 단 속도 캡을 **`v_robot_max` 실측값**으로
대체 (v2의 임의 0.3~0.5 m/s → 유도값). `v_robot_max < 0.3`이면 §9.7 대안 B.

### 9.7 🔴 G4 실패 시 대안 아키텍처 B

`v_robot_max < 0.5 m/s`로 나오면 알고리즘 문제가 아니라 **센서 구성 문제**다.
Nav2 통합보다 다음이 먼저다.

```
대안 B (저속·정지 중심 재구성):
  1. fast-class 를 "회피 대상"이 아니라 "대기 대상"으로 재정의
     → 자전거 접근 시 인도 가장자리 정지 + 통과 대기 (YIELD 의 fast-class 판)
     → swerve/우회 완전 배제, stop_ttc 를 낮추는 대신 정지 상태에서 대기
  2. 센서 보강 선행: 전방 지향 고밀도 센서(저반사율 강건) 추가 검토
     (v3.1: ToF 는 사거리가 짧아 자전거 조기 경보엔 부적합할 수 있음 — 스펙 확정 후 판정)
  3. Track N 을 P10 이후로 유지 (v2 순서 그대로)
  4. 요구 B 의 성공 기준을 "회피"에서 "안전한 양보"로 하향 조정하고 문서화
```

**대안 B로 가는 것은 실패가 아니다.** 15 m @10% 반사율이라는 물리 제약 하에서 6 m/s
상대속도를 조향으로 회피한다는 것 자체가 원래 무리였을 수 있다. v2가 이미
"고속 물체에는 swerve 아니라 조기 정지"를 원칙으로 삼았으므로, 대안 B는 그 원칙의
논리적 귀결이다.

v3.1 보강: Falanga는 조기 제동을 정당화할 뿐 "충돌 경로 위 부동"까지 규정하지
않으며, **가장자리 이동 후 정지는 RSS 호환 중간지대**다 — 이는 정확히 기구현 YIELD의
형태이므로 대안 B는 강등이 아니라 **기존 기계의 재사용**이고, G4 결과와 무관하게
fast-class 대응의 유력 기본 후보로 A/B 비교 가치가 있다.

### 9.8 Track V / Track N

**Track V** — v2와 동일. 추가: `TrackedObstacleArray.obstacle_class` 필드에 채우도록 계약 갱신.

**Track N** — 진입 시점은 **P10'(b) 이후 유지** (v2 원칙 보존). 단:
- 사전 결정 사항은 §3.4에서 이미 확정 → 진입 시 재논의 없음
- `/social/goal` 계약이 Phase 5'에서 이미 고정 → `goal_follower` 삭제 + BT 노드로 교체
- MPPI 파라미터는 측정 4에서 이미 확정
- `nav2_collision_monitor` A/B 비교 시 `intervention ratio` 지표 필수:
  `safety filter가 명령을 수정한 시간 / 전체 주행 시간`. 값이 크면 planner가 나쁘거나
  filter가 과보수적이라는 뜻 — 어느 쪽인지는 궤적 품질과 함께 봐야 판별된다
- ⚠ intervention 측정 시 **gait 반사로 인한 편차를 filter 개입으로 오분류하지 않도록**
  `cmd_vel_safe` 대비 실측 odom 속도를 함께 기록 (§1.6 단서)

---

## §10. 리스크 갱신

v2 리스크 표 유지 + 다음을 **신규/승격**한다.

| 리스크 | 등급 | 대응 |
|---|---|---|
| 🔴 **Go2 X SKU가 Ethernet+CycloneDDS 접근을 허용하지 않음** | 신규·최상위 | 측정 3에서 최우선 판정. 실패 시 WebRTC 경로의 지연을 실측하고 요구 B 성공 기준 재정의 (§9.7) |
| 🔴 **L2 0.05 m 블라인드로 self-filter 파손 → 로봇이 기동 불가** | 신규 | G0 게이트. `self_filter.yaml` 전면 재작성 (§6.1). 시뮬에서 먼저 재현 (Phase 3.5) |
| 🔴 **저반사율 자전거의 트랙 형성 거리가 필요 감지 거리 미달** | 신규 | 측정 7 → `v_robot_max` 역산 → G4 → 대안 B (§9.7) |
| 🟠 시뮬 센서 변경으로 통과한 Phase 1~5 게이트가 무효 (v3.1: P4·P5 포함) | 신규 | Phase 3.5 재기준화 + 델타 문서화 (§9.1) |
| 🟠 Jazzy에서 `unitree_ros2` 미검증 (CycloneDDS 0.10.2 핀) | 신규 | 측정 3. Humble 드라이버 컨테이너 + 브리지가 폴백 |
| 🟠 Jetson에서 pre-Eigen MPPI 처리량 부족 | 신규 | 측정 4 → G3. v3.1: 데스크톱에서도 batch 1000/steps 40/10 Hz 경량화 운용 중(CPU 과부하가 크롤링·ABORT 원인) — Orin Nano 실패 개연성 높음. 백포트 / Vector Pursuit 를 준-기본 경로로 |
| 🟠 L2 드라이버 생태계 (v3.1) | 신규 | 별도 SDK(unilidar_sdk2 — L1용과 미호환) · Jazzy 미지원 · 타임스탬프 버그 사례 → 측정 5 무결성 체크 (§8.1) |
| 🟡 C3 이관 중 튜닝 불변식 소실 (v3.1) | 신규 | §6.7 보존 목록 + 이관 전후 P5 게이트 동등성 회귀 |
| 🟠 CP 분위수의 시뮬↔실기체 델타가 큼 | 신규 | P9' 재캘리브레이션. 델타 자체가 열화 하네스 유효성 지표 |
| 🟠 Harmonic 보행자 시뮬 부재 | 신규 | 스크립트 궤적으로 진행. HuNavSim 포팅은 후순위 (§7.3) |
| 🟡 벤더 회피 스위치가 이벤트로 재활성화 | 신규 | 측정 1 Step4로 주기 결정 + 가드 노드 재확인 + bag 무효 플래그 |
| 🟡 `goal_follower`에 로직이 축적되어 삭제 불가해짐 | 신규 | 파일 헤더에 "삭제 예정" 명시 + 100줄 상한을 lint로 강제 |
| 🟡 대장 PENDING 값으로 실기체 기동 | 신규 | 원칙 9 런타임 가드 + pytest |
| ⬇️ **L1 근거리 블라인드로 낮은 장애물 못 봄** | v2 최상위 → **재분류 (v3.1)** | 0.8 m는 URDF 설정값(§1.2 정정) — 시뮬 min_range 0.05 정합(Phase 3.5) + 실기 근거리 특성 측정 5·8 확정 후 Phase 6' 규모 판정 (§9.4) |
| ⬆️ **실제 라이다 희소·비반복 스캔으로 클러스터 파손** | v2 최상위 → **유지** | 여전히 최대 단일 리스크. `real_l2` + P9' bag 재튜닝 |

---

## §11. 파일 지도 (v3)

```text
ros2_ws/src/perception_avoidance/
  perception_avoidance/
    collision_oracle_node.py      # 평가 [v3: pending_params/vendor_avoid_state 열]
    lidar_obstacle_node.py        # [v3: ROI 대장참조, 누적 재작성, N 분리, tracked 퍼블리시]
    prediction_node.py            # ★신규 C1 — CVM + CP tube
    safety_stop_node.py           # 게이트 [v3: CP 반경, 비등방 footprint, PENDING 가드, vy 제거]
    goal_follower.py              # ★신규 C3 (임시 — Track N 에서 삭제)
    cbf_filter_node.py            # ★신규 C4 (계층 2, P4' 선택)
    degrade_pointcloud_node.py    # [v3: L2 특성]
    (P5') sidewalk_polygon_node.py, social_gap_node.py   # goal 출력으로 설계 변경
    (P6') ground_obstacle_node.py                         # 조건부 존속
    (P7)  occlusion_speed_node.py
    (V)   yolo_class_node.py, class_fusion_node.py
  config/
    params_ledger.yaml            # ★신규 C7 — 정본, 기계 판독
    cp_quantiles.yaml             # ★신규 C1
    safety_stop.yaml              # [v3: status 스키마]
    self_filter.yaml              # [v3: L2 기하 전면 재작성]
    (P5') sidewalk_boundaries.yaml, (P6') camera.yaml, ground_obstacle.yaml
  msg/
    TrackedObstacle.msg, TrackedObstacleArray.msg   # ★신규 C6
    PredictionTube.msg, PredictionTubeArray.msg     # ★신규 C1
  test/                           # [v3: status 검사, vy 불변식, ObstaclesAvoid grep]
ros2_ws/src/go2_simulation/       # [v3: L2 라이다 파라미터, 측정용 월드]
(P8') ros2_ws/src/go2_real_bringup/
    sport_cmd_adapter.py          # [v3: vy 제한, 커플링 보상]
    go2_vendor_avoid_guard.py     # ★신규 C9
    proprio_safety_node.py        # ★신규 v3.1 — 고유수용성 ESTOP 입력 (§6.11)
scripts/cp_calibration.py         # ★신규 C1
measurement/                      # ★신규 — 측정 스크립트 9종 + common.py
verification/
  profiles/  real_l2.yaml, worst_case_l2.yaml, real_l1.yaml(DEPRECATED)
  measurement_worlds/             # ★신규 — 반사율/계단/낙차/폴/클러터
docs/
  interfaces.md                   # [v3: TrackedObstacle 추가, depth 범위 주석]
  field_protocol.md               # [v3: G0~G5, 측정 1~9]
  measurement_protocol.md         # ★신규
  changelog/l1_to_l2_delta.md     # ★신규 — Phase 3.5 델타
```

---

## §12. 실행 순서

```
0. ✅ Phase 3 (검증 인프라) + P4 (46/46) + P5a/5b/P5-4 + Track N v1/v2 시뮬 통합
   — L1 baseline 완료분 (v3.1 현행화)
1. 측정 3, 4  ← 하드웨어 불필요. Go2 X DDS 권한 판정이 최우선 리스크
   (v3.1: 서면 확인 요청은 즉시, §1.5)
2. Phase 3.5  하드웨어 정합 + 회귀 재기준화 (P4·P5 포함) → phase3.5_rebaseline.csv
3. Phase 4'   재검증 + CP 교체 + 클래스별 N (PENDING 값은 PROVISIONAL 로 시뮬 진행)
4. Phase 5'   C3 이관 (기구현 YIELD → goal 인터페이스) + 재검증
   (병렬) Track V 계약 유효, 팀원 segmentation/depth 개발
5. 로봇 확보 시 즉시 병렬:  G0 → 측정 1 → 5 → 6 → 9 → 7 → G1~G5
6. Phase 6' 규모 결정 (측정 8) → Phase 7'
7. 대장 PENDING 전량 해제 → 시뮬 전체 회귀 재통과
8. P9' → P10' → Track N
```

**Phase 4'와 5'는 실기체를 기다리지 않는다.** 두 phase의 산출물은 구조와 인터페이스이고,
확정되지 않은 것은 스칼라 값뿐이다. 대장의 PENDING/PROVISIONAL 구분이 그 경계를 명시한다.

---

## §13. 미해결 질문 (측정으로만 답이 나오는 것)

1. Go2 X SKU에서 Ethernet + CycloneDDS `unitree_ros2` 접속이 가능한가? (측정 3 —
   단 v3.1: 구매처/Unitree 서면 확인을 선행, §1.5) — **최상위**
2. 내장 L2의 실제 포인트율은 21,600인가 64,000인가? (측정 5)
3. 저반사율 자전거의 `R_confirm_p10`은 몇 m인가? 그래서 `v_robot_max`는? (측정 7) — **G4**
4. L2로 낙차(하강 단차)를 검출할 수 있는가? (측정 8C) — **G5**
5. negative angle 모드가 Go2 내장 L2에서 노출되는가? (측정 8A)
6. `K_vy / K_vx`는 얼마인가 — Go2를 holonomic으로 다룰 수 있는가? (측정 9)
7. Jazzy pre-Eigen MPPI가 Jetson 풀스택에서 15 Hz를 내는가? (측정 4) — **G3**
8. 벤더 회피 스위치가 어떤 이벤트에서 뒤집히는가? (측정 1 Step4)

---

## 부록 A. v2 → v3 변경 이력 요약

| 구분 | 항목 수 | 비고 |
|---|---|---|
| 유지 (핵심 원칙) | 7 | §0.1 |
| 무효화 (하드웨어) | 7 | §0.2 |
| 신규 구조 (C1~C10) | 10 | §0.3 |
| 신설 원칙 | 3 (7·8·9) | §2 |
| 코드 수정 파일 | 13 | §6 |
| 신규 파일 | 12+ | §6.14, §11 |
| 신규 채택 문헌 | 9 | §7.2 |
| 신설 게이트 | 6 (G0~G5) | §8.3 |
| 신설 Phase | 1 (3.5) | §9.1 |
| 조건부 Phase | 1 (6') | §9.4 |
| v3.1 정정 (2026-08-01) | 9 | 헤더 개정 블록 참조 |

## 부록 B. 이번 개정에서 배운 것 (프로세스)

1. **유도 결과만 기록하면 전제가 바뀔 때 무엇을 다시 계산해야 하는지 알 수 없다.**
   `ROI = 15 m`가 아니라 `ROI = v_rel × (stop_ttc + T_total)` + 각 심볼의 출처를 기록한다.
   → 원칙 8, 파라미터 대장.
2. **하드웨어 스펙은 제품 페이지가 아니라 데이터시트로, 그리고 최종적으로 실측으로
   확인한다.** 제품 페이지의 "21,600 pts/s"가 데이터시트와 3배 차이 났다.
3. **개선이 열화로 위장해 오는 경우가 있다.** 근거리 블라인드 0.8 → 0.05 m는 스펙상
   개선인데 self-filter를 파손시킨다. "스펙이 좋아졌다"를 "영향 없다"로 읽으면 안 된다.
   (v3.1 후속: 0.8 m 자체가 URDF 설정값이었음이 확인됐다 — **"시뮬 설정값을 하드웨어
   스펙으로 오기억"하는 것**이 네 번째 교훈이다, §1.2.)
4. **v2의 열화 게이트 인터리브와 phase별 CSV 산출물이 피해를 국지화했다.** 종단에
   sim2real을 몰지 않은 판단이 옳았다. 이 두 장치는 v3에서 강화한다.
