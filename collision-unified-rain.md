# Unitree Go2 장애물 회피 시스템 설계 계획

## Context

Unitree Go2 로봇의 인도 자율주행을 위한 장애물 회피 알고리즘을 개발한다. Gazebo Harmonic 시뮬레이션에 보행자 actor가 있지만 actor는 물리에 참여하지 않으므로 시뮬레이터 내장 collision으로는 충돌을 판정할 수 없다. 따라서 평가용 ground-truth와 회피용 perception을 명확히 분리해 단계적으로 구현한다.

---

## 핵심 설계 원칙

**회피 알고리즘은 Gazebo actor ground-truth pose에 의존하지 않는다.**

- Actor pose는 **평가용 ground-truth (collision oracle / metric)** 로만 사용한다.
- 회피 판단은 L1 LiDAR(`/unitree_lidar/points`) 와 D435i(`/d435i/*`) 등 **실제 로봇 센서 토픽**만 사용한다.
- 이 분리가 sim2real 전이의 전제 조건이다.

```text
Gazebo ground-truth pose
  → evaluation / metric / collision oracle only

L1 LiDAR + D435i
  → perception
  → SLOW_DOWN / STOP / WAIT / (옵션) lateral avoid
  → cmd_vel safety gate
  → CHAMP controller
```

---

## 카메라 스펙 주의

현재 시뮬레이션은 **D435i** (RGB hfov 87°, depth 0.105~10m, IMU 200Hz)로 모델링되어 있다. 그러나 다음을 명시한다.

- 실제 사용 카메라는 **변경 가능성이 높다** (D455, ZED, Realsense 외 모델 등).
- 따라서 회피/평가 알고리즘은 **특정 카메라 스펙(hfov, depth range, 해상도)에 hardcode 하지 말 것**.
- 카메라 파라미터(intrinsics, depth range, fov)는 ROS2 파라미터 또는 YAML 설정으로 분리한다.
- D435i depth range(약 10m)는 빠른 객체(예: 자전거 20km/h)에 부족할 수 있으므로 **거리/TTC는 LiDAR 기반**이 안전 기준선이다. 카메라가 더 좋은 스펙으로 교체되더라도 LiDAR-primary 원칙은 유지.

---

## 원래 계획의 Gap 분석 (재정리)

| 영역                           | 문제점                                              | 보완 방향                                                                |
| ------------------------------ | --------------------------------------------------- | ------------------------------------------------------------------------ |
| **Phase 1 역할 혼재**          | 평가용 ground-truth와 회피 트리거가 한 phase에 섞임 | 평가 oracle 전용으로 축소. 회피는 perception 기반으로 분리               |
| **Actor collision 의존성**     | Actor 물리 collision 시도 실패 (롤백 완료)          | Custom system plugin으로 actor pose 발행 → 평가 oracle만 소비            |
| **Robot pose 소스**            | `/odom`은 drift 누적 → 메트릭 부정확                | 평가에는 Gazebo ground-truth (`/world/default/pose/info`) 사용           |
| **회피 메트릭 정의**           | 단순 center 거리                                    | `clearance = center_distance - robot_radius - actor_radius`              |
| **좁은 인도에서 lateral 회피** | VFF는 인도 폭(1.5~2m)에서 과설계, curb 충돌 위험    | Phase 4를 4a (stop/wait/resume) + 4b (lateral)로 분리                    |
| **커스텀 메시지 신규 패키지**  | 초기 범위에 과투자                                  | `vision_msgs`, `geometry_msgs` 등 표준 메시지 재사용                     |
| **패키지 5개 분할**            | 현재 범위 대비 무거움                               | 단일 `perception_avoidance` 패키지로 시작, 안정 후 분리                  |
| **D435i 거리 추정 의존**       | 빠른 객체(자전거)에 깊이 10m 한계                   | 거리/TTC는 LiDAR. D435i + YOLO는 분류 보조                               |
| **인도 경계 인식**             | 미정                                                | 정적 polygon geofence (시뮬용) + LiDAR 연석 감지 (실환경 전이용, 후순위) |
| **레이턴시 분석**              | 자전거 시나리오 미검증                              | LiDAR-only 긴급정지 경로 필수 (YOLO 우회)                                |

---

## 현재 구현 상태 (as-built, 2026-06-28 기준)

계획과 실제 코드가 일부 앞서거나 뒤처져 있다. 각 phase 진입 전 아래 드리프트를 먼저 해소한다.

| 항목                | 계획                                                  | 실제 코드                                                                                 | 조치                                                                    |
| ------------------- | ----------------------------------------------------- | ----------------------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| Oracle 위치/이름    | `perception_avoidance/collision_oracle_node.py`       | `go2_simulation/scripts/collision_monitor_node.py` (클래스 `CollisionMonitorNode`), launch도 여기서 실행 | ✅ perception_avoidance/collision_oracle_node.py로 이전 + 다중 actor 자동 지원 |
| 속도 추정           | `/obstacles/lidar/velocity` (frame-to-frame + Hungarian) | ✅ frame-to-frame 추적(greedy NN+EMA) 구현                                              | ✅ 구현 완료 (safety_stop이 상대속도 TTC 사용)                      |
| twist_mux + e-stop  | Phase 1에 설치 (lock 토픽)                            | 미설치. safety gate가 cmd_vel 경로에 인라인(`/cmd_vel`→gate→`/cmd_vel_safe`)               | ✅ Phase 5로 연기 확정 (단일 소스라 불필요; Nav2 도입 시 구성)          |
| ROI z 범위          | `z ∈ [−0.3, 1.8m]`                                    | `self_filter.yaml: roi_z_max=0.5`                                                          | ✅ 문서를 config(z_max=0.5)에 맞춤. 머리 위 구조물 제외 유지          |
| Bridge 잔재         | actor_pose/info 한 줄                                 | dynamic_pose/info + actor_pose/info 둘 다 존재                                             | ✅ dynamic_pose/info 제거                                              |

확인된 정상 동작: 플러그인 `<model_name>go2</model_name>` 등록됨, actor 이름(`pedestrian_on_sidewalk16`)이 oracle 기본값과 일치 → 단일 actor 평가 경로 동작 전제 충족.

---

## 검증 공통 원칙 (모든 phase 적용)

각 phase는 아래 6원칙을 만족해야 "통과"로 간주한다. 정성·1회·육안 확인은 통과 근거가 아니다.

1. **정량 PASS/FAIL 임계**: 모든 검증 항목에 숫자 기준. 애매하면 통과 아님.
2. **반복(N≥5회)**: 확률적 항목(false stop, stuck, 충돌)은 1회가 아니라 N회 실행 후 통계로 판정.
3. **자동화 시나리오 러너**: 고정 pose spawn → 고정 cmd 입력 → actor 궤적 → rosbag 기록 → 메트릭 자동 산출 → 임계 비교 PASS/FAIL 출력. 수동 teleop 육안 금지.
4. **회귀(regression)**: 새 phase 진입 시 이전 phase 게이트 전부 재실행.
5. **산출물 의무화**: phase별 `verification/phaseN_metrics.csv` + PASS/FAIL 1줄 로그를 남겨야 넘어간다. (메트릭 phase에서 CSV는 선택이 아닌 필수)
6. **단위 테스트**: Gazebo 없이 검증 가능한 순수 함수(clustering, TTC/clearance 수식, corridor 필터)는 pytest로 분리.

---

## 시스템 아키텍처 (재정리)

```text
[Gazebo Harmonic]
    │
    ├── /unitree_lidar/points   (PointCloud2)     ─┐
    ├── /d435i/image            (Image, RGB)        │  perception 입력
    ├── /d435i/depth_image      (Image, depth)      │  (스펙 변경 가능)
    ├── /d435i/points           (PointCloud2)       │
    ├── /d435i/imu              (Imu)               │
    ├── /odom                   (Odometry)         ─┘
    │
    ├── /world/default/pose/info        (Pose_V → TF)  ─┐
    └── /world/default/actor_pose/info  (커스텀 plugin) │ evaluation 입력
                                                       ─┘
    ▼
┌──────────────── EVALUATION (평가 전용) ────────────────┐
│                                                        │
│  [collision_oracle_node]                               │
│   Sub: ground-truth robot pose, actor pose             │
│   Calc: clearance = center_dist - r_robot - r_actor    │
│   Pub: /collision_events, metrics CSV                  │
│                                                        │
│   ※ 회피 로직과 완전 분리. cmd_vel에 영향 없음.        │
└────────────────────────────────────────────────────────┘

┌──────────────── PERCEPTION LAYER ──────────────────────┐
│                                                        │
│  [lidar_obstacle_node]                                 │
│   Sub: /unitree_lidar/points                           │
│   Pipeline:                                            │
│     TF → base_link, ROI crop, self-filter (다리/몸통), │
│     ground removal, voxel, cluster, track              │
│   Pub: /obstacles/lidar (vision_msgs/Detection3DArray) │
│        /obstacles/lidar/velocity (TwistStamped[])      │
│                                                        │
│  [yolo_class_node]  (Phase 4)                          │
│   Sub: /d435i/image (camera 파라미터 분리)             │
│   Pub: /obstacles/class (vision_msgs/Detection2DArray) │
│                                                        │
│  ※ 안전 거리/TTC는 LiDAR 단독.                         │
│  ※ class는 임계값 보수화에만 사용 (안전거리 완화 금지) │
└────────────────────────────────────────────────────────┘
                        │
                        ▼
┌──────────────── DECISION LAYER ────────────────────────┐
│                                                        │
│  [safety_stop_node]  (cmd_vel safety gate)             │
│   Sub: /cmd_vel_in (Nav2/teleop 등 상위 입력)          │
│        /obstacles/lidar, /odom                         │
│   동작: 입력 cmd_vel을 위험도에 따라 scale/stop        │
│   States: NOMINAL → SLOW_DOWN → STOP → WAIT → RESUME   │
│           (+ STUCK, Phase 2.5)                         │
│   Pub: /cmd_vel_safety = filter(/cmd_vel_in)           │
│                                                        │
│  [lateral_avoid_node] (Phase 3, Nav2 도입 후 OFF)      │
│   sidewalk polygon 내 측면 회피                        │
└────────────────────────────────────────────────────────┘
                        │
                        ▼
┌──────────────── CONTROL LAYER ─────────────────────────┐
│                                                        │
│  [twist_mux]                                           │
│   P4: /cmd_vel_estop      (긴급정지: lock 토픽 권장)   │
│   P3: /cmd_vel_safety     (gate 출력)                  │
│   P2: /cmd_vel_nav        (Phase 5 Nav2 도입 시)       │
│   P1: /cmd_vel_teleop                                  │
│   → /cmd_vel → CHAMP                                   │
│                                                        │
│   ※ E-stop은 zero twist + lock으로 상위 입력 차단      │
│     (timeout 사이 이전 명령 통과 방지)                 │
└────────────────────────────────────────────────────────┘
```

---

## 핵심 기술 결정 (업데이트)

| 결정 사항          | 선택                                                     | 이유                                                                                                |
| ------------------ | -------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| 구현 언어          | C++ 우선 (plugin은 C++, ROS 노드는 초기 Python 가능)     | 실시간성 + 빠른 프로토타이핑 균형                                                                   |
| 회피 알고리즘      | **stop/wait 우선, lateral은 옵션**                       | 좁은 인도(1.5~2m)에서 lateral은 curb 충돌 위험                                                      |
| Costmap 사용       | **초기엔 사용 안 함**, Phase 5에서 Nav2와 함께 도입      | 빠른 장애물에 costmap 5~10Hz 업데이트는 느림                                                        |
| FSM vs BT          | FSM (5상태: NOMINAL/SLOW/STOP/WAIT/RESUME)               | 시나리오 유한, BT 과설계                                                                            |
| 기준 프레임        | base_link                                                | 반응형 회피는 로봇 상대 위치만 필요                                                                 |
| 주 LiDAR           | **L1 (`/unitree_lidar/points`) only**                    | 실기체 탑재. Velodyne 제거됨                                                                        |
| 거리/TTC 계산      | **LiDAR 단독**                                           | 카메라 스펙 의존성 제거. **정적 TTC(robot_vx만)로는 정지 상태에서 접근하는 물체 미반응** → 상대속도 TTC + velocity 추정 필수 (Phase 1)                                                       |
| 분류 (class)       | YOLO + D435i 보조                                        | 거리는 LiDAR. class는 보조                                                                          |
| Actor pose 소스    | **custom gz-sim system plugin** (`actor_pose_publisher`) | `dynamic_pose/info`에 actor 미포함. world-level plugin이 actor 자동 추종                            |
| 평가용 robot pose  | **Gazebo ground-truth**                                  | `/odom` drift 회피. metric 정확도 확보                                                              |
| 메시지 정의        | **표준 메시지 재사용**                                   | `vision_msgs/Detection3DArray`, `geometry_msgs/TwistStamped`, `std_msgs/String`. 신규 패키지 미생성 |
| 패키지 분할        | **단일 `perception_avoidance` 패키지로 시작**            | 안정화 후 perception / evaluation / avoidance로 분리                                                |
| 카메라 스펙 의존성 | **파라미터화 (YAML)**                                    | 카메라 모델 변경 가능성 높음. hardcode 금지                                                         |

---

## 단계별 개발 계획 (재구성)

### Phase 0: 센서/브리지 현황 정리 (1일)

현재 bridge 되어 있는 토픽을 문서로 확정하고, 누락된 항목만 보완.

대상 토픽:

- `/unitree_lidar/points`
- `/d435i/image`, `/d435i/depth_image`, `/d435i/points`, `/d435i/camera_info`
- `/d435i/imu`
- `/odom`

verify (topic rate + 시각적 확인):

```bash
ros2 topic hz /unitree_lidar/points
ros2 topic hz /d435i/depth_image
ros2 topic hz /d435i/imu

# Phase 1 진입 전 필수 추가 검증
rviz2  # PointCloud2 표시
# 확인 항목:
#  1) actor가 /unitree_lidar/points에 실제로 찍히는가?
#  2) actor cluster가 충분히 dense한가? (5 points 이상)
#  3) Go2 자기 다리/몸통 point가 obstacle로 잡히는가?
#     → 잡히면 Phase 1 전처리에 self-filter 필수
#  4) actor와 벽/가로수 cluster가 붙어버리는가?
#  5) lidar_frame → base_link TF, timestamp 정상인가?
```

**검증 게이트 (정량, 모두 PASS해야 Phase 0.5 진입)**:

- topic rate: `/unitree_lidar/points` ≥ 8Hz, `/d435i/depth_image` ≥ 12Hz, `/d435i/imu` ≥ 180Hz (스펙 15/200Hz 하한).
- **actor 탐지 곡선**: actor를 전방 x = 1·2·4·6·8m에 정지시켜 각 거리의 actor cluster point 수 기록 → "8m에서 ≥ 5점" 같은 임계로 탐지 한계 거리 확정.
- **근거리 블라인드 존 측정**: 물체를 0.3→1.0m로 접근시켜 LiDAR에 처음 잡히는 거리 기록 (L1 min range ~0.8m 확인). 이 구간은 D435i depth(near 0.105m)로 보완하거나 한계로 문서화.
- **self/TF 정합**: 빈 구간 정지 시 self-filter 후 전방 cone cluster 0개. `base_link←lidar_l1_link` TF 존재 + timestamp drift < 50ms.
- **Phase 4 사전조건 사전 확인**: `/d435i/camera_info` intrinsics 채워짐, `base_link→camera` / `base_link→lidar_l1_link` TF 존재.
- 산출물: `verification/phase0_sensors.csv` (거리별 point 수, rate).

actor가 LiDAR에 잘 안 찍히면 회피 알고리즘이 작동 못 함 → 이 경우 actor model의 `<visual>`/`<collision>` 형상을 더 단순한 cylinder/box proxy로 보강하거나, perception 디버그용 보이지 않는 collision proxy를 별도 spawn.

**참고**: 카메라 스펙(D435i)은 변경 가능성 높음. 토픽 네임스페이스(`/d435i/`)를 알고리즘 코드에 hardcode 하지 말고 launch arg 또는 remap으로 처리.

---

### Phase 0.5: Evaluation Oracle (2~3일)

**목적**: 평가용 ground-truth 인프라를 회피 알고리즘보다 먼저 완성. 이후 모든 phase의 성능을 동일 기준으로 측정.

작업:

1. `actor_pose_publisher` gz-sim system plugin 작성 (C++, ~80~100줄)
   - World-level plugin. `components::Actor` entity 자동 탐색.
   - `Actor::WorldPose(ecm)` 매 PostUpdate 읽기.
   - `gz.msgs.Pose_V`를 `/world/default/actor_pose/info`로 발행.
   - 다중 actor 자동 지원.

2. Bridge 한 줄 추가 (`pose_bridge.yaml`):
   - `/world/default/actor_pose/info` → `tf2_msgs/msg/TFMessage`
   - **주의**: ros_gz의 일반 `Pose_V → TFMessage` 변환은 `pose.name` 대신 `pose.header.data["child_frame_id"]` / `["frame_id"]` 키를 읽음 (참고: ros_gz#172). SceneBroadcaster가 발행하는 `dynamic_pose/info`는 이 키를 채우지 않아 변환 후 `child_frame_id`가 빈 문자열이 됨. **본 plugin은 두 키를 명시적으로 설정**해 변환 정확성을 확보 (검증 완료).
   - 향후 다중 actor 명세화/외부 도구 호환을 위해 별도 평가 전용 메시지(`geometry_msgs/PoseArray` + 이름 목록, 또는 안정화 후 custom `ActorPoseArray.msg`)로 마이그레이션 가능. 현재는 TFMessage가 가장 적은 변경량으로 동작.

3. Robot ground-truth pose 토픽 확보:
   - 동일 plugin의 `<model_name>` SDF 파라미터로 robot 모델(`go2`)도 같은 토픽으로 발행. SceneBroadcaster의 `pose/info`는 `name`만 채우고 header data를 비워 bridge 호환성이 떨어지므로 plugin 경유가 안전.
   - Plugin은 actor entity 수가 늘 경우 매 step `Each<Actor>` 스캔이 부하가 될 수 있음 → 향후 entity 캐시 + 변경 시점 무효화로 개선 (현재 actor 1명에선 무시 가능).

4. `collision_oracle_node` 작성 (단일 패키지 `perception_avoidance` 하위, Python 가능):
   - Sub: actor pose (TF), robot pose (TF)
   - Calc:
     ```
     clearance = center_distance - robot_radius - actor_radius
     ```
   - 파라미터: `robot_radius` (기본 0.25m), `actor_radius` (기본 0.3m), `collision_threshold` (기본 0.0m clearance), `warning_threshold` (기본 1.5m clearance)
   - Pub: `/collision_events` (`std_msgs/String`)
   - Metric: `min_clearance`, `collision_count` (종료 시 Summary 로그)

verify (정량):

- **수식 단위테스트(pytest, Gazebo 불필요)**: 알려진 robot/actor 좌표 입력 → clearance 기대값과 일치 (오차 < 1e-6).
- **다중 actor 대응**: oracle를 단일 `actor_name`이 아닌 actor 리스트로 확장. actor 2명 동시 배치 시 min clearance가 가장 가까운 actor 기준으로 계산되는지 검증. (없으면 "다중 정적 장애물 / 군중" 시나리오 메트릭이 무의미)
- actor 접근 시 `/collision_events` 발행 + clearance 임계 일치.
- 산출물: 1회 주행 → `verification/phase0_5_events.csv` (t, actor, clearance) + 종료 요약. clearance 시계열 그래프로 최저점 확인.

**수정/생성 파일**:

- `ros2_ws/src/go2_simulation/plugins/actor_pose_publisher/` (신규 C++ plugin) — 구현 완료
- `ros2_ws/src/go2_simulation/config/pose_bridge.yaml` (actor_pose/info 추가됨; dynamic_pose/info 잔재 제거 검토)
- `ros2_ws/src/go2_simulation/worlds/small_city.sdf` (`<plugin>` 등록 완료, `<model_name>go2</model_name>` 포함)
- `ros2_ws/src/perception_avoidance/` (신규 단일 패키지)
- `ros2_ws/src/perception_avoidance/perception_avoidance/collision_oracle_node.py` — ✅ 이전 완료 (다중 actor 자동 지원, `go2_simulation`의 구 스크립트 삭제됨)

---

### Phase 1: LiDAR 기반 safety stop (3~4일)

**목적**: perception 기반 거리/TTC 계산과 정지 동작. Actor pose 미사용.

작업:

1. `lidar_obstacle_node` 작성 (전처리 순서가 중요):
   - Sub: `/unitree_lidar/points`
   - 파이프라인:
     1. TF 변환: `lidar_frame` → `base_link`
     2. **ROI crop**: x ∈ [0.5, 8m], y ∈ [−3, 3m], z ∈ [−0.3, 0.5m] (인도 주행 범위; z 상한은 머리 위 구조물 제외, config와 일치)
     3. **self-filter**: Go2 body box + 4족 다리 영역 제거 (다리는 보수적으로 약간 크게). 4족 로봇에서 누락 시 자기 다리를 obstacle로 잡아 끊임없이 STOP 발생.
     4. ground removal (Z 임계 + IMU pitch 보정 옵션)
     5. voxel downsampling
     6. Euclidean clustering
     7. frame-to-frame association으로 속도 추정 (centroid 매칭 + Hungarian) — **Phase 1 필수. 현재 미구현(`lidar_obstacle_node`에 미반영)이므로 본 단계 구현이 Phase 1 완료 조건.**
   - Pub:
     - `/obstacles/lidar` (`vision_msgs/Detection3DArray`)
     - `/obstacles/lidar/velocity` (`geometry_msgs/TwistStamped[]` 또는 별도 토픽; cluster id 매칭)

2. `safety_stop_node` (**cmd_vel safety gate**, planner 아님):
   - Sub:
     - `/cmd_vel_in` (`geometry_msgs/Twist`) — Nav2 또는 teleop의 상위 명령
     - `/obstacles/lidar`, `/obstacles/lidar/velocity`
     - `/odom` (로봇 자기 속도)
   - 동작:
     - **직사각형 footprint** corridor (Go2 0.70×0.31 비율): 측면 |y| ≤ robot_half_width(0.155)+corridor_margin(0.18), 전방 clearance = front_face_x − robot_half_length(0.35). 원형 robot_radius 폐기(전방 과소평가 해소).
     - **TTC = clearance / closing_speed** (clearance≤0이면 즉시 STOP, closing≤0이면 ∞)
       - `closing_speed = -dot(p_rel, v_rel) / norm(p_rel)`
       - `v_rel = obstacle_vel − robot_vel` (base_link frame)
       - ✅ 상대속도 TTC 구현됨 (속도는 Detection3D의 covariance[0/1]에 임베드 — 별도 토픽 stamp 매칭의 순서 경쟁 회피). closing은 `max(closing, robot_vx)`로 정적 fallback 보장.
     - **거리 STOP은 작은 `emergency_clearance`(0.25m, python 기본 0.4) 안에서만**. 큰 거리 반응은 TTC(접근 중) 기준 → 회전으로 정적 벽을 마주봐도(closing≈0→TTC=∞) STOP 안 함 (false STOP 방지).
     - 상태별 출력 (**회전 `angular.z`는 SLOW/STOP에서도 항상 통과** → 벽을 향한 채 교착되지 않고 돌아 나감. 후진은 후방 미감지로 차단):
       - NOMINAL: `cmd_vel_out = cmd_vel_in` (그대로 통과)
       - SLOW_DOWN: linear만 scale, `angular.z` 풀통과
       - STOP/WAIT: linear = 0, `angular.z` 통과 (sensor timeout 시는 전체 zero)
     - Sensor timeout(>0.5s): STOP
   - Pub: `/cmd_vel_safety` (`geometry_msgs/Twist`)
   - **핵심 원칙**: safety_stop은 planner가 아닌 **filter**. 입력 cmd_vel이 없으면 출력도 없음. SLOW 시 "원래 가려던 속도"를 그대로 비례 축소.

3. **임계값은 class 인지 가능 시 class별로 분리** (Phase 4 통합 후):
   - person/unknown/bicycle 별 SLOW/STOP 거리·TTC 분리.
   - **unknown은 person보다 덜 보수적이어선 안 됨**. unknown은 항상 보수적.
   - Phase 1 단계에서는 class 정보 없음 → unknown 기준의 보수적 임계 단일값 사용.

4. twist_mux 설치 및 구성:
   - 우선순위는 아키텍처 다이어그램 참조.
   - **emergency stop은 zero twist + lock 토픽 둘 다** (timeout 사이 이전 명령 통과 방지).
   - **현황: 미설치(인라인 게이트). 단일 소스라 지금은 불필요 → twist_mux+e-stop lock은 Phase 5(Nav2 도입, 다중 소스)로 연기 확정.**

verify (정량, N≥5회 반복 + 산출물 `verification/phase1_metrics.csv`):

- **정적 정지 게이트**: 로봇 0.3m/s 직진, actor 정지. STOP 시 oracle clearance ≥ (stop_clearance − margin). 5회 모두 충돌 0.
- **접근 물체 게이트 (velocity 필수)**: 로봇 정지/저속 + actor가 로봇 쪽으로 이동 → SLOW/STOP 반응. 상대속도 TTC 미구현이면 **이 게이트에서 실패**(= velocity 구현 강제). 자전거(고속) 시 STOP 거리 마진 ≥ 2m.
- **근거리 블라인드 존**: 0.8m 이내 진입 물체 처리 결과 기록 (LiDAR 미탐지 → D435i 보완 또는 한계 문서화).
- **false stop**: 장애물 없는 빈 구간 30초 주행 → STOP 0회.
- **timeout**: `lidar_obstacle_node` kill → 정지까지 < 0.5s (측정).
- **입력 0 → 출력 0** 단위 확인.
- twist_mux 도입 시: e-stop lock으로 nav/teleop 입력 차단 확인 (미도입이면 후순위 명시).

**파일**:

- `ros2_ws/src/perception_avoidance/perception_avoidance/lidar_obstacle_node.py` (또는 C++)
- `ros2_ws/src/perception_avoidance/perception_avoidance/safety_stop_node.py`
- `ros2_ws/src/perception_avoidance/config/safety_stop.yaml`
- `ros2_ws/src/perception_avoidance/config/twist_mux.yaml`

---

### Phase 2: STOP / WAIT / RESUME FSM (2~3일)

**목적**: 정지 후 보행자가 지나갈 때까지 대기, 통과 후 자동 재출발.

상태:

- **NOMINAL**: 위협 없음. 발행 안 함.
- **SLOW_DOWN**: 장애물 진입. 속도 감소.
- **STOP**: 정지.
- **WAIT**: 정지 후 일정 시간 동안 obstacle 후방 통과/이탈 확인.
- **RESUME**: 점진적으로 속도 복원.

전이 조건은 단일 distance가 아닌 복합 신호 + persistence:

- **clearance** (center 거리 아님)
- **TTC** (상대속도 기반)
- **relative velocity** sign (가까워지는 중인지)
- **persistence time** (chattering 방지, hysteresis margin)
- **sensor timeout** (즉시 STOP)

전이 예시:

- NOMINAL → SLOW_DOWN: clearance < slow_dist **또는** TTC < slow_ttc, persistence ≥ 0.1s
- SLOW_DOWN → STOP: clearance < stop_dist **또는** TTC < stop_ttc
- STOP → WAIT: STOP 조건 해제 시 진입 (계속 zero linear, 회전 통과)
- WAIT → RESUME: **clear**(in_stop 아님 + `ttc > slow_ttc`, 즉 접근 안 함)가 `wait_clear_duration` 유지 (✅ 구현; 별도 거리 임계 없이 zone 재사용)
- RESUME: linear을 `resume_time`(1.5s) 동안 0→full 램프, in_slow면 slow 스케일과 min → 완료 시 NOMINAL (✅ 구현)
- (any) → STOP: sensor timeout

verify (정량):

- `/safety/state` bag 캡처 → NOMINAL→SLOW→STOP→WAIT→RESUME **전이 순서 및 각 상태 지속시간**을 스크립트로 자동 검증.
- chattering: 경계 부근 30초에서 상태 전환 ≤ N회.
- 메트릭(N≥5회 통계): WAIT→통과 평균 시간, RESUME 후 속도 오버슈트, **false_stop 횟수(시나리오당 ≤ 1)**.

**파일**:

- `safety_stop_node`에 FSM 통합 (또는 `safety_fsm_node` 분리)

---

### Phase 2.5: Stuck Detection & Recovery (2일)

**목적**: 4족 로봇은 curb·작은 턱·다리 걸림 등으로 인해 회피보다 stuck이 더 자주 실패 원인. cmd_vel은 정상 발행되는데 실제로 움직이지 못하는 상태를 감지하고 복구.

상태 추가: **STUCK**, **ESTOP** (✅ safety_stop_node에 통합 구현)

감지 조건 (✅ 구현):

- **게이트 출력** `cmd_out_speed > stuck_v_min` (= 실제로 전진을 *명령* 중)이 `stuck_duration`(2.0s) 이상 지속이면서
  - ⚠️ `cmd_vel_in`이 아니라 **게이트 출력**으로 봐야 정상 STOP(장애물로 0 출력)을 stuck으로 오판하지 않음.
- **최근 `stuck_duration` 창(window)의 net 이동거리 < `stuck_disp_min`(0.1m)** (= 실제로 안 움직임)
  - ⚠️ **순간 odom 속도(`|v|`)는 사용 불가** — 라이브 검증에서 4족 gait 흔들림이 제자리에서도 `|v|`를 ±0.2 m/s로 출렁이게 만들어(65% 샘플이 0.03 초과, 연속 저속 최장 0.72s) 순간속도 기준은 stall 타이머가 계속 리셋돼 **영영 트리거 안 됨**. 창 net 이동거리는 앞뒤 흔들림이 상쇄돼 강건. 측정: 막힘 ~0.05 m/2s vs 정상보행 ~0.45 m/2s → `0.1m`로 9배 마진 분리.

복구 동작 (deterministic, **회전 전용** — 후방 미감지로 블라인드 후진 폐기):

1. zero cmd_vel `recover_settle_time` (0.3s) — settle
2. yaw 회전 (좌/우 중 LiDAR nearest-obstacle 더 먼 쪽) × `recover_rotate_time` (1.2s)
3. NOMINAL 복귀 (이후 기존 전진 명령이 새 heading으로 탈출)
4. 실제로 움직이면(창 net 이동거리 ≥ `stuck_disp_min`) 카운터 리셋; 복구해도 계속 stuck이면 카운터 증가, `max_recover_attempts`(3) 초과 시 **ESTOP 래치**(zero 고정 + 경고, 재시작까지 유지). 트리거·복구 완료 시 창 버퍼를 비워 re-arm(다음 트리거 전 창 재충전 강제).

파라미터 (config/safety_stop.yaml):

- `stuck_enable`, `stuck_v_min`, `stuck_disp_min`, `stuck_duration`, `recover_settle_time`, `recover_rotate_time`, `recover_yaw_rate`, `max_recover_attempts`

verify (정량, ✅ 완료):

- 오프라인(창 로직 복제): 막힘 creep→2.0s STUCK / 정상보행→미트리거 / 완전차단→3회→ESTOP / idle→미트리거 = **4/4 PASS**.
- 라이브 E2E(벽 밀착, 장애물 STOP 임계 무력화 + `enable_gate=true`): NOMINAL→**STUCK(att1)**→회전복구→NOMINAL→**STUCK(att2)**→복구. 순간속도 음수(−0.15)에도 정상 트리거. net 0.417 m/14s(막힘).
- false positive = 0 (정상 보행·idle을 STUCK으로 오판하지 않음).

**파일**:

- ✅ `safety_stop_node`에 STUCK/ESTOP 통합 (별도 노드 분리 안 함). 파라미터는 `config/safety_stop.yaml`에 포함.

---

### Phase 3: LiDAR 기반 단순 lateral 회피 (옵션, 4~5일)

**전제**: 인도 폭에 여유가 있을 때만 활성화. 기본값은 stop/wait.
**중요**: 본 노드는 **Nav2 도입(Phase 5) 전 임시 reactive 회피용**. Phase 5 이후엔 Nav2 local planner(MPPI/DWB)가 lateral path를 생성하므로 본 노드는 **기본 OFF**. 동시 활성 시 cmd_vel 충돌 발생.

작업:

1. 인도 경계 polygon: YAML로 정적 정의 (시뮬용)
2. lateral bias 계산: 장애물 회피 방향이 polygon 내인지 확인
3. polygon 밖이면 fallback → STOP
4. 회피 후 원래 heading 복귀 (RECOVERING 상태)

파라미터:

- `enable_lateral_avoidance` (기본 false)
- `min_sidewalk_clearance_to_avoid` (기본 0.5m)

verify (정량):

- 좁은 인도(polygon 폭 ~1.5m): lateral 0회(STOP만). 카운트로 확인.
- 넓은 구간: lateral 동작 시 polygon 이탈 0회.
- 회귀: Phase 1·2 게이트 재통과.

**파일**:

- `ros2_ws/src/perception_avoidance/perception_avoidance/lateral_avoid_node.py`
- `ros2_ws/src/perception_avoidance/config/sidewalk_boundaries.yaml`

---

### Phase 4: YOLO + D435i class fusion (3~4일)

**원칙**: safety-critical distance/TTC는 LiDAR 단독 유지. YOLO/D435i는 **분류 보조만**.

작업:

1. YOLO node 출력을 표준 메시지로 변환 (`vision_msgs/Detection2DArray`)
2. fusion: YOLO bbox → LiDAR cluster와 angular 매칭 (depth lookup 없이 각도/거리 비교로 1차 association)
3. cluster에 class 라벨 부여 (person, bicycle, vehicle, other)
4. class별 임계 정책:
   - **unknown은 person보다 덜 보수적이어선 안 됨**. unknown은 기본보수.
   - person: 기본 임계
   - bicycle/vehicle: person보다 보수적 (더 큰 stop_dist + 더 큰 stop_ttc)
   - **YOLO class는 안전거리/TTC를 *완화*하는 데 사용 금지**. 보수화에만 사용.
   - **정적/동적 "과감 기동"은 class 기반이 아님**: 정적 구조물 근처를 과감히 우회하는 안전한 근거는 **map(Phase 5)** 이지 단일 프레임 class가 아니다. class 오인식(서 있는 사람→구조물)으로 사람 근처에서 과감해지는 것을 막기 위해, class는 여기서도 *완화 없이* 보수화/정보 제공만. (정적 known-boldness는 Phase 5 static layer 참조)

**Fusion 필수 사전조건** (depth lookup을 안 쓰더라도 필요):

- camera ↔ lidar **extrinsic calibration** (TF 등록)
- `camera_info` intrinsics (bbox → ray 변환용)
- **approximate time synchronization** (예: `message_filters.ApproximateTimeSynchronizer`, `max_time_diff = 0.1s`)
- TF tree 정합성 (`base_link` 기준)

fusion 파라미터 예:

```yaml
fusion:
  target_frame: base_link
  max_time_diff: 0.1
  max_angular_error_deg: 5.0
  max_range_error_m: 1.0
```

verify (정량):

- `/obstacles/lidar`의 각 Detection3D에 class hypothesis 추가.
- 라벨 시나리오에서 bbox↔cluster 매칭율 ≥ X%, false association ≤ Y% (목표치 사전 설정).
- **안전 회귀**: class 도입 후 Phase 1·2 게이트 전부 재통과(임계 완화 없음 확인).

**카메라 스펙 변경 대비**:

- D435i depth lookup 기반 3D 위치 추정은 **사용하지 않음**.
- 카메라 교체 시 YOLO/분류만 영향. 안전 거리 로직은 무영향.
- 단, intrinsics/extrinsics는 카메라마다 재캘리브 필요.

**파일**:

- `ros2_ws/src/perception_avoidance/perception_avoidance/yolo_class_node.py`
- `ros2_ws/src/perception_avoidance/perception_avoidance/class_fusion_node.py`

---

### Phase 5: Nav2 통합 (충분히 안정화 후)

작업:

1. LiDAR SLAM으로 2D 맵 생성
2. Nav2 + MPPI/DWB local planner 구성 (MPPI Cost Critic + 직사각형/SE2 footprint, Go2 0.70×0.31)
3. **정적/동적 장애물 분리 (핵심 설계 원칙)**:
   - **정적(건물·메일박스·기둥) → SLAM/static map layer에 "known"으로 반영 → planner가 footprint 여유만 두고 정상 속도로 *우회***. 반응형 STOP/WAIT/TTC 대상 아님. (= "정적 근처를 과감히 기동")
   - **동적(보행자·자전거) → costmap obstacle layer(clearing) + 반응층 safety_stop(TTC/STOP/WAIT)**.
   - **"정적이라 과감해도 된다"의 안전한 근거는 map(시간 누적으로 검증된 점유)이지 velocity·단일프레임 class가 아니다.** 서 있는 사람과 메일박스는 속도·형상으로 구분 불가 → 반응층/class에서 미리 과감해지면 안 됨. map은 움직이는 agent를 지속 clearing하므로 사람이 static layer에 굳지 않는다.
   - **근거(freezing robot problem)**: 정적 클러터에 과조심하면 *오히려* 다중 액터 충돌·정체 위험이 커진다 → map+planner로 정적을 매끄럽게 우회해 계속 움직이는 것이 동적 안전에도 유리.
4. **safety_stop_node는 Nav2 출력 *아래*에 최상위 안전 레이어로 유지**:
   - 흐름: `Nav2 cmd_vel → safety_stop_node (filter) → twist_mux → CHAMP`
   - Nav2 Collision Monitor와 역할이 유사 (Jazzy 공식 패키지 `nav2_collision_monitor` 참고).
   - 선택: 직접 구현한 safety_stop을 유지 vs `nav2_collision_monitor` 교체 — 비교 평가 후 결정.
5. **`lateral_avoid_node` 기본 OFF** (Nav2 planner와 lateral 역할 중복 회피).

**개발 스캐폴딩 옵션** (실 환경 미적용):

- actor ground-truth pose를 PointCloud2로 합성해 Nav2 costmap에 직접 입력하는 디버그 모드.
- 기본 OFF, perception 우회 디버깅 시만 ON.
- sim2real에 영향 없음 (launch arg로 분리).

verify (정량):

- safety_stop을 Nav2 아래 둔 상태에서 Phase 1·2 게이트 전부 재실행 통과.
- `nav2_collision_monitor`와 A/B 비교 메트릭(min_clearance, false_stop, 정지 거리).

---

## 시나리오 테스트 (Phase 1 이후 누적)

| 시나리오                 | 설명                        | 핵심 메트릭                    |
| ------------------------ | --------------------------- | ------------------------------ |
| 정면 접근                | 보행자가 로봇을 향해 걸어옴 | 반응 시간, 최소 clearance      |
| 횡단                     | 보행자가 수직 방향 횡단     | clearance, 정지 정확도         |
| 다중 정적 장애물         | 2m 간격 3~4개               | Phase 3 활성 시 통과율         |
| 고속 접근                | 빠른 객체                   | 긴급정지 작동 여부, TTC 정확도 |
| 인도 경계 (Phase 3 활성) | 좌측 장애물 우측 회피 유도  | polygon 이탈 여부              |
| 회복 (Phase 3 활성)      | 회피 후 원래 경로 복귀      | 복귀 시간, heading 안정성      |

**통과 기준 (Phase 1~2, 각 항목 N≥5회 + 산출물)**:

- `clearance > 0` (즉 충돌 0회) 항상
- 최소 clearance > 0.2m
- 시나리오 완료 시간 < 명목 시간의 2배
- sensor timeout 시 정지 < 0.5s 내
- **false stop 횟수 ≤ 시나리오당 1회** (장애물 없을 때 멈춤 = 실패)
- **접근 물체 게이트**: 로봇 정지 상태에서도 상대속도 TTC로 반응 (정적 TTC만으론 실패)
- **근거리 블라인드 존**: 0.8m 이내 진입 물체 처리 또는 한계 명시
- **stuck 발생 시 자동 복구** (Phase 2.5 활성 시)

**메트릭 분류 (수집 대상)**:

```text
safety:
  min_clearance, min_ttc, collision_count, near_miss_count, emergency_stop_count

comfort:
  max_deceleration, jerk, stop_duration, resume_time

perception:
  first_detection_distance, detection_latency,
  missed_detection_count, false_stop_count

task:
  scenario_success, total_time, path_deviation, completion_rate
```

CSV/bag 저장은 사용자 선택. 본 phase 0.5의 oracle은 ROS 로그 + 종료 시 Summary만 출력.

---

## 레이턴시 분석 (재정리)

- L1 LiDAR 스캔 주기: ~100ms
- LiDAR cluster + safety decision: 10~20ms
- cmd_vel → CHAMP: ~5ms (200Hz)
- **LiDAR-only 안전 정지 경로**: 약 120~140ms

YOLO 추론(30~50ms)은 **분류 보조 경로만** 통과하므로 안전 정지 레이턴시에 영향 없음. 카메라 스펙이 바뀌어도 안전 거리/정지 로직 무영향.

자전거 20km/h(5.6m/s) 시나리오:

- 안전 정지 거리 ≈ v · t_total + 제동 거리. t_total ≈ 0.15s → 최소 0.84m + 제동 거리.
- → STOP zone 임계는 충분한 마진(예: 2m) 권장.

---

## 리스크와 대응 (재정리)

| 리스크                                         | 대응                                                               |
| ---------------------------------------------- | ------------------------------------------------------------------ |
| 카메라 스펙 변경 (D435i → 다른 모델)           | 거리/TTC는 LiDAR 단독. 카메라 파라미터 YAML 분리                   |
| Actor pose plugin이 다중 actor에서 누락        | World-level plugin이 매 step entity 재스캔 (안정화 후 entity 캐시) |
| `/odom` drift로 평가 부정확                    | 평가는 Gazebo ground-truth pose 사용                               |
| 좁은 인도 lateral 회피로 curb 충돌             | Phase 3 기본 OFF, polygon 제약                                     |
| Sensor timeout 시 무한 주행                    | safety_stop_node에 watchdog → 정지                                 |
| Nav2 costmap 업데이트 지연 (Phase 5)           | safety_stop_node를 최상위 우선순위로 유지                          |
| Velodyne 기반 가정 코드 잔존                   | grep으로 `velodyne` 잔재 제거 확인                                 |
| **자기 다리/몸통이 obstacle로 잡힘**           | Phase 1 전처리에 self-filter + ROI crop 필수                       |
| **장애물 없는데 멈춤 (false stop)**            | hysteresis, persistence, class별 보수 임계. 메트릭으로 추적        |
| **로봇이 cmd 받지만 안 움직임 (stuck)**        | Phase 2.5 stuck detection + deterministic recovery                 |
| **자전거 등 빠른 객체에 STOP zone 부족**       | class 인지 후 더 큰 stop_dist/stop_ttc. unknown은 보수적 기본값    |
| **후진 시 후방 장애물 미감지**                 | 후진 기본 차단(회전 우선). L1 ~360° → 필요 시 후방 corridor 추가(ROI 확장). Phase 5 costmap이 360° 누적 |
| **정적 TTC로 접근 물체 미반응 (정지 상태)**    | velocity 추정 + 상대속도 TTC를 Phase 1 필수 게이트로 격상          |
| **LiDAR 근거리 블라인드 존(~0.8m)**            | D435i depth(near 0.105m) 보완 또는 재출발 시 한계 문서화           |
| **oracle 단일 actor만 평가**                   | 다중 actor 리스트로 확장 (군중/다중 장애물 메트릭 정확도)          |
| **camera-lidar fusion 시 timestamp 불일치**    | `ApproximateTimeSynchronizer` + max_time_diff. extrinsic TF 등록   |
| **emergency stop 사이 이전 cmd 통과**          | twist_mux **lock 토픽**으로 상위 입력 차단                         |
| **Nav2와 lateral_avoid 동시 활성 시 cmd 충돌** | Phase 5 진입 시 lateral_avoid 기본 OFF                             |

---

## 주요 수정/생성 파일 요약

**기존 파일 수정**:

- `ros2_ws/src/go2_simulation/worlds/small_city.sdf` — `actor_pose_publisher` plugin 등록
- `ros2_ws/src/go2_simulation/config/pose_bridge.yaml` — `/world/default/actor_pose/info` bridge 한 줄 추가
- `ros2_ws/src/go2_simulation/launch/unitree_go2_launch_small_city.py` — perception_avoidance 노드 실행 (옵션)
- ~~`ros2_ws/src/go2_simulation/scripts/collision_monitor_node.py`~~ — ✅ 삭제됨, `perception_avoidance/collision_oracle_node.py`로 이전

**새로 생성**:

- `ros2_ws/src/go2_simulation/plugins/actor_pose_publisher/` — C++ system plugin
- `ros2_ws/src/perception_avoidance/` — 단일 패키지 (이후 분리 검토)
  - `collision_oracle_node.py` (Phase 0.5)
  - `lidar_obstacle_node.py` (Phase 1, **self-filter + ROI crop 포함**)
  - `safety_stop_node.py` (Phase 1~2, **cmd_vel gate** + FSM + STUCK 상태)
  - `lateral_avoid_node.py` (Phase 3, 옵션, Phase 5 후 OFF)
  - `yolo_class_node.py`, `class_fusion_node.py` (Phase 4)
  - `config/safety_stop.yaml` — class별 SLOW/STOP 거리·TTC, hysteresis
  - `config/twist_mux.yaml` — 우선순위 + **lock 토픽**
  - `config/sidewalk_boundaries.yaml`
  - `config/camera.yaml` — 카메라 스펙 분리 (D435i 기본값, 교체 대비)
  - `config/self_filter.yaml` — Go2 body/leg 박스 정의 (Phase 1)
  - ~~`config/stuck_recovery.yaml`~~ → `safety_stop.yaml`에 통합 (Phase 2.5, STUCK/ESTOP)
  - `verification/` — phase별 자동화 시나리오 러너 + `phaseN_metrics.csv` 산출물 (검증 공통 원칙)

**제거된 항목** (이전 plan에 있었으나 폐기):

- ~~`obstacle_avoidance_msgs` 신규 패키지~~ → 표준 메시지 사용
- ~~`lidar_obstacle_detector`, `sensor_fusion`, `avoidance_controller`, `collision_monitor` 5개 패키지 분할~~ → 단일 패키지
- ~~Velodyne VLP-16 의존~~ → L1 only
- ~~D455 기반 설계~~ → D435i (변경 가능성 명시)
- ~~trunk contact sensor 기반 충돌~~ → distance/clearance oracle
- ~~Actor → 물리 model 변환 (TrajectoryFollower)~~ → 실패 사례. Actor 유지 + plugin 기반 pose 발행

---

## 첫 실행 순서 권장

1. **Phase 0.5 plugin 구현** → actor pose 발행 확인 (`ros2 topic echo /world/default/actor_pose/info`)
2. **collision_oracle_node** → ground-truth clearance 메트릭 수집
3. **Phase 1 LiDAR safety stop** → 실제 회피의 시작
4. 이후 메트릭으로 각 phase 성능 비교

이 순서가 "회피 알고리즘 ↔ 평가 인프라" 분리를 강제하고, 향후 카메라/센서 스펙 변경에도 회피 로직 안정성을 유지하는 가장 단순한 경로다.
