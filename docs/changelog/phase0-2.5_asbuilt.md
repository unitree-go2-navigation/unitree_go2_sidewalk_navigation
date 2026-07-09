# Phase 0: 센서 인프라 보완 - 변경 로그

## 목표
D455 깊이 카메라 토픽을 Gazebo bridge에 추가하여 모든 센서 데이터가 ROS2에서 사용 가능하도록 함.

---

## 변경 기록

### [1] RealSense D435 RGBD Camera 센서 추가 — unitree_go2_gazebo.xacro
- **파일**: `ros2_ws/src/go2_simulation/urdf/unitree_go2_gazebo.xacro`
- **변경 내용**: 기존 RGB camera (calibration 기반)를 D435 스펙의 RGBD camera로 교체
- **이유**: 장애물까지의 거리 정보가 필요. 실제 하드웨어(D435)와 매칭.
- **세부사항**:
  - sensor type: `camera` → `rgbd_camera`로 변경
  - topic namespace: `rgb_image` → `d435` (d435/image, d435/depth_image, d435/points, d435/camera_info)
  - D435 스펙 적용: hfov=69.4° (1.211 rad), depth near=0.105m, depth far=10m
  - 기존 distortion 파라미터 제거 (D435 기본 사용)
  - update_rate: 30Hz → 15Hz (depth 처리 부하 고려)
- **참고**: 실제 하드웨어가 D435이므로 D455는 옵션으로만 유지

### [2] Gazebo Bridge에 D435 토픽 추가 — launch 파일
- **파일**: `ros2_ws/src/go2_simulation/launch/unitree_go2_launch_small_city.py`
- **변경 내용**: gazebo_bridge arguments에 D435 RGBD 토픽 추가, 기존 `/rgb_image` bridge 제거
- **이유**: Gazebo 내 센서 데이터를 ROS2 토픽으로 전달해야 perception 노드에서 사용 가능
- **추가 토픽**:
  - `/d435/image` → `sensor_msgs/msg/Image` (RGB)
  - `/d435/depth_image` → `sensor_msgs/msg/Image` (depth)
  - `/d435/points` → `sensor_msgs/msg/PointCloud2` (depth point cloud)
  - `/d435/camera_info` → `sensor_msgs/msg/CameraInfo`
- **제거**: 기존 `/rgb_image` bridge 제거 (D435 `/d435/image`로 대체)

### [3] D435 → D435i 변경 + 내장 IMU 추가
- **파일**: `ros2_ws/src/go2_simulation/urdf/unitree_go2_gazebo.xacro`, `launch/unitree_go2_launch_small_city.py`, `rviz/rviz.rviz`
- **변경 내용**:
  - 카메라 모델: D435 → D435i (실제 하드웨어 매칭)
  - RGB hfov: 1.211 rad (69.4°) → 1.518 rad (87°) — D435i는 더 넓은 FOV
  - topic namespace: `/d435/` → `/d435i/`
  - D435i 내장 IMU 센서 추가 (BMI055, 200Hz, topic: `/d435i/imu`)
  - IMU noise: gyro stddev=1e-3, accel stddev=2e-2
  - bridge에 `/d435i/imu` 토픽 추가
- **이유**: 실제 구비된 카메라가 D435i. IMU 내장으로 카메라 자체 자세 추정 가능.

### [4] Velodyne 제거, Go2 기본 LiDAR(unitree_lidar)만 사용
- **파일**: `ros2_ws/src/go2_simulation/urdf/unitree_go2_robot.xacro`, `launch/unitree_go2_launch_small_city.py`
- **변경 내용**:
  - `velodyne.xacro` include 주석처리
  - launch bridge에서 `/velodyne_points/points` 토픽 제거
- **이유**: 실제 Go2에 Velodyne 미탑재. 기본 4D LiDAR L1 (`/unitree_lidar/points`)만 사용.
- **유지**: `/unitree_lidar/points` bridge는 그대로 유지

---

## Phase 1 변경 기록

### [4] Actor → Collision Model (TrajectoryFollower) 교체 — small_city.sdf
- **파일**: `ros2_ws/src/go2_simulation/worlds/small_city.sdf`
- **변경 내용**: kinematic-only actor를 collision 가능한 model + TrajectoryFollower 플러그인으로 교체
- **이유**: Actor는 Gazebo Harmonic에서 물리 충돌 불가. 충돌 감지 및 회피 테스트를 위해 실체가 있는 model 필요.
- **세부사항**:
  - cylinder collision (r=0.3m, h=1.7m) — 사람 크기 근사
  - 녹색 시각화 (디버깅용)
  - TrajectoryFollower로 x=-5 ~ x=20 왕복 (인도 위)
  - force=30, torque=10 (보행 속도 ~0.5m/s 목표)

### [5] trunk collision 이름 명시 — unitree_go2_robot.xacro
- **파일**: `ros2_ws/src/go2_simulation/urdf/unitree_go2_robot.xacro`
- **변경 내용**: trunk link의 collision에 `name="trunk_collision"` 명시적 추가
- **이유**: Gazebo world-level contact 감지 시 collision 이름으로 필터링 가능하도록

### [6] trunk_contact sensor 추가 후 제거 (롤백)
- **파일**: `unitree_go2_gazebo.xacro`, `unitree_go2_launch_small_city.py`
- **변경 내용**: trunk_contact sensor 추가했다가 제거
- **이유**: 실제 Go2에 body contact sensor 없음. 실기체에 있는 센서만 사용.
- **대안**: 충돌 평가는 Gazebo world-level contact (`gz topic -e -t /world/default/contact`)로 모니터링

### [7] TrajectoryFollower model → Actor 복원 (롤백)
- **파일**: `ros2_ws/src/go2_simulation/worlds/small_city.sdf`
- **변경 내용**: 물리 model(TrajectoryFollower) 접근 폐기, 원래 actor로 복원
- **이유**: 
  - Gazebo actor는 공식적으로 collision 불가 (물리에 참여하지 않음)
  - 물리 model로 교체 시 중력/마찰 때문에 넘어지고 부자연스러운 자세 발생
  - Actor는 GPU 센서(카메라, LiDAR)에 감지되므로 회피 알고리즘 테스트에 충분
- **충돌 평가 방법 변경**: 
  - 물리적 contact 대신 로봇-actor 간 거리 기반으로 충돌 판정
  - Gazebo에서 actor pose 구독 → 로봇 odom과 비교 → threshold 이하면 "충돌"로 기록

### [8] 거리 기반 충돌 모니터 노드 구현
- **파일**: `ros2_ws/src/go2_simulation/scripts/collision_monitor_node.py`
- **변경 내용**: 로봇-actor 간 거리를 실시간 모니터링하여 충돌 판정하는 ROS2 노드 작성
- **동작 방식**:
  - `/world/default/dynamic_pose/info` (bridge 추가) → actor pose 추출
  - `/odom` → 로봇 위치
  - 거리 < 0.5m → COLLISION 이벤트 발행 (`/collision_events`)
  - 거리 < 2.0m → WARNING 로그
  - 노드 종료 시 총 충돌 횟수, 최소 이격거리 요약 출력
- **파라미터**: actor_name, collision_threshold(0.5m), warning_threshold(2.0m)
- **launch bridge 추가**: `/world/default/dynamic_pose/info` → TFMessage

### [옵션] Actor collision model 방식 (미래 사용 가능)
- **상태**: 현재 미사용. 추후 물리적 충돌 반응이 필요할 때 적용 가능
- **방식**: Actor를 `<model>` + TrajectoryFollower로 교체하여 물리 collision 활성화
- **문제점**: 
  - 중력/마찰으로 넘어짐 → 자세 제약(joint constraint) 필요
  - 걷기 애니메이션 없음 → cylinder/mesh가 미끄러지듯 이동
  - force/mass 튜닝 필요
- **해결 방향 (향후)**:
  - 6DOF joint으로 roll/pitch 고정, yaw/translation만 허용
  - 또는 kinematic model (static=false, 직접 pose 설정하는 커스텀 플러그인)
  - Gazebo Harmonic 향후 업데이트에서 actor collision 지원 가능성 있음

---

## Phase 1 보강 기록 (2026-06-28)

### [9] Collision oracle 정본 일원화 + 다중 actor 지원
- **파일**: `perception_avoidance/perception_avoidance/collision_oracle_node.py` (신규), `go2_simulation/scripts/collision_monitor_node.py` (삭제), `go2_simulation/CMakeLists.txt`, `launch/unitree_go2_launch_small_city.py`, `perception_avoidance/setup.py`, `perception_avoidance/package.xml`
- **변경**: 평가 oracle를 `go2_simulation` 스크립트 → `perception_avoidance` 노드로 이전. `child_frame_id != robot_name`인 transform을 모두 actor로 자동 처리(다중 actor 최소 clearance). 구 스크립트/CMake 설치 항목/launch 참조 제거.
- **이유**: 평가 인프라를 단일 패키지로 통일. 군중/다중 장애물 시나리오 메트릭 확보.

### [10] LiDAR frame-to-frame 속도 추정 추가
- **파일**: `perception_avoidance/perception_avoidance/lidar_obstacle_node.py`
- **변경**: cluster centroid를 greedy NN + EMA로 프레임 간 추적 → `/obstacles/lidar/velocity`(`geometry_msgs/PoseArray`, Detection3DArray와 동일 stamp·인덱스 정렬)로 발행.
- **이유**: 상대속도 기반 TTC 입력. centroid를 base_link에서 추적해 그 미분이 곧 v_rel(obstacle−robot).

### [11] safety_stop 상대속도 TTC로 교체
- **파일**: `perception_avoidance/perception_avoidance/safety_stop_node.py`, `config/safety_stop.yaml`
- **변경**: `ttc = clr / robot_vx`(정적) → `closing = −dot(p_rel, v_rel)/|p_rel|`, `closing = max(closing, robot_vx)`로 클램프. velocity 토픽 stamp 매칭, `closing_eps` 파라미터 추가.
- **이유**: 로봇 정지/저속 상태에서 다가오는 보행자/자전거에 반응(정적 fallback 대비 절대 덜 보수적이지 않음).

### [12] config/문서 정합
- **파일**: `go2_simulation/config/pose_bridge.yaml`, `collision-unified-rain.md`
- **변경**: 구독처 없는 `dynamic_pose/info` bridge 제거. 계획서 Phase 1 ROI z 표기를 config(z_max=0.5)에 맞춤.
- **이유**: 잔재 정리 + 문서·코드 일치.

### [13] TTC 항상 inf 버그 수정 (velocity 전달 경로)
- **파일**: `lidar_obstacle_node.py`, `safety_stop_node.py`, `config/safety_stop.yaml`
- **증상**: 상태전이는 정상인데 `/safety/state`의 ttc가 다가올 때도 항상 inf.
- **원인**: detections/velocity를 별도 토픽으로 발행 + stamp 매칭 → 단일 스레드 executor에서 `obs_cb`가 해당 프레임 velocity가 버퍼에 들어오기 전에 실행돼 항상 `None`(정적 fallback) → 로봇 정지 시 `closing=0` → ttc=inf.
- **수정**: 상대속도(vx,vy)를 Detection3D의 `results[0].pose.covariance[0/1]`에 임베드해 위치와 원자적으로 전달. safety_stop은 같은 메시지에서 읽음(토픽 간 순서 경쟁 제거). 디버그용 `/obstacles/lidar/velocity` PoseArray는 유지, safety_stop의 velocity 구독/버퍼/`vel_topic` 파라미터 제거.
- **검증**: 오프라인 수식 테스트 — 정지 로봇+접근 0.5m/s→ttc 4.9s, 멀어짐→inf(정상), 자전거 5.6m/s→0.97s.

### [14] 회전 시 false STOP 수정 (TTC-primary + 회전 통과)
- **파일**: `safety_stop_node.py`, `config/safety_stop.yaml`
- **증상**: 기체를 회전시켜 정적 구조물(벽/건물)을 마주보면 닿지 않았는데 STOP으로 전이. STOP이 angular.z까지 죽여 회전으로 벗어날 수 없는 교착.
- **원인**: `in_stop = (clr ≤ stop_clearance 1.5) or ...` — 거리만으로 STOP 강제. 회전 중 corridor에 들어온 정적 물체가 거리 조건으로 STOP. + STOP이 전체 cmd zero.
- **수정 (A+B)**: ① 거리 STOP을 작은 `emergency_clearance`(0.4m) backstop으로 축소, 큰 거리 반응은 TTC(접근 중) 기준 → 회전 중 정적 벽은 closing≈0→TTC=∞→STOP 안 함. ② SLOW/STOP에서 `angular.z` 항상 통과(전진/측면/후진만 차단) → 회전으로 탈출. `stop_clearance` 파라미터 제거(→ emergency_clearance), `_scale_for_slow` 하한을 emergency_clearance로.
- **검증**: 오프라인 로직 시뮬 — 벽 1.5m/0.8m 제자리 회전→STOP 안 함, angular 통과; 0.35m(emergency 내)도 angular 통과로 탈출; 전방 접근은 감속→STOP 유지; 빠른 보행자(ttc<1)→STOP.

### [15] emergency_clearance 튜닝 (0.4 → 0.25)
- **파일**: `config/safety_stop.yaml`
- **변경**: 정적 장애물은 회전반경(동체 외접 ~0.36m) 밖이면 안전하다는 점에 근거해 거리 STOP backstop을 0.25m로 축소. 근거: 회전 안전(~0.1) + 전방 동체 과소평가 보정(~0.08, robot_radius 0.25 < 실제 ~0.33) + 보행/cluster/localization 노이즈 마진(~0.07). floor ~0.2.
- **유지**: 전방 접근 안전은 `_scale_for_slow`(emergency에서 속도 0) + TTC가 담당하므로 backstop 축소가 전진 충돌 위험을 키우지 않음. python 기본값 0.4는 yaml 미로드 시 fallback.

### [16] 직사각형 footprint 도입 (원형 robot_radius 폐기)
- **파일**: `safety_stop_node.py`, `config/safety_stop.yaml`
- **변경**: 전방 clearance를 `front_x − robot_radius(0.25, 원형)` → `front_x − robot_half_length(0.35)`. Go2 실제 비율(0.70×0.31 → 반-extent 0.35×0.155)에 맞춤. 측면은 기존대로 robot_half_width(0.155)+corridor_margin. corridor_margin은 측면 전용(불확실성), 종방향 안전 여유는 emergency_clearance.
- **이유**: 원형 0.25가 전방 동체 길이(~0.35)를 과소평가 → 전방 clearance 낙관적이던 문제 해소. STOP 경계 = 전면 0.60m(=0.35+0.25)로 물리적으로 정확.
- **검증**: 오프라인 — 전면 0.55/0.60m→STOP, 0.80m→SLOW(회전 통과 유지).

### 후방 장애물 관련 (질문 답변 기록)
- 현재 `safety_stop`은 **후진을 차단**(후방 미감지) → "후진 중 후방 충돌"이 구조적으로 발생 안 함. 회피는 STOP/WAIT + 회전.
- **Phase 2.5 stuck recovery의 블라인드 후진은 잠재 위험** → 회전 우선 또는 후방 corridor 추가 후 사용 권장(리스크 표/Phase 2.5에 기록).
- 별도 "후방 인식" phase는 없음. **Phase 5(Nav2 local costmap)** 가 360° 장애물 누적으로 사실상 커버. 단기엔 L1(~360°) ROI 확장으로 후방 corridor 추가 가능.

---

## Phase 2 기록 (2026-06-29)

### [17] STOP/WAIT/RESUME FSM 구현 (3상태 → 5상태)
- **파일**: `safety_stop_node.py`, `config/safety_stop.yaml`
- **변경**: 기존 NOMINAL/SLOW_DOWN/STOP에 **WAIT, RESUME** 추가. `_next_state()`로 전이 일원화, `_scaled_cmd()`로 출력 DRY화.
  - `STOP→WAIT`: STOP 조건 해제 시 진입(계속 zero linear, 회전 통과).
  - `WAIT→RESUME`: clear(`not in_stop` AND `ttc > slow_ttc`)가 `wait_clear_duration`(1.0s) 유지. 별도 거리 임계 없이 기존 zone 재사용 → `wait_clear_clearance` 파라미터 제거.
  - `RESUME`: `_resume_scale`을 `resume_time`(1.5s) 동안 0→1 선형 램프, in_slow면 slow 스케일과 min → 완료 시 NOMINAL. (4족 급가속 전도 방지)
  - 하드 STOP은 어느 상태에서나 우선. SLOW만 거친 경우(완전 정지 안 함)는 WAIT/RESUME 없이 즉시 NOMINAL.
- **이유**: 보행자 통과 대기 후 자동 재출발 + 부드러운 속도 복원.
- **검증**: 오프라인 FSM 시뮬 — 접근(ttc<1)→STOP→(보행자 전방)WAIT→(클리어)WAIT 1s dwell→RESUME 램프(0.02→0.30, 1.5s)→NOMINAL. 빌드 성공, `wait_clr` 잔여 0.

### [결정] twist_mux + e-stop lock → Phase 5 연기
- 현재 cmd_vel 소스가 teleop 단일이라 중재 불필요. 다중 소스(Nav2)가 들어오는 **Phase 5에서 twist_mux + e-stop lock 함께 구성** 확정.

### [결정] 정적/동적 장애물 분리 원칙 문서화 (Phase 4·5)
- **파일**: `collision-unified-rain.md` (Phase 4, Phase 5)
- **내용**: 정적(건물/메일박스)=map으로 known 처리 → planner 정상속도 우회(과감), 동적=반응층 TTC/STOP/WAIT 전용. **"과감의 안전한 근거는 map(Phase 5)이지 velocity·단일프레임 class가 아님"**(서 있는 사람=메일박스 구분 불가). class는 완화 금지·보수화만 유지. 근거: freezing robot problem(과조심→다중 액터 충돌·정체 위험 증가).
- **반응 코드 변경 없음**: 현 reactive 단계는 동적 전용 안전망으로 유지, 정적 known-boldness는 Phase 5에서 구현.

### [18] TTC 히스테리시스 배선 (미사용 파라미터 활성화)
- **파일**: `safety_stop_node.py` (`_danger_levels`)
- **변경**: 거리(clr)에만 적용되던 히스테리시스를 TTC에도 대칭 적용. STOP/SLOW 상태일 때 `stop_ttc`/`slow_ttc`에 `hysteresis_ttc(0.5s)`를 더해 풀려날 때 더 큰 시간 마진 요구.
- **이유**: TTC = clearance/closing 이고 closing은 노이즈 큰 속도 추정 기반 → 두 신호 중 TTC가 더 떨림. 정작 히스테리시스가 덜 떨리는 clr에만 있던 비대칭 해소. (`hysteresis_ttc`는 초기 PoC부터 선언됐으나 미배선이었음 — 의도적 생략 아님)
- **검증**: 오프라인 — ttc가 1.0 경계 진동 시 STOP 토글 10회→1회로 채터링 제거. 빌드 성공.

---

## Phase 2.5 기록 (2026-06-30)

### [19] Stuck Detection & Recovery 구현 (STUCK/ESTOP 상태 추가)
- **파일**: `safety_stop_node.py`, `config/safety_stop.yaml`
- **변경**: safety_stop에 STUCK/ESTOP 상태 통합. 별도 노드/`stuck_recovery.yaml` 안 만들고 통합.
  - **감지**: `_cmd_out_speed > stuck_v_min`(게이트가 실제 전진을 명령 중) AND `odom_speed < stuck_v_stall`(미이동)이 `stuck_duration`(2.0s) 지속. ⚠️ `cmd_vel_in`이 아니라 **게이트 출력** 기준 → 정상 STOP(장애물로 0 출력)을 stuck으로 오판 안 함.
  - **복구(회전 전용)**: settle(0.3s) → LiDAR nearest-obstacle 더 먼 쪽으로 제자리 회전(yaw_rate 0.5 × 1.2s) → NOMINAL. 후방 미감지로 **블라인드 후진 폐기**(앞 결정 반영). obs_cb에 좌/우 nearest 거리 추가로 회전 방향 선택.
  - **ESTOP**: 실제 이동 시 attempt 리셋, 복구 반복 실패로 `max_recover_attempts`(3) 초과 시 ESTOP 래치(zero 고정 + 경고).
  - STUCK은 노드가 능동적으로 회전 명령을 내는 유일 예외(시간 제한). `/safety/state`에 `stall=..s att=..` 추가.
- **이유**: 4족은 curb·턱·다리 걸림 stuck이 회피보다 잦은 실패 원인.
- **검증**: 오프라인 — ①정상 STOP(출력0)은 stuck 미트리거 ②전진+미이동→2.0s STUCK→회전복구→3.5s NOMINAL ③반복 실패→att 4(>3)→ESTOP. 빌드 성공.

### [20] Stuck 감지 신호 교체: 순간 odom 속도 → 창(window) net 이동거리 (라이브 검증에서 결함 발견)
- **파일**: `safety_stop_node.py` (import `deque`, `odom_cb`, `tick` stuck 블록, 헬퍼 `_record_pose`/`_window_disp`, state init), `config/safety_stop.yaml`
- **발단**: 라이브 시뮬에서 벽 밀착 후에도 STUCK이 **절대 트리거 안 됨**. 원인 계측: 4족 gait 흔들림이 순간 odom `|v|`를 제자리에서도 0~0.357 m/s(평균 0.108)로 출렁이게 함 → 65% 샘플이 `stuck_v_stall`(0.03) 초과, 연속 저속 최장 0.72s(<2.0s 필요) → `stall_timer` 매번 리셋. **순간속도는 4족에서 stall 신호로 부적합**.
- **변경**:
  - `robot_speed = hypot(vx,vy)` 제거. `odom_cb`가 위치(`robot_x/y`)를 기록.
  - `_pose_hist`(deque)에 매 tick 위치 저장, `_window_disp()`가 최근 `stuck_duration` 창의 net 이동거리 반환.
  - stall 판정: `_cmd_out_speed > stuck_v_min`(명령 중)이 `stuck_duration` 지속 **AND** 창 net 이동거리 `< stuck_disp_min`(0.1m). 진행(≥0.1m) 시 attempt 리셋. 트리거·복구완료 시 창 clear로 re-arm.
  - 파라미터 `stuck_v_stall`(0.03 m/s) 제거 → `stuck_disp_min`(0.1 m) 추가.
- **근거 수치**: 막힘 ~0.05 m/2s vs 정상보행 ~0.45 m/2s → 0.1m로 9배 마진.
- **검증**:
  - 오프라인(창 로직 복제) 4/4 PASS: A 막힘 creep→2.0s STUCK, B 정상보행→미트리거, C 완전차단→3회→ESTOP(12.56s), D idle→미트리거.
  - **라이브 E2E**: NOMINAL→STUCK(att1, 7.86s)→회전복구→NOMINAL(9.36s)→STUCK(att2, 11.41s)→복구(12.92s). 순간 `v=-0.15`에도 정상 트리거. net 0.417 m/14s. `colcon build` 성공.
  - **라이브 E2E 전체 체인(teleop 지속 push, ESTOP 도달)**: att 1→2→3 회전복구 모두 탈출 실패(창 이동거리 <0.1m라 카운터 리셋 안 됨) → 4번째 감지에서 att 4>3 → **ESTOP 래치** 확인(`/safety/state = ESTOP|...att=4`, 재시작까지 유지). 각 복구 1.5s(settle 0.3+rotate 1.2). gait 순간속도 노이즈에도 att 리셋 없이 도달 = 창 로직이 노이즈에 안 속음을 재확인.
- **비고**: 라이브 테스트는 `enable_gate=true` + 장애물 STOP 임계 무력화(`slow/emergency_clearance`,`slow/stop_ttc`=-100)로 벽 밀착을 NOMINAL 유지시켜 stuck 경로만 격리(테스트 전용 config, 회피 없음).
