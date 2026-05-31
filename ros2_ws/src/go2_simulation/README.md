# go2_simulation

`go2_simulation`은 Unitree Go2 로봇을 Gazebo Sim Harmonic에서 실행하기 위한
ROS 2 Jazzy 시뮬레이션 패키지입니다. 기본 CHAMP 보행 제어기, ROS 2 position
controller, 프로젝트 전용 월드, Gazebo 모델, RViz 설정을 한 번에 실행할 수
있습니다.

이 문서의 첫 번째 목표는 새로 저장소를 받은 사용자가 `small_city` 월드에서
Go2를 띄우고, controller가 활성화되었는지 확인하고, 로봇이 실제로 움직이는
모습을 Gazebo GUI에서 직접 확인하게 하는 것입니다.

## 주요 기능

- Gazebo Sim Harmonic 기반 Unitree Go2 시뮬레이션
- CHAMP 기반 4족 보행 제어
- `JointGroupPositionController` 기반 12개 관절 position command
- `default`, `small_city`, `campus`, `TIbuilding` 월드
- `/unitree_lidar/points`를 기본으로 표시하는 프로젝트 전용 RViz 설정
- `small_city` 인도 주행 검증을 위한 spawn 위치와 collision 조정
- joint order 및 lateral gait 진단 스크립트

## 시스템 요구사항

| 항목 | 버전 |
| --- | --- |
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Sim Harmonic |
| Build tool | `colcon` |

필요한 ROS 의존성은 가능하면 `rosdep`으로 설치합니다. 키보드 조작을 사용할
경우 `teleop_twist_keyboard`도 설치합니다.

```bash
sudo apt update
sudo apt install python3-rosdep ros-jazzy-teleop-twist-keyboard
```

## 빠른 시작

### 1. 의존성 설치

```bash
cd ~/unitree_go2_sidewalk_navigation/ros2_ws
source /opt/ros/jazzy/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 빌드

처음 빌드할 때는 workspace 전체를 빌드합니다.

```bash
cd ~/unitree_go2_sidewalk_navigation/ros2_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

코드를 수정한 뒤 관련 패키지만 다시 빌드할 때는 다음 명령을 사용할 수
있습니다.

```bash
colcon build --symlink-install \
  --packages-select champ_base unitree_go2_description unitree_go2_sim go2_simulation
source install/setup.bash
```

새 터미널을 열 때마다 ROS 2와 workspace를 다시 source 해야 합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_go2_sidewalk_navigation/ros2_ws/install/setup.bash
```

### 3. Small City 실행

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py
```

Gazebo GUI만 보고 싶고 RViz를 끄려면 다음과 같이 실행합니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py rviz:=false
```

실행 후 Gazebo GUI에서 다음을 직접 확인합니다.

- 도로, 건물, 인도가 포함된 `small_city` 월드가 보인다.
- Go2가 인도 위에 spawn 된다.
- controller 활성화 후 로봇이 넘어지거나 지면 아래로 빠지지 않는다.
- `/cmd_vel` 명령을 주면 다리만 움직이는 것이 아니라 몸체 위치가 바뀐다.

빌드 성공이나 에러 없는 로그만으로 주행 성공을 판단하지 않습니다. 최종
판정은 Gazebo GUI에서 로봇의 실제 이동을 눈으로 확인하는 것입니다.

## 로봇 조작

### 키보드 teleop

새 터미널에서 workspace를 source 한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_go2_sidewalk_navigation/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`teleop_twist_keyboard`는 `/cmd_vel`에 `geometry_msgs/msg/Twist`를 발행합니다.
처음에는 작은 속도로 직진, 후진, 좌우 이동을 확인하십시오.

### Twist 직접 발행

저속 전진:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

저속 좌측 이동:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.03, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

정지:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

연속 발행을 끝낼 때는 `Ctrl+C`를 누르고 정지 명령을 한 번 더 발행합니다.

## 런타임 검증

시뮬레이션을 실행한 상태에서 새 터미널을 열고 다음을 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_go2_sidewalk_navigation/ros2_ws/install/setup.bash

ros2 control list_controllers
ros2 node list | grep -E "quadruped_controller_node|state_estimation_node"
timeout 5 ros2 topic hz /joint_group_controller/commands
timeout 5 ros2 topic hz /odometry/local
timeout 5 ros2 topic hz /unitree_lidar/points
```

controller 목록에는 다음 두 항목이 `active`로 표시되어야 합니다.

```text
joint_state_broadcaster
joint_group_controller
```

관절 순서를 추가로 검사하려면 다음 명령을 실행합니다.

```bash
ros2 run go2_simulation verify_joint_order.py
```

## 실행 구조

기본 실행선은 CHAMP + position control입니다.

```text
/cmd_vel
  -> champ_base/quadruped_controller_node
  -> /joint_group_controller/commands  (std_msgs/msg/Float64MultiArray)
  -> position_controllers/JointGroupPositionController
  -> gz_ros2_control
  -> Gazebo Sim
```

`unitree_go2_launch_small_city.py`는 Gazebo, robot state publisher, Gazebo bridge,
CHAMP controller, state estimator, EKF, TF publisher, ROS 2 controller spawner,
RViz를 함께 실행합니다.

## 월드 선택

| 환경 | 실행 명령 | 용도 |
| --- | --- | --- |
| Default | `ros2 launch go2_simulation unitree_go2_launch.py` | 기본 평지 검증 |
| Small City | `ros2 launch go2_simulation unitree_go2_launch_small_city.py` | 인도 및 도심 주행 검증 |
| Campus | `ros2 launch go2_simulation unitree_go2_launch_campus.py` | 야외 캠퍼스 검증 |
| TI Building | `ros2 launch go2_simulation unitree_go2_launch_TI.py` | 건물 환경 검증 |

사용 가능한 launch argument는 다음 명령으로 확인합니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py --show-args
```

자주 쓰는 argument:

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `rviz` | `true` | RViz 실행 여부 |
| `world` | `worlds/small_city.sdf` | Gazebo world 경로 |
| `robot_name` | `go2` | Gazebo entity 이름 |
| `world_init_x` | `15.0` | Small City spawn X |
| `world_init_y` | `5.2` | Small City spawn Y |
| `world_init_z` | launch 파일 기준 | Small City spawn 높이 |
| `world_init_heading` | `0.0` | 초기 heading |

`world_init_z`는 인도 collision과 직접 관련된 보정값입니다. collision 검증 없이
임의로 낮추거나 조정하지 마십시오. 현재 checkout의 실제 기본값은 반드시
`--show-args`로 확인하십시오.

## RViz와 LiDAR

프로젝트 전용 RViz 설정은 `rviz/rviz.rviz`입니다. 현재 기본 point cloud
topic은 다음과 같습니다.

```text
/unitree_lidar/points
```

Velodyne LiDAR를 비활성화한 상태에서도 RViz가 point cloud를 표시할 수 있도록
기본 topic을 `/velodyne_points/points`에서 `/unitree_lidar/points`로
변경했습니다.

point cloud가 보이지 않으면 다음을 확인합니다.

```bash
ros2 topic list | grep unitree_lidar
timeout 5 ros2 topic hz /unitree_lidar/points
```

## GitHub main 대비 변경점

원격 `origin/main`에는 `go2_simulation` 패키지가 없습니다. 이 작업 브랜치는
기존 `unitree_go2_ros2` 시뮬레이션 위에 프로젝트 전용 패키지를 추가하고,
인도 주행 검증을 위해 다음 내용을 확장했습니다.

| 영역 | 변경 내용 |
| --- | --- |
| Custom package | `go2_simulation` 패키지, 월드, 모델, RViz 설정 추가 |
| Small City | `small_city.sdf`와 도심 모델 추가, 인도 collision 조정 |
| Control mode | effort trajectory 경로를 position command 경로로 변경 |
| ROS 2 controller | `joint_group_controller`를 `JointGroupPositionController`로 구성 |
| Joint command | `/joint_group_controller/commands`에 `Float64MultiArray` 발행 |
| URDF | 관절 command interface를 `effort`에서 `position`으로 변경 |
| Contact | 발 마찰 계수와 contact damping 조정 |
| Gait | swing height, stance duration, controller update rate 조정 |
| RViz | 기본 point cloud topic을 `/unitree_lidar/points`로 변경 |
| Diagnostics | joint order 및 lateral gait 검증 스크립트 추가 |

position controller 이식에는 `go2_simulation` 외에도 다음 파일이 관여합니다.

```text
ros2_ws/src/unitree_go2_ros2/champ_base/
ros2_ws/src/unitree_go2_ros2/unitree_go2_description/urdf/
ros2_ws/src/unitree_go2_ros2/unitree_go2_sim/config/
ros2_ws/src/unitree_go2_ros2/unitree_go2_sim/launch/
```

## 진단 및 실험 도구

다음 스크립트는 `colcon build` 후 `ros2 run go2_simulation <script>` 형식으로
실행할 수 있습니다.

| 스크립트 | 목적 | 기본 실행선 포함 여부 |
| --- | --- | --- |
| `verify_joint_order.py` | `/joint_states`의 관절 구성 확인 | 진단용 |
| `record_lateral_gait_metrics.py` | lateral gait odom 변화량 기록 | 진단용 |
| `heading_correction_node.py` | `/cmd_vel_raw`를 받아 heading을 보정한 `/cmd_vel` 발행 | 선택 실험 |
| `capstone_gait_controller.py` | Capstone식 position gait 직접 발행 실험 | 선택 실험 |

기본 launch는 CHAMP controller를 사용합니다. `capstone_gait_controller.py`는
같은 `/joint_group_controller/commands` topic에 직접 명령을 발행하므로 CHAMP와
동시에 사용하지 마십시오.

lateral odom 지표 기록 예시:

```bash
ros2 run go2_simulation record_lateral_gait_metrics.py --ros-args \
  -p odom_topic:=/odometry/local \
  -p duration_sec:=20.0 \
  -p expected_direction:=left
```

## 패키지 구조

```text
go2_simulation/
├── config/       # 실험용 gait controller 설정
├── launch/       # 월드별 ROS 2 launch 파일
├── models/       # Small City에서 사용하는 Gazebo 모델
├── rviz/         # 프로젝트 전용 RViz 설정
├── scripts/      # 진단 및 선택 실험 도구
├── urdf/         # 프로젝트 전용 robot xacro
└── worlds/       # Gazebo world 파일
```

## 문제 해결

### Gazebo는 보이지만 로봇이 움직이지 않음

```bash
ros2 control list_controllers
timeout 5 ros2 topic hz /cmd_vel
timeout 5 ros2 topic hz /joint_group_controller/commands
```

`/cmd_vel`은 발행되는데 `/joint_group_controller/commands`가 갱신되지 않으면
CHAMP controller 실행 상태를 확인합니다. 두 topic이 모두 갱신되는데 몸체가
움직이지 않으면 Gazebo GUI에서 발 접촉, 자세, 인도 collision을 확인합니다.

### RViz 없이 Gazebo만 실행하고 싶음

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py rviz:=false
```

### Small City spawn 자세가 비정상임

`world_init_z`를 즉시 낮춰서 우회하지 마십시오. 인도 collision과 spawn 높이를
함께 확인해야 합니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py --show-args
```

## 참고 자료

- [ROS 2: Using colcon to build packages](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [ROS 2 Jazzy: Creating a launch file](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 Jazzy: Understanding topics](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [CHAMP quadruped controller framework](https://github.com/chvmp/champ)
