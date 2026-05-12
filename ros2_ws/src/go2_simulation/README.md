# go2_simulation 패키지

이 패키지는 Unitree Go2 로봇을 위한 커스텀 Gazebo Harmonic 시뮬레이션 환경과 런치 파일들을 포함하고 있습니다.
기본 로봇 설정(Controller, base URDF 등)은 팀 레포지토리인 `unitree_go2_ros2`를 참조하며, 추가적인 월드(World), 모델(Model), 센서 설정만을 이 패키지에서 독립적으로 관리합니다.

## 패키지 구조

```
go2_simulation/
├── CMakeLists.txt        # 패키지 빌드 스크립트
├── package.xml           # 패키지 정보 및 의존성 정의
├── README.md             # 패키지 설명서 및 사용법 (한국어)
├── launch/               # Gazebo 및 RViz 실행을 위한 런치 파일
│   ├── unitree_go2_launch.py
│   ├── unitree_go2_launch_TI.py
│   ├── unitree_go2_launch_campus.py
│   └── unitree_go2_launch_small_city.py
├── models/               # Gazebo에서 사용할 각종 3D 모델들
│   ├── ambulance, apartment, ... 등 (small_city 모델 포함)
├── rviz/                 # RViz2 시각화 설정 파일
├── urdf/                 # 커스텀이 포함된 로봇 외형/센서 설정
│   ├── unitree_go2_gazebo.xacro
│   └── unitree_go2_robot.xacro
└── worlds/               # 시뮬레이션 맵 파일들
    ├── TIbuilding.sdf
    ├── campus.sdf
    └── small_city.sdf

```

## 시스템 요구사항

- **OS**: Ubuntu 24.04 (권장)
- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-harmonic)
- **Base Package**: `unitree_go2_ros2` 패키지가 워크스페이스 내에 존재해야 함

## 설치 및 빌드

이 패키지는 ROS2 ament_cmake 기반으로 만들어졌습니다.
워크스페이스 루트(예: `~/ros2_ws`)에서 빌드합니다.

```bash
cd ~/ros2_ws
colcon build --packages-up-to go2_simulation
source install/setup.bash
```

## 사용 방법

시뮬레이션을 시작하려면 원하는 환경에 맞는 런치 파일을 실행하세요.
각 런치 파일은 Gazebo, 로봇 모델 스폰(Spawn), CHAMP 컨트롤러, RViz2 등을 한 번에 실행합니다.

### 1. Small City 환경

도심의 도로와 인도가 구현되어 있는 환경입니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_small_city.py
```

### 2. Campus 환경

기본적인 캠퍼스 건물과 넓은 야외 지형이 포함된 환경입니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_campus.py
```

### 3. TI Building 환경

복합 건물 구조를 갖춘 환경입니다.

```bash
ros2 launch go2_simulation unitree_go2_launch_TI.py
```

## 4. 기존 환경과의 차이점 및 주의사항 (For Developers/Agents)

이 패키지는 기존에 사용하던 `unitree_go2_ros2_jazzy` 패키지를 완전히 대체하기 위해 만들어졌습니다.
다른 에이전트나 개발자가 유지보수할 때 혼동을 피하기 위해 다음 사항을 반드시 숙지해야 합니다.

1. **`unitree_go2_ros2_jazzy` 비활성화**:
   - `jazzy` 패키지 내부에는 `COLCON_IGNORE` 파일이 생성되어 빌드 대상에서 제외되었습니다. 이로 인해 ROS2의 패키지 경로 탐색 시, 중복되던 패키지들(`unitree_go2_description`, `unitree_go2_sim` 등)은 모두 **Base 레포지토리(`unitree_go2_ros2`)**로 연결됩니다.

2. **RViz 설정 분리 (`rviz/rviz.rviz`)**:
   - Base 레포지토리의 RViz 설정에는 D455 Depth 카메라 등 무거운 뷰어 설정이 포함되어 있습니다.
   - 불필요한 센서(D455 등) 토픽 참조 오류를 막고 기존과 동일한 쾌적한 화면을 띄우기 위해, 기존에 쓰던 `rviz.rviz` 파일을 이 패키지의 `rviz/` 디렉토리로 독립시켰습니다. 런치 파일 수정 시 **반드시 이 패키지(`go2_simulation`) 내부의 RViz 설정을 참조하도록 유지**해야 합니다.

3. **Gazebo 물리 엔진 설정 주의 (`small_city.sdf`)**:
   - `small_city.sdf` 내의 `real_time_update_rate`는 기본값인 **1000**을 유지해야 합니다. (이 값을 100으로 낮추면 시뮬레이션이 0.1배속의 슬로우 모션으로 강제 구동되는 심각한 성능 저하가 발생합니다.)
   - 카메라 센서 플러그인(`unitree_go2_gazebo.xacro`)은 정상 작동(30fps, visualize=1, always_on=1)하도록 설정되어 있습니다.

만약 월드 추가, 센서 추가, 물리 엔진 세팅 등의 추가적인 커스텀 설정이 필요하다면 앞으로는 **반드시 이 `go2_simulation` 패키지 내부**의 파일들을 수정하여 사용하시기 바랍니다.
