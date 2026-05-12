# 마이그레이션 진행 기록 (Migration Log)

기존 `unitree_go2_ros2_jazzy` 패키지에서 독립된 `go2_simulation` 패키지로 시뮬레이션 환경을 분리하는 작업을 수행한 내역입니다.

## 1. 패키지 초기화
- `go2_simulation` 디렉토리를 생성하고 ROS2 호환을 위한 `CMakeLists.txt` 및 `package.xml`을 작성했습니다.
- 빌드 의존성으로 `launch_ros`, `ros_gz_sim`, `unitree_go2_description`, `unitree_go2_sim`을 추가했습니다.

## 2. 파일 및 폴더 이동
- **Models**: `jazzy` 최상단에 있던 `models` 폴더와 `small_city_world_package/models`의 내용물을 모두 `go2_simulation/models`로 통합 복사했습니다.
- **Worlds**: `small_city_world_package/worlds` 내부 파일과 `unitree_go2_description/worlds` 내의 `TIbuilding.sdf`, `campus.sdf` 등을 `go2_simulation/worlds`로 복사했습니다.
- **Launch Files**: 커스텀 환경 실행 파일인 `unitree_go2_launch_TI.py`, `unitree_go2_launch_campus.py`, `unitree_go2_launch_small_city.py` 등을 `go2_simulation/launch`로 복사했습니다.
- **URDF**: 로봇의 센서 위치나 추가 설정이 변경된 `unitree_go2_gazebo.xacro`, `unitree_go2_robot.xacro`를 `go2_simulation/urdf`로 복사했습니다.

## 3. 코드 내 경로 참조 일괄 패치
- 런치 파일(Launch files) 내에서 기존에 `unitree_go2_description`이나 `jazzy`를 바라보던 `urdf`, `worlds`, `models` 경로를 새로 생성한 `go2_simulation` 패키지를 참조하도록 일괄 변경(Replace) 하였습니다.
- Xacro 파일 내에서도 공통 요소(leg, materials 등)는 원본을 참조하되, 커스텀되는 모델은 `go2_simulation`을 보도록 수정했습니다.

## 4. 기존 패키지 비활성화
- 기존 `unitree_go2_ros2_jazzy` 패키지 폴더 최상단에 `COLCON_IGNORE` 파일을 생성하여 향후 빌드 시 `unitree_go2_ros2`와의 이름 충돌이 발생하지 않도록 막았습니다.

## 5. 빌드 및 검증 (Verification)
- Conda 환경 변수 충돌 문제를 우회하여 시스템 파이썬(`/usr/bin/python3`)으로 전체 워크스페이스를 클린 빌드했습니다. (`colcon build`)
- `ros2 launch go2_simulation unitree_go2_launch_small_city.py`를 실행하여 Gazebo Harmonic 환경과 로봇이 정상 로드됨을 확인했습니다.

## 6. 트러블슈팅 및 설정 롤백 (Troubleshooting & Rollbacks)
마이그레이션 직후 발생했던 두 가지 주요 문제를 다음과 같이 해결했습니다.
1. **Gazebo 시뮬레이션 속도 저하 문제 복구**: 
   - 증상: 시뮬레이션이 극도로 느리게 동작하는 현상.
   - 원인: 초기 최적화 시도 중 SDF 파일의 `real_time_update_rate`를 100으로 낮추어 Gazebo가 의도적으로 0.1배속으로 동작하게 만들었던 설정 오류였습니다.
   - 조치: `real_time_update_rate`를 1000으로 되돌려 원래의 1.0배속(정상 속도)으로 복구하고, 임의로 껐던 카메라 센서(30fps, visualize=1)도 기존 상태로 롤백했습니다.
2. **RViz2 상의 불필요한 D455/Depth 카메라 이슈 해결**:
   - 증상: 기존에 사용하지 않던 D455 센서 구독 오류가 RViz에 나타남.
   - 원인: `unitree_go2_ros2_jazzy` 패키지가 비활성화(`COLCON_IGNORE`)되면서, 런치 파일이 베이스 레포지토리(`unitree_go2_ros2`)의 RViz 설정을 로드하게 되었습니다. 베이스 레포지토리의 설정 파일에 D455가 켜져 있어 발생한 문제였습니다.
   - 조치: 기존에 정상적으로 사용하던 `rviz.rviz` 파일을 `go2_simulation/rviz/` 폴더로 직접 복사하고, `CMakeLists.txt` 및 런치 파일들이 이 새 경로를 참조하도록 수정하여 완벽히 기존 환경과 동일하게 맞추었습니다.
