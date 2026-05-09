# Small City World Package for Gazebo Harmonic

이 패키지는 Gazebo Harmonic에서 Unitree Go2 로봇 시뮬레이션을 위해 마이그레이션된 `small_city` world와 필요한 모든 모델을 포함합니다.

---

## 패키지 구조

```
small_city_world_package/
├── README.md           # 이 문서
├── worlds/
│   └── small_city.sdf  # Gazebo Harmonic world 파일
└── models/             # World에 필요한 모든 3D 모델 (34개)
    ├── ambulance/
    ├── apartment/
    ├── cardboard_box/
    ├── city_terrain/
    ├── dumpster/
    ├── fast_food/
    ├── fire_hydrant/
    ├── fountain/
    ├── gas_station/
    ├── gazebo/
    ├── hatchback/
    ├── hatchback_blue/
    ├── hatchback_red/
    ├── house_1/
    ├── house_2/
    ├── house_3/
    ├── lamp_post/
    ├── law_office/
    ├── oak_tree/
    ├── ocean/
    ├── osrf_first_office/
    ├── pickup/
    ├── pier/
    ├── pine_tree/
    ├── postbox/
    ├── post_office/
    ├── radio_tower/
    ├── salon/
    ├── stop_light_post/
    ├── stop_sign/
    ├── suv/
    ├── telephone_pole/
    ├── thrift_shop/
    └── truss_bridge/
```

---

## 시스템 요구사항

- **OS**: Ubuntu 24.04 (권장)
- **ROS2**: Jazzy
- **Gazebo**: Harmonic (gz-harmonic)
- **Python**: 3.10+

---

## 설치 방법

### 방법 1: 심볼릭 링크 (권장)

심볼릭 링크를 사용하면 여러 워크스페이스에서 동일한 모델을 공유할 수 있습니다.

```bash
# 1. 패키지를 원하는 위치에 복사 (예: /opt/gazebo_worlds/)
sudo mkdir -p /opt/gazebo_worlds
sudo cp -r small_city_world_package /opt/gazebo_worlds/

# 2. Gazebo 리소스 경로 환경변수 설정 (~/.bashrc에 추가)
echo 'export GZ_SIM_RESOURCE_PATH=/opt/gazebo_worlds/small_city_world_package/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
echo 'export SDF_PATH=/opt/gazebo_worlds/small_city_world_package/models:$SDF_PATH' >> ~/.bashrc
source ~/.bashrc

# 3. 검증
gz sdf -k /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf
```

**장점**:
- 디스크 공간 절약
- 모델 업데이트가 모든 워크스페이스에 즉시 반영
- 관리가 간편

---

### 방법 2: 직접 복사

특정 워크스페이스에서만 사용하려면 직접 복사합니다.

```bash
# 1. 워크스페이스 디렉토리로 이동
cd ~/your_ros2_workspace/src/

# 2. 패키지 복사
cp -r /path/to/small_city_world_package ./

# 3. 환경변수 설정 (해당 워크스페이스에서만)
cd ~/your_ros2_workspace
echo 'export GZ_SIM_RESOURCE_PATH=$PWD/src/small_city_world_package/models:$GZ_SIM_RESOURCE_PATH' >> install/setup.bash
echo 'export SDF_PATH=$PWD/src/small_city_world_package/models:$SDF_PATH' >> install/setup.bash

# 4. 환경변수 로드
source install/setup.bash
```

**장점**:
- 워크스페이스 독립성
- 다른 워크스페이스에 영향 없음

---

### 방법 3: ROS2 패키지로 통합 (고급)

ROS2 launch 파일과 함께 사용하려면 패키지로 만듭니다.

```bash
# 1. ROS2 워크스페이스로 이동
cd ~/your_ros2_workspace/src/

# 2. 패키지 복사
cp -r /path/to/small_city_world_package ./

# 3. package.xml 생성
cd small_city_world_package
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>small_city_world</name>
  <version>1.0.0</version>
  <description>Small City world for Gazebo Harmonic</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# 4. CMakeLists.txt 생성
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(small_city_world)

find_package(ament_cmake REQUIRED)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
EOF

# 5. 빌드
cd ~/your_ros2_workspace
colcon build --packages-select small_city_world

# 6. 환경변수 자동 설정
source install/setup.bash
```

**장점**:
- ROS2 생태계와 완전 통합
- Launch 파일에서 쉽게 참조 가능
- `colcon build`로 관리

---

## 사용 방법

### 1. Gazebo GUI에서 실행

```bash
# 환경변수 확인
echo $GZ_SIM_RESOURCE_PATH
echo $SDF_PATH

# 실행 (방법 1 기준)
gz sim /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf

# 또는 상대 경로 (현재 디렉토리가 패키지 루트일 때)
gz sim worlds/small_city.sdf
```

### 2. ROS2 Launch 파일에서 사용

**방법 A: Python Launch 파일**

```python
# your_package/launch/small_city_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 환경변수 설정
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/opt/gazebo_worlds/small_city_world_package/models:' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    sdf_path = SetEnvironmentVariable(
        name='SDF_PATH',
        value='/opt/gazebo_worlds/small_city_world_package/models:' + 
              os.environ.get('SDF_PATH', '')
    )
    
    # Gazebo 실행
    gazebo = Node(
        package='ros_gz_sim',
        executable='gz_server',
        arguments=['/opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf'],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        sdf_path,
        gazebo,
    ])
```

**방법 B: XML Launch 파일**

```xml
<!-- your_package/launch/small_city_sim.launch.xml -->
<launch>
  <!-- 환경변수 설정 -->
  <env name="GZ_SIM_RESOURCE_PATH" value="/opt/gazebo_worlds/small_city_world_package/models:$(env GZ_SIM_RESOURCE_PATH)" />
  <env name="SDF_PATH" value="/opt/gazebo_worlds/small_city_world_package/models:$(env SDF_PATH)" />
  
  <!-- Gazebo 실행 -->
  <node pkg="ros_gz_sim" exec="gz_server" 
        args="/opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf"
        output="screen" />
</launch>
```

### 3. Go2 로봇과 함께 사용

```bash
# Terminal 1: Gazebo world 실행
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/opt/gazebo_worlds/small_city_world_package/models:$GZ_SIM_RESOURCE_PATH
export SDF_PATH=/opt/gazebo_worlds/small_city_world_package/models:$SDF_PATH
gz sim /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf

# Terminal 2: Go2 로봇 spawn
source /opt/ros/jazzy/setup.bash
source ~/your_go2_workspace/install/setup.bash
ros2 launch go2_description spawn_go2.launch.py \
    x:=0.0 y:=0.0 z:=0.5
```

---

## 검증 방법

### 1. SDF 문법 검증

```bash
gz sdf -k /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf
```

**예상 출력**: `Valid.`

### 2. 모델 경로 확인

```bash
# 환경변수 확인
echo $GZ_SIM_RESOURCE_PATH | grep -o "/opt/gazebo_worlds/small_city_world_package/models"

# 모델 파일 존재 확인
ls /opt/gazebo_worlds/small_city_world_package/models/city_terrain/model.sdf
ls /opt/gazebo_worlds/small_city_world_package/models/ocean/model.sdf
```

### 3. Gazebo 로드 테스트 (서버 모드)

```bash
timeout 10s gz sim -s /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf 2>&1 | grep -iE "error|unable"
```

**예상**: 에러 메시지 없음 (timeout 종료)

### 4. GUI 시각 확인

```bash
gz sim /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf
```

**확인 사항**:
- [ ] World가 정상적으로 로드됨
- [ ] 건물, 도로, 차량이 모두 보임
- [ ] 배경에 terrain과 산이 보임
- [ ] 콘솔에 에러 메시지 없음

---

## World 특징

### 물리 파라미터

- **Physics Engine**: ODE
- **Max Step Size**: 0.005s
- **Real Time Factor**: 1.0
- **Update Rate**: 200Hz

### 주요 구성 요소

1. **도로 시스템**:
   - 8개 road 모델 (road_y_1~5, road_x_1~3)
   - 어두운 asphalt 색상
   - Box 기반 collision geometry
   - 높이: z=0.01~0.015

2. **Sidewalk**:
   - 38개 sidewalk 모델 (sidewalk_3~40)
   - Polyline collision geometry (Go2 로봇 충돌 감지용)
   - 높이: 0.15m

3. **Terrain**:
   - city_terrain: Heightmap 기반 지형
   - ocean: 수로/바다 영역
   - city_base_plane: 도시 바닥 보강 (ocean 노출 방지)

4. **오브젝트**:
   - 건물: 7개 (apartment, houses, offices, shops)
   - 차량: 9개 (hatchback, pickup, suv, ambulance)
   - 구조물: 18개 (bridge, tower, trees, lamp posts, etc.)

### 카메라 초기 위치

```xml
<pose>-40.637 0.86 2.365 0 0.1899 0.476</pose>
```

도시 블록 서쪽에서 동쪽을 바라보는 각도

---

## 문제 해결

### 문제 1: "Unable to find uri" 에러

**증상**:
```
Error: Unable to find uri[model://city_terrain]
```

**해결책**:
```bash
# 환경변수가 제대로 설정되었는지 확인
echo $GZ_SIM_RESOURCE_PATH
echo $SDF_PATH

# 모델 디렉토리가 존재하는지 확인
ls /opt/gazebo_worlds/small_city_world_package/models/city_terrain

# 환경변수 다시 설정
export GZ_SIM_RESOURCE_PATH=/opt/gazebo_worlds/small_city_world_package/models:$GZ_SIM_RESOURCE_PATH
export SDF_PATH=/opt/gazebo_worlds/small_city_world_package/models:$SDF_PATH
```

---

### 문제 2: "Permission denied" 에러

**증상**:
```
Error: Permission denied: /opt/gazebo_worlds/...
```

**해결책**:
```bash
# 소유권 확인
ls -la /opt/gazebo_worlds/small_city_world_package

# 필요시 권한 수정
sudo chmod -R 755 /opt/gazebo_worlds/small_city_world_package

# 또는 사용자 홈 디렉토리로 이동
cp -r /opt/gazebo_worlds/small_city_world_package ~/gazebo_worlds/
export GZ_SIM_RESOURCE_PATH=~/gazebo_worlds/small_city_world_package/models:$GZ_SIM_RESOURCE_PATH
```

---

### 문제 3: Terrain이 보이지 않음

**증상**: 도시 블록만 보이고 주변 terrain/산이 보이지 않음

**해결책**:
```bash
# city_terrain 모델이 올바른지 확인
ls /opt/gazebo_worlds/small_city_world_package/models/city_terrain/model.sdf
ls /opt/gazebo_worlds/small_city_world_package/models/city_terrain/materials/textures/

# heightmap 이미지 존재 확인
ls /opt/gazebo_worlds/small_city_world_package/models/city_terrain/materials/textures/city_terrain.jpg

# 텍스처 파일이 누락되었다면 원본에서 복사
cp -r original_source/models/city_terrain/materials /opt/gazebo_worlds/small_city_world_package/models/city_terrain/
```

---

### 문제 4: Go2 로봇이 sidewalk를 통과함

**증상**: 로봇이 sidewalk collision을 감지하지 못함

**해결책**:

이 패키지의 `small_city.sdf`에는 이미 37개 sidewalk에 collision geometry가 추가되어 있습니다. 만약 문제가 있다면:

```bash
# Collision 개수 확인 (49개여야 함)
grep -c '<collision name="collision">' worlds/small_city.sdf

# 특정 sidewalk collision 확인
grep -A5 "sidewalk_3" worlds/small_city.sdf | grep collision

# 로봇의 collision 필터 설정 확인 (Go2 모델 파일)
# - 로봇이 static 오브젝트와 충돌하도록 설정되어 있는지 확인
```

---

### 문제 5: 성능 저하 (FPS 낮음)

**증상**: Gazebo가 느리게 실행됨

**해결책**:

```bash
# 1. 그래픽 설정 낮추기
gz sim worlds/small_city.sdf --render-engine ogre

# 2. 센서 비활성화 (카메라, 라이다 등)
# Launch 파일에서 센서 플러그인 제거 또는 비활성화

# 3. Physics update rate 낮추기
# worlds/small_city.sdf에서 수정:
# <real_time_update_rate>200</real_time_update_rate>
# → <real_time_update_rate>100</real_time_update_rate>

# 4. 멀리 있는 오브젝트 제거
# 필요없는 건물/차량 모델을 world 파일에서 삭제
```

---

## 패키지 업데이트

이 패키지를 업데이트하려면:

```bash
# 1. 백업
cp -r /opt/gazebo_worlds/small_city_world_package /opt/gazebo_worlds/small_city_world_package.backup

# 2. 새 버전 복사
sudo rm -rf /opt/gazebo_worlds/small_city_world_package
sudo cp -r /path/to/new/small_city_world_package /opt/gazebo_worlds/

# 3. 검증
gz sdf -k /opt/gazebo_worlds/small_city_world_package/worlds/small_city.sdf
```

---

## 기여 및 문의

이 world는 Gazebo Classic `small_city.world`에서 Gazebo Harmonic으로 마이그레이션되었습니다.

**주요 변경사항**:
- Classic `<road>` primitive → box 기반 model
- Material script → PBR material
- Polyline collision 추가 (37개 sidewalk)
- Terrain heightmap z 파라미터 조정
- City base plane 추가 (ocean 노출 방지)

**원본 출처**:
- TU Delft / York University / ETH Zurich 등 다양한 오픈소스 프로젝트에서 수집된 모델들

**라이센스**: 각 모델의 원본 라이센스를 따릅니다 (대부분 Apache-2.0 또는 BSD)

---

## 체크리스트

설치 후 다음 항목을 확인하세요:

- [ ] 패키지 디렉토리 구조가 올바름 (`worlds/`, `models/` 존재)
- [ ] 환경변수 `GZ_SIM_RESOURCE_PATH`가 설정됨
- [ ] 환경변수 `SDF_PATH`가 설정됨
- [ ] `gz sdf -k` 검증 통과
- [ ] Gazebo GUI에서 world 로드 성공
- [ ] 건물/도로/terrain 모두 정상 렌더링
- [ ] 콘솔 에러 메시지 없음

---

## 버전 정보

- **패키지 버전**: 1.0.0
- **생성일**: 2026-05-06
- **Gazebo 버전**: Harmonic
- **ROS2 버전**: Jazzy (Ubuntu 24.04)
- **World 파일**: small_city.sdf (5922 lines, 170KB)
- **포함 모델**: 34개

---

## 추가 리소스

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [SDF Format Specification](http://sdformat.org/)
- [Gazebo Fuel](https://app.gazebosim.org/fuel/models) - 추가 모델 다운로드

---

**Happy Simulating! 🤖🏙️**
