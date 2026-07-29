"""Nav2 통합 런치 (Track N, 시뮬 한정).

기존 풀스택(unitree_go2_launch_small_city.py: Gazebo+CHAMP+인지+안전게이트
+polygon) 위에 Nav2를 얹는다:
  Nav2(MPPI) → /cmd_vel → 안전게이트 → /cmd_vel_safe → CHAMP
게이트 최상위 유지 — Nav2는 조종자(teleop)를 대체할 뿐 안전 의미론 불변.
로컬라이제이션: map→odom identity 정적 TF (/odom이 월드 좌표 초기화임을
계측으로 확정 — 시뮬 한정 정당). 맵: verification/gen_nav_map.py 산출물.
twist_mux는 미설치라 보류 (설치 후 teleop 우선순위 먹싱 추가 예정).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'unitree_go2_launch_small_city.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'world': LaunchConfiguration('world'),
            'oracle_csv': LaunchConfiguration('oracle_csv'),
            'world_init_heading': LaunchConfiguration('world_init_heading'),
            'degrade_profile': LaunchConfiguration('degrade_profile'),
            'cameras_enabled': LaunchConfiguration('cameras_enabled'),
            'band_source': LaunchConfiguration('band_source'),
            # Nav2가 횡제어 소유 → 소셜 레이어 OFF (두 횡제어기 충돌 방지;
            # 게이트는 안전 본연 역할 유지)
            'safety_stop_extra_params': os.path.join(
                pkg, 'config', 'safety_social_off_nav.yaml'),
        }.items())

    # gui:=true면 Nav2 디스플레이(costmap·경로·footprint·보도 밴드) 포함
    # 전용 RViz도 함께 (base 런치의 rviz 인자와 별개 — 이중 실행 방지 위해
    # base에는 rviz false 유지)
    nav_rviz = Node(
        package='rviz2', executable='rviz2', name='nav2_rviz',
        condition=IfCondition(LaunchConfiguration('gui')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # map→odom 정적 TF는 base 런치에 이미 있음. 여기서는 odom→base_footprint
    # 를 /odom에서 중계 (Nav2의 TF 체인 요구 — 기존 파이프라인은 토픽 기반이라
    # 이 TF가 없었음).
    odom_tf = Node(
        package='go2_simulation', executable='odom_tf_broadcaster.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    # 맵리스(mapless:=true)면 map_server 자체를 띄우지 않음 — 전역 코스트맵이
    # rolling으로 전환되어 정적 지도 불필요 (nav2_mapless.yaml 오버레이)
    mapless = LaunchConfiguration('mapless')
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        condition=UnlessCondition(mapless),
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': os.path.join(
                         pkg, 'maps', 'small_city_strip.yaml')}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        condition=UnlessCondition(mapless),
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server']}])

    # bringup의 navigation_launch는 collision_monitor·docking 등 이 스택과
    # 역할이 겹치거나 불필요한 노드까지 포함 (미구성 시 lifecycle 중단) —
    # 필요 노드만 명시 기동. 충돌 안전은 우리 안전게이트가 담당.
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    mapless_params = os.path.join(pkg, 'config', 'nav2_mapless.yaml')
    common = [{'use_sim_time': True}, params]
    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        # planner_server(전역 코스트맵 소유)만 프로파일 분기 — 나머지는 공통
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=common,
             condition=UnlessCondition(mapless)),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=common + [mapless_params],
             condition=IfCondition(mapless)),
        Node(package='nav2_behaviors', executable='behavior_server',
             output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen', parameters=common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['controller_server',
                                         'planner_server',
                                         'behavior_server',
                                         'bt_navigator',
                                         'velocity_smoother']}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # 기본 = 축소판(sidewalk_16 인접만) — 가벼운 반복 실험용.
        # small_city(원본)는 최종 회피 알고리즘 완성 시 실기체 전이 전
        # 테스트에 사용 (world:=.../small_city.sdf 로 전환; 앞당김 가능).
        DeclareLaunchArgument('world', default_value=os.path.join(
            pkg, 'worlds', 'small_city_test.sdf')),
        DeclareLaunchArgument('oracle_csv', default_value='/tmp/oracle.csv'),
        DeclareLaunchArgument('world_init_heading', default_value='3.141593'),
        DeclareLaunchArgument('degrade_profile', default_value=''),
        # nav 기본 OFF: 카메라 렌더가 L1을 기아 상태로 만듦 (RTF 1.0에서
        # 10Hz→2.7Hz 실측). 데모 POV 필요 시 cameras_enabled:=true
        DeclareLaunchArgument('cameras_enabled', default_value='false'),
        # true = 사전 지도 없이 rolling 전역 코스트맵 (야외 전이 형태).
        # 기본 false = 결정적 맵 (회귀 평가 재현성). goal은 30m 창 안에서.
        DeclareLaunchArgument('mapless', default_value='false'),
        # 보도 밴드 출처 — config(사전 정의) | lidar(실시간 연석·벽 추정).
        # 사전 주입 최소화 방침(2026-07-29)에 따라 lidar 권장; config는
        # 러너 회귀 재현성용 유지.
        DeclareLaunchArgument('band_source', default_value='config'),
        base,
        nav_rviz,
        odom_tf,
        map_server,
        map_lifecycle,
    ] + nav_nodes)
