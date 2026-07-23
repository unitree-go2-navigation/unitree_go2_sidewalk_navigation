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
            # Nav2가 횡제어 소유 → 소셜 레이어 OFF (두 횡제어기 충돌 방지;
            # 게이트는 안전 본연 역할 유지)
            'safety_stop_extra_params': os.path.join(
                pkg, 'config', 'safety_social_off_nav.yaml'),
        }.items())

    # map→odom 정적 TF는 base 런치에 이미 있음. 여기서는 odom→base_footprint
    # 를 /odom에서 중계 (Nav2의 TF 체인 요구 — 기존 파이프라인은 토픽 기반이라
    # 이 TF가 없었음).
    odom_tf = Node(
        package='go2_simulation', executable='odom_tf_broadcaster.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': os.path.join(
                         pkg, 'maps', 'small_city_strip.yaml')}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'node_names': ['map_server']}])

    # bringup의 navigation_launch는 collision_monitor·docking 등 이 스택과
    # 역할이 겹치거나 불필요한 노드까지 포함 (미구성 시 lifecycle 중단) —
    # 필요 노드만 명시 기동. 충돌 안전은 우리 안전게이트가 담당.
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    common = [{'use_sim_time': True}, params]
    nav_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=common,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=common),
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
        DeclareLaunchArgument('world', default_value=os.path.join(
            pkg, 'worlds', 'small_city.sdf')),
        DeclareLaunchArgument('oracle_csv', default_value='/tmp/oracle.csv'),
        DeclareLaunchArgument('world_init_heading', default_value='3.141593'),
        DeclareLaunchArgument('degrade_profile', default_value=''),
        base,
        odom_tf,
        map_server,
        map_lifecycle,
    ] + nav_nodes)
