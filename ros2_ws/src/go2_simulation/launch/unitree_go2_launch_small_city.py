import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = "base_link"

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    go2_simulation = launch_ros.substitutions.FindPackageShare(
        package="go2_simulation").find("go2_simulation")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")

    joints_config = os.path.join(unitree_go2_sim, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(unitree_go2_sim, "config/gait/gait.yaml")
    links_config = os.path.join(unitree_go2_sim, "config/links/links.yaml")
    default_model_path = os.path.join(go2_simulation, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(go2_simulation, "worlds/small_city.sdf")
    small_city_models_path = os.path.join(go2_simulation, "models")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_safety_gate = DeclareLaunchArgument(
        "safety_gate", default_value="true",
        description="Enable safety_stop gate. false = pass-through (baseline/teleop test).",
    )
    declare_oracle_csv = DeclareLaunchArgument(
        "oracle_csv", default_value="",
        description="Per-run clearance CSV path for collision_oracle (scenario runner). Empty = off.",
    )
    declare_degrade_profile = DeclareLaunchArgument(
        "degrade_profile", default_value="",
        description="Sensor degradation profile YAML (sim2real robustness). "
                    "Empty = clean cloud. When set, lidar_obstacle_node consumes "
                    "the degraded topic.",
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="15.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="5.2")
    # Fixed for the calibrated small_city sidewalk spawn. Do not tune implicitly.
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="1.0")
    declare_world_init_roll = DeclareLaunchArgument("world_init_roll", default_value="0.0")
    declare_world_init_pitch = DeclareLaunchArgument("world_init_pitch", default_value="0.0")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    # Description nodes and parameters
    xacro_file = LaunchConfiguration("unitree_go2_description_path")
    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    # CHAMP quadruped controller (Position control과 호환)
    robot_description_for_controller = ParameterValue(
        Command(['xacro ', LaunchConfiguration('unitree_go2_description_path')]),
        value_type=str
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_controller/commands"},
            {"urdf": robot_description_for_controller},
            joints_config,
            links_config,
            gait_config,
            {"hardware_connected": False},
            {"close_loop_odom": True},
        ],
        # CHAMP reads filtered cmd_vel from safety_stop output
        remappings=[("/cmd_vel/smooth", "/cmd_vel_safe")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": True},
            {"urdf": robot_description_for_controller},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": base_frame},
            {"world_frame": base_frame},
            {"odom0": "odom/raw"},
            {"imu0": "imu/data"},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odom/filtered")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_footprint"},
            {"world_frame": "odom"},
            {"odom0": "odom/filtered"},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    # Go2 static frame connection (map -> odom)
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        name='map_to_odom_tf_node',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
    )

    # Go2 URDF connection (base_footprint -> base_link)
    base_footprint_to_base_link_tf_node = Node(
        package='tf2_ros',
        name='base_footprint_to_base_link_tf_node',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ],
    )

    # heading_correction_node는 go_sim controller에서 불필요 (자체 제어)
    # heading_correction_node = Node(
    #     package='go2_simulation',
    #     executable='heading_correction_node.py',
    #     name='heading_correction_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'kp': 2.0},
    #         {'ki': 0.1},
    #         {'kd': 0.5},
    #         {'max_correction': 0.3},
    #     ],
    # )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(go2_simulation, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        # parameters=[{"use_sim_time": use_sim_time}]
    )

    # Evaluation oracle: ground-truth clearance metric (not used by avoidance)
    collision_oracle_node = Node(
        package='perception_avoidance',
        executable='collision_oracle_node',
        name='collision_oracle_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'csv_path': ParameterValue(
                LaunchConfiguration('oracle_csv'), value_type=str),
        }],
    )

    # Phase 1: LiDAR perception + safety stop gate
    perception_avoidance = get_package_share_directory('perception_avoidance')
    self_filter_yaml = os.path.join(perception_avoidance, 'config/self_filter.yaml')
    safety_stop_yaml = os.path.join(perception_avoidance, 'config/safety_stop.yaml')

    # Sensor degradation harness (sim2real robustness). Off unless a profile
    # YAML is given; then the perception input is rerouted through it.
    degrade_profile = LaunchConfiguration('degrade_profile')
    degrade_enabled = PythonExpression(["'", degrade_profile, "' != ''"])
    degrade_pointcloud_node = Node(
        package='perception_avoidance',
        executable='degrade_pointcloud_node',
        name='degrade_pointcloud_node',
        output='screen',
        parameters=[degrade_profile, {'use_sim_time': use_sim_time}],
        condition=IfCondition(degrade_enabled),
    )
    lidar_points_topic = PythonExpression(
        ["'/unitree_lidar/points_degraded' if '", degrade_profile,
         "' != '' else '/unitree_lidar/points'"])

    lidar_obstacle_node = Node(
        package='perception_avoidance',
        executable='lidar_obstacle_node',
        name='lidar_obstacle_node',
        output='screen',
        parameters=[
            self_filter_yaml,
            {'use_sim_time': use_sim_time,
             'points_topic': ParameterValue(lidar_points_topic, value_type=str)},
        ],
    )

    # safety_stop gate: teleop /cmd_vel → filter → /cmd_vel_safe → CHAMP
    # 게이트 파라미터 override yaml (빈 문자열 = 없음). 기본 yaml 위에
    # 겹쳐 로드 — 데모/실험용 임시 튜닝을 본 설정 파일 수정 없이 주입.
    declare_safety_stop_extra = DeclareLaunchArgument(
        "safety_stop_extra_params", default_value="",
        description="Optional extra params YAML layered over safety_stop.yaml")
    safety_stop_extra = LaunchConfiguration('safety_stop_extra_params')
    safety_stop_params = [
        safety_stop_yaml,
        {'use_sim_time': use_sim_time,
         'enable_gate': ParameterValue(
             LaunchConfiguration('safety_gate'), value_type=bool)},
    ]
    safety_stop_node = Node(
        package='perception_avoidance',
        executable='safety_stop_node',
        name='safety_stop_node',
        output='screen',
        parameters=safety_stop_params + [
            PythonExpression(["'", safety_stop_extra, "' or '",
                              safety_stop_yaml, "'"])],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),       # subscribe to teleop output
            ('/cmd_vel_safety', '/cmd_vel_safe'),  # publish filtered to CHAMP input
        ],
    )

    # 보도 경계 polygon (Phase 5a) — 팀원 segmentation 계약의 시뮬 대역.
    # /odom이 월드 좌표로 초기화되는 스택이라 스폰 인자 전달 불필요.
    sidewalk_polygon_yaml = os.path.join(
        perception_avoidance, 'config/sidewalk_polygon.yaml')
    sidewalk_polygon_node = Node(
        package='perception_avoidance',
        executable='sidewalk_polygon_node',
        name='sidewalk_polygon_node',
        output='screen',
        parameters=[
            sidewalk_polygon_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gazebo_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=small_city_models_path,
    )
    small_city_sdf_path = AppendEnvironmentVariable(
        name="SDF_PATH",
        value=small_city_models_path,
    )
    # Allow Gazebo to find custom ActorPosePublisher / WaypointMover plugins.
    go2_simulation_plugin_path = os.path.normpath(
        os.path.join(go2_simulation, "..", "..", "lib"))
    gazebo_plugin_path = AppendEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=go2_simulation_plugin_path,
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # gui:=false → server-only(-s). GPU 센서는 DISPLAY(GLX)로 렌더.
            # (--headless-rendering(EGL)은 이 머신에서 서버 행 유발 — 사용 금지)
            'gz_args': [
                LaunchConfiguration('world'),
                PythonExpression(
                    ["' -r -s' if '",
                     LaunchConfiguration('gui'), "' == 'false' else ' -r'"]),
            ]
        }.items(),
    )

    # Spawn robot in Gazebo Sim
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-R', LaunchConfiguration('world_init_roll'),
            '-P', LaunchConfiguration('world_init_pitch'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )

    # Bridge ROS 2 topics to Gazebo Sim
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Gazebo to ROS
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',

            # RealSense D435i RGBD camera + IMU topics
            '/d435i/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # 분리 RGB 이미저 (69°/720p, 데모 POV용 — xacro d435i_color 참조)
            '/d435i/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435i/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/d435i/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/d435i/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/d435i/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # ROS to Gazebo
            # NOTE: /cmd_vel→gz.msgs.Twist 브리지는 제거됨 — position control에서
            # 구동은 /cmd_vel_safe→CHAMP→joint 명령 경로뿐이며, 이 브리지는
            # 안전 게이트를 우회하는 잠재 경로였음 (gz velocity plugin 재활성 시).
            '/joint_group_controller/commands@std_msgs/msg/Float64MultiArray]gz.msgs.Double_V',
        ],
    )

    # Pose bridge for actor tracking (YAML config - handles complex topic paths)
    pose_bridge_config = os.path.join(go2_simulation, "config/pose_bridge.yaml")
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': pose_bridge_config,
        }],
    )

    # Use spawner nodes directly to handle the configuration step. (load → configure → activate)
    # Note: controller_manager는 전역 namespace에 있음 (namespace 제거)
    # Spawner will wait internally (--controller-manager-timeout) until controller_manager is ready
    controller_spawner_js = TimerAction(
        period=3.0,  # Start early, spawner waits internally for controller_manager
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_state_broadcaster",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    controller_spawner_effort = TimerAction(
        period=3.0,  # Start early, spawner waits internally for controller_manager
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_group_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    # Shell script to manually check controller status
    controller_status_check = TimerAction(
        period=15.0,  # Check status after controllers should be loaded
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Checking controller status:' && ros2 control list_controllers"],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        [
            # Launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_safety_gate,
            declare_safety_stop_extra,
            declare_oracle_csv,
            declare_degrade_profile,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_roll,
            declare_world_init_pitch,
            declare_world_init_heading,
            declare_description_path,
            gazebo_resource_path,
            small_city_sdf_path,
            gazebo_plugin_path,

            # Gazebo and robot nodes first
            gz_sim,
            robot_state_publisher_node,
            gazebo_spawn_robot,
            gazebo_bridge,
            pose_bridge,

            # CHAMP controller nodes
            quadruped_controller_node,
            state_estimator_node,

            # EKF nodes for localization
            base_to_footprint_ekf,
            footprint_to_odom_ekf,

            # TF publishers for frame connections
            map_to_odom_tf_node,
            base_footprint_to_base_link_tf_node,

            # Controller spawners that handle the complete lifecycle
            controller_spawner_js,
            controller_spawner_effort,
            controller_status_check,

            # Visualization (only if rviz flag is set)
            rviz2,

            # Evaluation oracle
            collision_oracle_node,

            # Phase 1: LiDAR safety gate (+ optional sim2real degradation)
            degrade_pointcloud_node,
            lidar_obstacle_node,
            safety_stop_node,

            # Phase 5a: 보도 경계 polygon
            sidewalk_polygon_node,
        ]
    )
