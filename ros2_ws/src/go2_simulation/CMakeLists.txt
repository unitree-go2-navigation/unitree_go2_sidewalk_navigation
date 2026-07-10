cmake_minimum_required(VERSION 3.8)
project(go2_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Gazebo Harmonic plugin: actor_pose_publisher
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(ActorPosePublisher SHARED
  plugins/actor_pose_publisher/src/ActorPosePublisher.cc
)
target_link_libraries(ActorPosePublisher
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
)
install(TARGETS ActorPosePublisher
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Gazebo Harmonic plugin: waypoint_mover (이동 보행자 — actor 대체, 회귀 러너용)
add_library(WaypointMover SHARED
  plugins/waypoint_mover/src/WaypointMover.cc
)
target_link_libraries(WaypointMover
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
)
install(TARGETS WaypointMover
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(DIRECTORY
  launch
  models
  worlds
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/heading_correction_node.py
  scripts/capstone_gait_controller.py
  scripts/verify_joint_order.py
  scripts/record_lateral_gait_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
