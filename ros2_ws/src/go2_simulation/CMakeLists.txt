cmake_minimum_required(VERSION 3.8)
project(go2_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

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
