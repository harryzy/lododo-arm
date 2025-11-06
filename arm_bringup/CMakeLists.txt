cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files, config, and docs
install(DIRECTORY
  launch
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts (only existing scripts)
install(PROGRAMS
  scripts/quick_launch.sh
  scripts/health_check.sh
  scripts/start_yolo_node.sh
  scripts/start_yolo_dual_view.sh
  scripts/start_yolo_cube_detect.sh
  scripts/check_sim_status.sh
  scripts/stop_yolo_node.sh
  scripts/verify_yolo_install.sh
  scripts/simple_joint_reader.py
  scripts/launch_real_bringup_with_log.sh
  scripts/optimize_camera_top_view.sh
  scripts/calibrate_servo_standalone.py
  scripts/setup_robot_side.sh
  scripts/setup_pc_side.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
