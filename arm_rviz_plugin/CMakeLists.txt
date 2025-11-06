cmake_minimum_required(VERSION 3.8)
project(arm_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt5 settings - MUST be set before add_library
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Headers that need MOC processing
set(${PROJECT_NAME}_HEADERS
  include/arm_rviz_plugin/arm_control_panel.hpp
)

# MOC the headers
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HEADERS})

# Create the plugin library
add_library(${PROJECT_NAME} SHARED
  src/arm_control_panel.cpp
  ${${PROJECT_NAME}_MOCS}
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  std_msgs
  pluginlib
)

# Link Qt libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (RViz configurations)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.rviz"
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_default_plugins
  std_msgs
  pluginlib
)

ament_package()
