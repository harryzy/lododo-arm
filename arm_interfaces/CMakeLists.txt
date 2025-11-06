cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MeasuredObject.msg"
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "srv/ScanAndMeasure.srv"
  "action/MeasureObject.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    action_msgs
)

ament_package()
