cmake_minimum_required(VERSION 3.8)
project(lane_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lane_pipeline_node
  src/main.cpp
  src/pipeline/lane_pipeline_node.cpp
)

target_include_directories(lane_pipeline_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline
)

ament_target_dependencies(lane_pipeline_node
  rclcpp
  sensor_msgs
  std_msgs
  rcl_interfaces
  cv_bridge
)

target_link_libraries(lane_pipeline_node ${OpenCV_LIBS})

install(TARGETS lane_pipeline_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
