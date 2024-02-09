cmake_minimum_required(VERSION 3.5)
project(isam2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(geometry_msgs REQUIRED)  
find_package(eufs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(isam_test isam2Node.cpp)

target_link_libraries(isam_test gtsam)
ament_target_dependencies(isam_test rclcpp message_filters eufs_msgs geometry_msgs sensor_msgs)
ament_export_include_directories(source)

install(TARGETS
  isam_test
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks()
ament_package()
