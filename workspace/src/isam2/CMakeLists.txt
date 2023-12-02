cmake_minimum_required(VERSION 3.5)
project(planning)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(gtsam REQUIRED)

add_executable(isam src/isam2Node.cpp)
ament_target_dependencies(isam rclcpp message_filters eufs_msgs gtsam)
ament_export_include_directories(source)

install(TARGETS
  isam
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks()
ament_package()
