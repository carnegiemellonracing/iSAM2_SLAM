cmake_minimum_required(VERSION 3.5)
project(raceline)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Indicate CMAKE_BUILD_TYPE
if (CMAKE_BUILD_TYPE STREQUAL Debug)
  message("<---- Building in Debug mode ---->")
endif()
if (NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  message("<---- Building NOT Debug mode ---->")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # remember that you may need to do -O0 when using gdb to debug
endif()
message(PROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}")
message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(GSL REQUIRED)


# <----  executable ---->
file(GLOB SOURCES src/*.cpp)
add_executable(raceline_run ${SOURCES}) # not actually used

# linking
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(raceline_run PUBLIC Eigen3::Eigen )
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(raceline_run PUBLIC Eigen3::Eigen)
endif()
ament_target_dependencies(raceline_run PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs Eigen3 GSL)

# including dependencies
target_include_directories(raceline_run PUBLIC include)

install(TARGETS raceline_run
  DESTINATION lib/${PROJECT_NAME})


# <---- raceline_lib ---->
add_library(raceline_lib SHARED ${SOURCES})
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(raceline_lib PUBLIC  Eigen3::Eigen)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(raceline_lib PUBLIC  Eigen3::Eigen)
endif()
ament_target_dependencies(raceline_lib PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs GSL Eigen3)
target_include_directories(raceline_lib PUBLIC include)
ament_export_include_directories(include)
ament_package()

