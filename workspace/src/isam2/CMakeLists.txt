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

# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

# <----  executable ---->
file(GLOB SOURCES src/*.cpp)
add_executable(isam2_run ${SOURCES})

# linking
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_run PUBLIC libtbb.so libgtsamDebug.so Eigen3::Eigen libfmt.so)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_run PUBLIC libtbb.so libgtsam.so Eigen3::Eigen libfmt.so)
endif()
ament_target_dependencies(isam2_run PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)

# including dependencies
target_include_directories(isam2_run PUBLIC include ${GTSAM_INCLUDE_DIR} )

install(TARGETS isam2_run
  DESTINATION lib/${PROJECT_NAME})


# <---- isam2_lib ---->
add_library(isam2_lib SHARED ${SOURCES})
# target_link_libraries(isam2_lib PUBLIC gtsam)
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_lib PUBLIC libtbb.so libgtsamDebug.so Eigen3::Eigen libfmt.so)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_lib PUBLIC libtbb.so libgtsam.so Eigen3::Eigen libfmt.so)
endif()
ament_target_dependencies(isam2_lib PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)
target_include_directories(isam2_lib PUBLIC include ${GTSAM_INCLUDE_DIR})
ament_export_include_directories(include)
ament_package()

