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
find_package(raceline REQUIRED)
find_package(TBB REQUIRED)
# import the raceline library
add_library(imported_raceline_lib SHARED IMPORTED)
set_property(TARGET imported_raceline_lib PROPERTY IMPORTED_LOCATION "${PROJECT_BINARY_DIR}/../raceline/libraceline_lib.so")

message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
message(PROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}")
# <----  executable ---->
file(GLOB SOURCES src/*.cpp)
add_executable(isam2_run ${SOURCES})

# linking
if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_libraries(isam2_run PUBLIC imported_raceline_lib libtbb.so libgtsamDebug.so Eigen3::Eigen )
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_run PUBLIC imported_raceline_lib libtbb.so libgtsam.so Eigen3::Eigen)
endif()
ament_target_dependencies(isam2_run PUBLIC rclcpp message_filters interfaces raceline geometry_msgs sensor_msgs)

# including dependencies
if (NOT EXISTS "${PROJECT_SOURCE_DIR}/../raceline/include")
    message(FATAL_ERROR "couldn't find raceline include dir")
endif()
if (EXISTS "${PROJECT_SOURCE_DIR}/../raceline/include")
    message("FOUND RACELINE INCLUDE DIR")
endif()
target_include_directories(isam2_run PUBLIC include ${GTSAM_INCLUDE_DIR} "${PROJECT_SOURCE_DIR}/../raceline/include")

install(TARGETS isam2_run
  DESTINATION lib/${PROJECT_NAME})

# <---- isam2_lib ---->
add_library(isam2_lib SHARED ${SOURCES})
# target_link_libraries(isam2_lib PUBLIC gtsam)
if(CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_lib PUBLIC libtbb.so libgtsamDebug.so Eigen3::Eigen imported_raceline_lib)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
  target_link_libraries(isam2_lib PUBLIC libtbb.so libgtsam.so Eigen3::Eigen imported_raceline_lib)
endif()
ament_target_dependencies(isam2_lib PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs raceline)
target_include_directories(isam2_lib PUBLIC include ${GTSAM_INCLUDE_DIR} "${PROJECT_SOURCE_DIR}/../raceline/include")
set_target_properties(isam2_run PROPERTIES
    INSTALL_RPATH "/home/dale/driverless-packages/iSAM2_SLAM/workspace/build/raceline"
    BUILD_RPATH "/home/dale/driverless-packages/iSAM2_SLAM/workspace/build/raceline"
    BUILD_WITH_INSTALL_RPATH TRUE
)

ament_export_include_directories(include)
ament_package()

