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

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(GTSAM REQUIRED)

# <---- isam_test executable ---->
file(GLOB SOURCES src/*.cpp)
add_executable(isam_test ${SOURCES})

# linking
target_link_libraries(isam_test PUBLIC gtsam)
ament_target_dependencies(isam_test PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)

# including dependencies
target_include_directories(isam_test PUBLIC include ${GTSAM_INCLUDE_DIR})

install(TARGETS isam_test
  DESTINATION lib/${PROJECT_NAME})


# <---- isam2_lib ---->
# add_library(isam2_lib STATIC "include/data_association.hpp"
#                               "include/ros_utils.hpp"
#                               "include/isam2_pkg.hpp"
#                               ${SOURCES})
add_library(isam2_lib STATIC ${SOURCES})
target_link_libraries(isam2_lib PUBLIC gtsam)
ament_target_dependencies(isam2_lib PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)
target_include_directories(isam2_lib PUBLIC include ${GTSAM_INCLUDE_DIR})

# Potentially need to link?
# target_link_libraries(isam2_lib PUBLIC gtsam)
# ament_target_dependencies(isam2_lib PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)
# target_include_directories(isam2_lib PUBLIC include ${GTSAM_INCLUDE_DIR})
# set_target_properties(isam2_lib PROPERTIES LINKER_LANGUAGE CXX)
# install(DIRECTORY include/
#     DESTINATION include)

ament_export_include_directories(include)
ament_package()

