include(CMakePrintHelpers)

cmake_minimum_required(VERSION 3.5)
project(test_ws)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "${PROJECT_BINARY_DIR}/../isam2/")
set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_compile_options(-Wall -Wextra -Wpedantic)
# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(TBB REQUIRED)
find_package(isam2 REQUIRED)

add_library(imported_isam2_lib SHARED IMPORTED)
set_property(TARGET imported_isam2_lib PROPERTY IMPORTED_LOCATION "${PROJECT_BINARY_DIR}/../isam2/libisam2_lib.so")

file(GLOB SOURCES src/*.cpp)
add_executable(test_run ${SOURCES})
# target_link_libraries(test_run PUBLIC gtsam "${PROJECT_BINARY_DIR}/../isam2/libisam2_lib.a")
target_link_libraries(test_run PUBLIC imported_isam2_lib Eigen3::Eigen libtbb.so libgtsamDebug.so)
ament_target_dependencies(test_run PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)

# specify where the include directories are
target_include_directories(test_run PUBLIC include "${GTSAM_INCLUDE_DIR}" "${PROJECT_SOURCE_DIR}/../isam2/include" "${PROJECT_SOURCE_DIR}/../isam2/src")

install(TARGETS
test_run
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
