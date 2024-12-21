include(CMakePrintHelpers)

cmake_minimum_required(VERSION 3.5)
project(test_ws)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "${PROJECT_BINARY_DIR}/../isam2/")
add_compile_options(-Wall -Wextra -Wpedantic)
# find dependencies
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(GTSAM REQUIRED)
# find_library(III isam2_lib REQUIRED)
# if (NOT III) 
#     message(FATAL_ERROR "bruh wtf")
# endif()
# if (III)
#     message("The path found is ${III}")
# endif()
message(PROJECT_BINARY_DIR="${PROJECT_BINARY_DIR}")
add_library(imported_isam2_lib STATIC IMPORTED )
set_property(TARGET imported_isam2_lib PROPERTY IMPORTED_LOCATION "${PROJECT_BINARY_DIR}/../isam2/libisam2_lib.a")
# find_file(ISAM2_LIB_FILE isam2_lib_file "/home/danielnguyen/cmr/iSAM2_SLAM/workspace/build/isam2/libisam2_lib.a")
# if (NOT ISAM2_LIB_FILE) 
#     message(FATAL_ERROR "linker can't find the fucking fuckity fucking file")
# endif()
# find_library(ISAM2_LIB isam2_lib REQUIRED )
# if (NOT ISAM2_LIB) 
#     message(FATAL_ERROR "linker can't find the fucking library")
# endif()


# use the code from the subdirectory; this is for adding library to the project 
# add_subdirectory(${PROJECT_SOURCE_DIR}/../isam2/include, ${PROJECT_SOURCE_DIR}/include)
# add_subdirectory(${PROJECT_SOURCE_DIR}/../isam2/include ${PROJECT_SOURCE_DIR}/include)
file(GLOB SOURCES src/*.cpp)
add_executable(test_run ${SOURCES}) 
# target_link_libraries(test_run PUBLIC gtsam "${PROJECT_BINARY_DIR}/../isam2/libisam2_lib.a")
target_link_libraries(test_run PUBLIC gtsam imported_isam2_lib)
ament_target_dependencies(test_run PUBLIC rclcpp message_filters interfaces geometry_msgs sensor_msgs)

# specify where the include directories are
target_include_directories(test_run PUBLIC include "${GTSAM_INCLUDE_DIR}" "${PROJECT_SOURCE_DIR}/../isam2/include" "${PROJECT_SOURCE_DIR}/../isam2/src") 

install(TARGETS
test_run
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
