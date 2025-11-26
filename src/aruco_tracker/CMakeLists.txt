cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# NOTE: we build opencv from source, hence the path /usr/local/lib
#find_package(OpenCV 4.10.0 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

find_package(OpenCV 4 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
link_directories(/usr/local/lib)

add_executable(aruco_tracker ArucoTracker.cpp)
add_executable(simple_precision_land src/simple_precision_land.cpp)
add_executable(drift_correction src/drift_correction.cpp)
add_executable(vision_position_bridge src/vision_position_bridge.cpp)

target_link_libraries(aruco_tracker ${OpenCV_LIBS})
target_link_libraries(simple_precision_land ${OpenCV_LIBS})
target_link_libraries(drift_correction ${OpenCV_LIBS})

ament_target_dependencies(aruco_tracker Eigen3 rclcpp px4_msgs sensor_msgs cv_bridge OpenCV tf2 tf2_geometry_msgs geometry_msgs)
ament_target_dependencies(simple_precision_land Eigen3 rclcpp px4_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(drift_correction Eigen3 rclcpp px4_msgs geometry_msgs)
ament_target_dependencies(vision_position_bridge rclcpp px4_msgs geometry_msgs tf2)

target_compile_features(aruco_tracker PUBLIC c_std_99 cxx_std_17)
target_compile_features(simple_precision_land PUBLIC c_std_99 cxx_std_17)
target_compile_features(drift_correction PUBLIC c_std_99 cxx_std_17)
target_compile_features(vision_position_bridge PUBLIC c_std_99 cxx_std_17)

install(TARGETS
  aruco_tracker
  simple_precision_land
  drift_correction
  vision_position_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
