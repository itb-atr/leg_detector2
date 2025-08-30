cmake_minimum_required(VERSION 3.8)

project(leg_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (OpenCV_DIR /usr/local/include)
include_directories(include ${OpenCV_INCLUDE_DIR})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(leg_detector_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# add the executables
add_executable (detect_leg_clusters
                src/detect_leg_clusters.cpp
                src/laser_processor.cpp
                src/cluster_features.cpp
                include/leg_detector/laser_processor.h
                include/leg_detector/cluster_features.h
                )
target_link_libraries(detect_leg_clusters PUBLIC rclcpp::rclcpp  ${leg_detector_msgs_TARGETS}  ${OpenCV_LIBS}  ${sensor_msgs_TARGETS}  ${tf2_TARGETS}  ${tf2_geometry_msgs_TARGETS}  ${tf2_ros_TARGETS}  ${visualization_msgs_TARGETS})

add_executable (trajectory_prediction
                src/trajectory_prediction.cpp
                )
target_link_libraries(trajectory_prediction PUBLIC rclcpp::rclcpp  ${leg_detector_msgs_TARGETS}  ${visualization_msgs_TARGETS})

add_executable (local_occupancy_grid_mapping
                src/laser_processor.cpp
                src/local_occupancy_grid_mapping.cpp
                include/leg_detector/laser_processor.h
                )
target_link_libraries(local_occupancy_grid_mapping PUBLIC rclcpp::rclcpp  ${leg_detector_msgs_TARGETS}  ${OpenCV_LIBS}  ${sensor_msgs_TARGETS}  ${tf2_TARGETS}  ${tf2_ros_TARGETS}  ${visualization_msgs_TARGETS}  ${nav_msgs_TARGETS}  ${tf2_geometry_msgs_TARGETS})

add_executable (inflated_human_scan
                src/inflated_human_scan.cpp
                )
target_link_libraries(inflated_human_scan PUBLIC rclcpp::rclcpp  ${leg_detector_msgs_TARGETS}  ${sensor_msgs_TARGETS}  ${message_filters_TARGETS})

install (TARGETS
  detect_leg_clusters
  local_occupancy_grid_mapping
  inflated_human_scan
  trajectory_prediction
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rosbag DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS
  scripts/joint_leg_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
