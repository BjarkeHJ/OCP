cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Define executeable name
set(EXEC_NAME planner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(PCL 1.14 REQUIRED)
find_package(pcl_conversions REQUIRED)

# Add executeable from source
add_executable(${EXEC_NAME} src/planner.cpp)

# Handle non-ROS2 packages
target_include_directories(${EXEC_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${EXEC_NAME} ${PCL_LIBRARIES})

# Handle ROS2 specific dependencies
ament_target_dependencies(${EXEC_NAME}
                          rclcpp
                          sensor_msgs
                          pcl_conversions
                          px4_msgs)


install(TARGETS ${EXEC_NAME} DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
