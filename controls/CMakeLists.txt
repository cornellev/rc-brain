
cmake_minimum_required(VERSION 3.8)
project(controls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(cev_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

add_executable(autobrake_node src/autobrake_node.cpp)
add_executable(trajectory_follower_node src/trajectory_follower.cpp)
add_executable(debug_trajectory src/debug_trajectory_publish.cpp)

ament_target_dependencies(autobrake_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  ackermann_msgs 
  cev_msgs)

ament_target_dependencies(trajectory_follower_node
  rclcpp 
  std_msgs 
  sensor_msgs 
  ackermann_msgs 
  cev_msgs
  nav_msgs)

ament_target_dependencies(debug_trajectory
  rclcpp 
  std_msgs 
  sensor_msgs 
  ackermann_msgs 
  cev_msgs
  nav_msgs)

install(TARGETS autobrake_node trajectory_follower_node debug_trajectory
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

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

