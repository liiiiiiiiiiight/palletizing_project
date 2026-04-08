cmake_minimum_required(VERSION 3.8)
project(kuka_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(arm_controller_node src/arm_controller_node.cpp)
target_include_directories(arm_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_controller_node
  "rclcpp"
  "std_msgs"
)

install(TARGETS arm_controller_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(box_creator src/box_creator.cpp)
ament_target_dependencies(box_creator
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  std_msgs
)

install(TARGETS box_creator
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(suction_manipulator src/suction_manipulator.cpp)
ament_target_dependencies(suction_manipulator 
  rclcpp 
  moveit_ros_planning_interface 
  std_msgs 
  geometry_msgs
)

install(TARGETS
  suction_manipulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
