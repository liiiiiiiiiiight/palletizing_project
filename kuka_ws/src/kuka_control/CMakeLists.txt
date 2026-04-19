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

# add_executable(arm_controller_node src/arm_controller_node.cpp)
# target_include_directories(arm_controller_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(arm_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   arm_controller_node
#   "rclcpp"
#   "std_msgs"
# )

# install(TARGETS arm_controller_node
#   DESTINATION lib/${PROJECT_NAME})

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

# --- 1. 查找新增的依赖 ---
find_package(rclcpp_action REQUIRED)
find_package(kuka_interfaces REQUIRED)
find_package(palletizing_interfaces REQUIRED)

# --- 2. 编译动作服务端 (Server) ---
add_executable(palletizing_server src/palletizing_server.cpp)
ament_target_dependencies(palletizing_server 
  rclcpp 
  rclcpp_action
  moveit_ros_planning_interface 
  kuka_interfaces
  # ... (如果你还需要用到 tf2 或其他包，一并加在这里)
)

# --- 3. 编译动作客户端 (Client) - 从 brain 系统接入 ---
add_executable(task_publisher_client src/task_publisher_client.cpp)
ament_target_dependencies(task_publisher_client
  rclcpp
  rclcpp_action
  kuka_interfaces
  palletizing_interfaces
)

# --- 4. 安装可执行文件 ---
install(TARGETS
  palletizing_server
  task_publisher_client
  DESTINATION lib/${PROJECT_NAME}
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
