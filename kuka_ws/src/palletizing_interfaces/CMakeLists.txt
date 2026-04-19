cmake_minimum_required(VERSION 3.8)
project(palletizing_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找核心编译工具
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# 2. 生成 Action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/BoxTask.action"
  "srv/GetTask.srv"
  DEPENDENCIES action_msgs
)

# 3. ament_package() 必须永远放在文件的最后一行！
ament_package()