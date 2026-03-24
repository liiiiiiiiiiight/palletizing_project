cmake_minimum_required(VERSION 3.5)

project(ee_description)

# 查找 ament_cmake 构建工具
find_package(ament_cmake REQUIRED)

# 可选：如果 launch 文件中使用了 Python launch 脚本，可能需要查找 ros2launch
# find_package(ros2launch REQUIRED)

# 安装目录定义
foreach(dir config launch meshes urdf)
    install(DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# 结束 ament 包
ament_package()