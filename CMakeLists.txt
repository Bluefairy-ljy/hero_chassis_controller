cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

# 查找所需的catkin包
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        control_toolbox
        geometry_msgs
        nav_msgs
        tf
        controller_interface
        controller_manager
        hardware_interface
        pluginlib
        gazebo_msgs
)
find_package(yaml-cpp REQUIRED)

# 定义catkin包的属性
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES hero_wheel_controller hero_chassis_controller_plugin
        CATKIN_DEPENDS roscpp std_msgs control_toolbox geometry_msgs nav_msgs tf controller_interface controller_manager hardware_interface pluginlib gazebo_msgs
)

# 添加头文件搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 构建hero_chassis_helper_library库
add_library(hero_chassis_helper_library
        src/kinematics_helper.cpp
        src/odometry_helper.cpp
        src/tf_transform_helper.cpp
)
# 链接hero_chassis_helper_library所需的库
target_link_libraries(hero_chassis_helper_library
        ${catkin_LIBRARIES}
)

# 构建hero_wheel_controller插件库
add_library(hero_chassis_controller_plugin
        src/hero_wheel_controller.cpp
)
# 链接hero_wheel_controller_plugin所需的库
target_link_libraries(hero_chassis_controller_plugin
        hero_chassis_helper_library
        ${catkin_LIBRARIES}
        yaml-cpp
)

# 安装插件描述文件
install(FILES hero_chassis_controller.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)