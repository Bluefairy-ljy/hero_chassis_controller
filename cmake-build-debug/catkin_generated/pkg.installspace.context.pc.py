# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;control_toolbox;geometry_msgs;nav_msgs;tf;controller_interface;controller_manager;hardware_interface;pluginlib;gazebo_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhero_wheel_controller;-lhero_chassis_controller_plugin".split(';') if "-lhero_wheel_controller;-lhero_chassis_controller_plugin" != "" else []
PROJECT_NAME = "hero_chassis_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"