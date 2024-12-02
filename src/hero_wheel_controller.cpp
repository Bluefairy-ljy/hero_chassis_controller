#include "hero_chassis_controller/hero_wheel_controller.h"

namespace hero_chassis_controller {

// 构造函数，初始化相关成员变量
HeroWheelController::HeroWheelController() {
    pid_controllers.resize(4);
    target_speeds.resize(4, 0);
    current_speeds.resize(4);
    joints.resize(4);
}
// init函数实现，完成控制器的初始化操作
bool HeroWheelController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &controller_nh) {
    // 获取四个轮子的关节句柄
    joints[0] = effort_joint_interface->getHandle("left_front_wheel_joint");
    joints[1] = effort_joint_interface->getHandle("right_front_wheel_joint");
    joints[2] = effort_joint_interface->getHandle("left_back_wheel_joint");
    joints[3] = effort_joint_interface->getHandle("right_back_wheel_joint");

    // 创建"pid"命名空间的节点句柄，用于初始化PID控制器，修改此处以适配参数所在的完整路径
    ros::NodeHandle base_pid_nh(controller_nh, "pid");

    // 记录每个轮子PID控制器初始化结果，初始化为false表示未成功初始化
    std::vector<bool> initResults(4, false);
    bool allInitialized = true;
    for (int i = 0; i < 4; i++) {
        std::stringstream ss;
        ss << (i == 0? "left_front_wheel" : (i == 1? "right_front_wheel" : (i == 2? "left_back_wheel" : "right_back_wheel")));

        // 为每个轮子创建对应完整路径的节点句柄，结合"pid"和轮子名称
        ros::NodeHandle pid_nh(base_pid_nh, ss.str());

        // 初始化PID控制器，传入对应轮子完整路径的节点句柄
        initResults[i] = pid_controllers[i].init(pid_nh);
        if (!initResults[i]) {
            ROS_ERROR_STREAM("Failed to initialize PID controller for wheel " << i);
            allInitialized = false;
            continue;
        }

        // 读取P、I、D参数并设置PID增益
        double p_gain, i_gain, d_gain;
        if (pid_nh.param<double>(pid_nh.getNamespace() + "/p", p_gain) &&
            pid_nh.param<double>(pid_nh.getNamespace() + "/i", i_gain) &&
            pid_nh.param<double>(pid_nh.getNamespace() + "/d", d_gain)) {
            pid_controllers[i].setGains(p_gain, i_gain, d_gain, std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), false);
        } else {
            ROS_ERROR_STREAM("Failed to read p, i, or d gain for " << ss.str() << ". Check the parameter name and configuration file.");
            initResults[i] = false;
            allInitialized = false;
            continue;
        }
    }

    if (allInitialized) {
        speed_subscriber = controller_nh.subscribe("wheel_speed_feedback", 10, &HeroWheelController::speedCallback, this);
        control_publisher = controller_nh.advertise<geometry_msgs::Twist>("control_output", 10);
        target_speed_subscriber = controller_nh.subscribe("target_speeds_topic", 10, &HeroWheelController::targetSpeedsCallback, this);
    }

    return allInitialized;
}

// update函数
void HeroWheelController::update(const ros::Time &time, const ros::Duration &period) {
    for (int i = 0; i < 4; ++i) {
        double error = target_speeds[i] - current_speeds[i];
        double control_output = pid_controllers[i].computeCommand(error, period);
        joints[i].setCommand(control_output);
    }
}

// starting函数，重置PID控制器，在控制器启动时调用
void HeroWheelController::starting(const ros::Time &time) {
    for (int i = 0; i < 4; ++i) {
        pid_controllers[i].reset();
    }
}

// stopping函数，目前为空实现，可按需后续完善，在控制器停止时调用
void HeroWheelController::stopping(const ros::Time &time) {}

// 速度回调函数，根据接收到的消息更新当前速度
void HeroWheelController::speedCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    current_speeds[0] = msg->linear.x;
    current_speeds[1] = msg->linear.x;
    current_speeds[2] = msg->linear.x;
    current_speeds[3] = msg->linear.x;
}

// 用于接收通过话题发布的目标速度的回调函数
void HeroWheelController::targetSpeedsCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    target_speeds[0] = msg->linear.x;
    target_speeds[1] = msg->linear.x;
    target_speeds[2] = msg->linear.x;
    target_speeds[3] = msg->linear.x;
}

} // namespace hero_chassis_controller