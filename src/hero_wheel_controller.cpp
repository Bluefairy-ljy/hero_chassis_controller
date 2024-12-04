#include "hero_chassis_controller/hero_wheel_controller.h"

namespace hero_chassis_controller {
// 构造函数
HeroWheelController::HeroWheelController() {
pid_controllers.resize(4);
target_speeds.resize(4, 0);
current_speeds.resize(4);
joints.resize(4);
}

// init函数
bool HeroWheelController::init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &controller_nh) {
// 获取四个轮子的关节句柄
joints[0] = effort_joint_interface->getHandle("left_front_wheel_joint");
joints[1] = effort_joint_interface->getHandle("right_front_wheel_joint");
joints[2] = effort_joint_interface->getHandle("left_back_wheel_joint");
joints[3] = effort_joint_interface->getHandle("right_back_wheel_joint");

ros::NodeHandle base_pid_nh(controller_nh, "pid");
std::vector<bool> initResults(4, false);
bool allInitialized = true;

std::string config_file_path = ros::package::getPath("hero_chassis_controller") + "/config/hero_chassis_controller.yaml";
YAML::Node config = YAML::LoadFile(config_file_path);

for (int i = 0; i < 4; i++) {
std::stringstream ss;
ss << (i == 0? "left_front_wheel" : (i == 1? "right_front_wheel" : (i == 2? "left_back_wheel" : "right_back_wheel")));

ros::NodeHandle pid_nh(base_pid_nh, ss.str());

initResults[i] = pid_controllers[i].init(pid_nh);
if (!initResults[i]) {
ROS_ERROR_STREAM("Failed to initialize PID controller for wheel " << i);
allInitialized = false;
continue;
}

double p_gain = config["hero_chassis_controller"]["pid"][ss.str()]["p"].as<double>();
double i_gain = config["hero_chassis_controller"]["pid"][ss.str()]["i"].as<double>();
double d_gain = config["hero_chassis_controller"]["pid"][ss.str()]["d"].as<double>();
pid_controllers[i].setGains(p_gain, i_gain, d_gain, std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), false);

if (initResults[i]) {
ROS_INFO_STREAM("Initialized PID controller for wheel " << i << " with p = " << p_gain << ", i = " << i_gain << ", d = " << d_gain);
} else {
ROS_ERROR_STREAM("Failed to initialize PID controller for wheel " << i);
}
}

chassis_params.wheelbase = config["hero_chassis_controller"]["chassis"]["wheelbase"].as<double>();
chassis_params.trackwidth = config["hero_chassis_controller"]["chassis"]["trackwidth"].as<double>();
chassis_params.wheel_radius = config["hero_chassis_controller"]["chassis"]["wheel_radius"].as<double>();
ROS_INFO_STREAM("Loaded chassis params: wheelbase = " << chassis_params.wheelbase << ", trackwidth = " << chassis_params.trackwidth << ", wheel_radius = " << chassis_params.wheel_radius);

// 订阅 /cmd_vel 话题
cmd_vel_subscriber = controller_nh.subscribe("/cmd_vel", 10, &HeroWheelController::cmdVelCallback, this);
if (allInitialized) {
speed_subscriber = controller_nh.subscribe("wheel_speed_feedback", 10, &HeroWheelController::speedCallback, this);
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

// starting函数
void HeroWheelController::starting(const ros::Time &time) {
for (int i = 0; i < 4; ++i) {
pid_controllers[i].reset();
}
}

// stopping函数
void HeroWheelController::stopping(const ros::Time &time) {}

// 速度回调函数
void HeroWheelController::speedCallback(const geometry_msgs::Twist::ConstPtr &msg) {
current_speeds[0] = msg->linear.x;
current_speeds[1] = msg->linear.x;
current_speeds[2] = msg->linear.x;
current_speeds[3] = msg->linear.x;
}

// 接收底盘速度指令话题的回调函数
void HeroWheelController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
// 输出接收到的底盘速度指令
ROS_INFO_STREAM("Received cmd_vel: vx = " << msg->linear.x << ", vy = " << msg->linear.y << ", omega_z = " << msg->angular.z);
std::vector<double> wheel_speeds;
kinematics_helper::inverseKinematics(*msg, chassis_params, wheel_speeds);
// 输出计算得到的轮子速度
ROS_INFO_STREAM("Calculated wheel speeds: wheel1 = " << wheel_speeds[0] << ", wheel2 = " << wheel_speeds[1] << ", wheel3 = " << wheel_speeds[2] << ", wheel4 = " << wheel_speeds[3]);
target_speeds[0] = wheel_speeds[0];
target_speeds[1] = wheel_speeds[1];
target_speeds[2] = wheel_speeds[2];
target_speeds[3] = wheel_speeds[3];
}

} // namespace hero_chassis_controller