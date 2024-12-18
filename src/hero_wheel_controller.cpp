#include "hero_chassis_controller/hero_wheel_controller.h"

namespace hero_chassis_controller {

//构造函数
HeroWheelController::HeroWheelController() {
  pid_controllers.resize(4);
  target_speeds.resize(4, 0);
  current_speeds.resize(4);
  joints.resize(4);
  odom_helper = nullptr;
  hasReceivedCmd = false;
  final_target_speeds.resize(4, 0);
  target_speeds_last_cmd.resize(4);
}

//init函数
bool HeroWheelController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& controller_nh) {
  //获取四个轮子的关节句柄
  joints[0] = effort_joint_interface->getHandle("left_front_wheel_joint");
  joints[1] = effort_joint_interface->getHandle("right_front_wheel_joint");
  joints[2] = effort_joint_interface->getHandle("left_back_wheel_joint");
  joints[3] = effort_joint_interface->getHandle("right_back_wheel_joint");

  ros::NodeHandle base_pid_nh(controller_nh, "pid");
  std::vector<bool> initResults(4, false);
  bool allInitialized = true;
  std::string config_file_path = ros::package::getPath("hero_chassis_controller") + "/config/hero_chassis_controller.yaml";
  YAML::Node config = YAML::LoadFile(config_file_path);
  //读pid
  for (int i = 0; i < 4; i++) {
    std::stringstream ss;
    ss << (i == 0 ? "left_front_wheel" : (i == 1 ? "right_front_wheel" : (i == 2 ? "left_back_wheel" : "right_back_wheel")));
    ros::NodeHandle pid_nh(base_pid_nh, ss.str());
    initResults[i] = pid_controllers[i].init(pid_nh);
    if (!initResults[i]) {
      ROS_ERROR_STREAM("failed to initialize pid for wheel " << i);
      allInitialized = false;
      continue;
    }
    auto p_gain = config["hero_chassis_controller"]["pid"][ss.str()]["p"].as<double>();
    auto i_gain = config["hero_chassis_controller"]["pid"][ss.str()]["i"].as<double>();
    auto d_gain = config["hero_chassis_controller"]["pid"][ss.str()]["d"].as<double>();
    pid_controllers[i].setGains(p_gain, i_gain, d_gain, std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), false);
    if (initResults[i]) {
      ROS_INFO_STREAM("initialized pid for wheel " << i << " with p = " << p_gain << ", i = " << i_gain << ", d = " << d_gain);
    } else {
      ROS_ERROR_STREAM("failed to initialize pid for wheel " << i);
    }
  }
  //读底盘参数
  chassis_params.wheelbase = config["hero_chassis_controller"]["chassis"]["wheelbase"].as<double>();
  chassis_params.trackwidth = config["hero_chassis_controller"]["chassis"]["trackwidth"].as<double>();
  chassis_params.wheel_radius = config["hero_chassis_controller"]["chassis"]["wheel_radius"].as<double>();
  ROS_INFO_STREAM("wheelbase = " << chassis_params.wheelbase << ", trackwidth = " << chassis_params.trackwidth << ", wheel_radius = " << chassis_params.wheel_radius);
  //读速度模式
  int speed_mode = 0;
  if (config["hero_chassis_controller"]["speed_mode"]) {
    speed_mode = config["hero_chassis_controller"]["speed_mode"].as<int>();
  }
  this->speed_mode = speed_mode;
  //读加速度
  auto chassis_acceleration = config["hero_chassis_controller"]["chassis_acceleration"].as<double>();
  this->chassis_acceleration = chassis_acceleration;
  //订阅/cmd_vel话题
  cmd_vel_subscriber = controller_nh.subscribe("/cmd_vel", 10, &HeroWheelController::cmdVelCallback, this);
  //订阅/joint_states话题
  joint_state_subscriber = controller_nh.subscribe("/joint_states", 10, &HeroWheelController::jointStateCallback, this);

  odom_helper = new odometry_helper(chassis_params);
  return allInitialized;
}

//update函数
void HeroWheelController::update(const ros::Time& time, const ros::Duration& period) {
  std::vector<double> current_wheel_speeds = getCurrentWheelSpeeds();
  if (!current_wheel_speeds.empty()) {
    current_speeds = current_wheel_speeds;
    for (int i = 0; i < 4; ++i) {
      if (hasReceivedCmd) {
        if (target_speeds[i] < final_target_speeds[i]) {
          target_speeds[i] += chassis_acceleration * period.toSec();
          if (target_speeds[i] > final_target_speeds[i]) {
            target_speeds[i] = final_target_speeds[i];
          }
        }
      }
      double error = target_speeds[i] - current_speeds[i];
      double control_output = pid_controllers[i].computeCommand(error, period);
      joints[i].setCommand(control_output);
    }
    if (odom_helper) {
      odom_helper->update_odometry(current_wheel_speeds, time);
    }
  }
}

//starting函数
void HeroWheelController::starting(const ros::Time& time) {
  for (int i = 0; i < 4; ++i) {
    pid_controllers[i].reset();
  }
}

//stopping函数
void HeroWheelController::stopping(const ros::Time& time) {}

//getCurrentWheelSpeeds函数
std::vector<double> HeroWheelController::getCurrentWheelSpeeds() const {
  std::vector<double> current_speeds;
  for (const auto& joint : joints) {
    const std::string& joint_name = joint.getName();
    if (!joint_states.name.empty()) {
      for (size_t j = 0; j < joint_states.name.size(); ++j) {
        if (joint_states.name[j] == joint_name) {
          current_speeds.push_back(joint_states.velocity[j]);
          break;
        }
      }
    }
  }
  return current_speeds;
}

//cmdVelCallback函数
void HeroWheelController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  hasReceivedCmd = true;
  if (this->speed_mode == 0) {
    //底盘坐标系速度模式
    ROS_INFO("base_link");
    std::vector<double> wheel_speeds;
    kinematics_helper::inverseKinematics(*msg, chassis_params, wheel_speeds);
    final_target_speeds = wheel_speeds;
  } else if (this->speed_mode == 1) {
    //全局坐标系速度模式
    geometry_msgs::Twist cmd_vel_base_link;
    ROS_INFO("odom");
    ROS_INFO("try to transform");
    if (tf_helper.transformCmdVelToBaseLink(*msg, cmd_vel_base_link)) {
      std::vector<double> wheel_speeds;
      kinematics_helper::inverseKinematics(cmd_vel_base_link, chassis_params, wheel_speeds);
      final_target_speeds = wheel_speeds;
    } else {
      ROS_ERROR("fail to transform");
    }
  }
}

//jointStateCallback函数
void HeroWheelController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg) {
    joint_states = *msg;
  }
}

}