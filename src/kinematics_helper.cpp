#include "hero_chassis_controller/kinematics_helper.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

namespace kinematics_helper {

//读取底盘参数
bool loadChassisParams(ChassisParams& chassis_params, const std::string& config_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_file_path);
    chassis_params.wheelbase = config["hero_chassis_controller"]["chassis"]["wheelbase"].as<double>();
    chassis_params.trackwidth = config["hero_chassis_controller"]["chassis"]["trackwidth"].as<double>();
    chassis_params.wheel_radius = config["hero_chassis_controller"]["chassis"]["wheel_radius"].as<double>();
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to load chassis configuration file: " << e.what());
    return false;
  }
}

//逆运动学计算函数
void inverseKinematics(const geometry_msgs::Twist& cmd_vel, const ChassisParams& chassis_params, std::vector<double>& wheel_speeds) {
  Eigen::MatrixXd J(4, 3); // 4个轮子，3个底盘速度分量（vx, vy, omega_z）
  double L = chassis_params.wheelbase;
  double W = chassis_params.trackwidth;
  double r = chassis_params.wheel_radius;

  J << 1 / r, -1 / r, -(L + W) / (2 * r),
      1 / r, 1 / r, (L + W) / (2 * r),
      1 / r, 1 / r, -(L + W) / (2 * r),
      1 / r, -1 / r, (L + W) / (2 * r);

  ROS_INFO_STREAM("Jacobian matrix : " << std::endl << J);

  //构建底盘速度向量
  Eigen::Vector3d chassis_vel(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  //通过矩阵乘法计算轮子角速度向量
  Eigen::VectorXd wheel_ang_vels = J * chassis_vel;
  //计算轮子线速度并赋给wheel_speeds
  wheel_speeds.resize(4);
  for (int i = 0; i < 4; ++i) {
    wheel_speeds[i] = wheel_ang_vels(i) * r;
  }
  //输出轮子线速度向量值
  ROS_INFO_STREAM("Wheel linear velocities: " << Eigen::VectorXd::Map(wheel_speeds.data(), wheel_speeds.size()).transpose());
}

//正运动学计算函数
void forwardKinematics(const std::vector<double>& wheel_speeds, const ChassisParams& chassis_params, geometry_msgs::Twist& chassis_vel) {
  double L = chassis_params.wheelbase;
  double W = chassis_params.trackwidth;
  double r = chassis_params.wheel_radius;

  //线速度转换为角速度（废话真是多）
  std::vector<double> wheel_ang_vels(wheel_speeds.size());
  for (size_t i = 0; i < wheel_speeds.size(); ++i) {
    wheel_ang_vels[i] = wheel_speeds[i] / r;
  }

  //计算底盘速度分量
  double vx = (wheel_ang_vels[0] + wheel_ang_vels[1] + wheel_ang_vels[2] + wheel_ang_vels[3]) * r / 4;
  double vy = (-wheel_ang_vels[0] + wheel_ang_vels[1] + wheel_ang_vels[2] - wheel_ang_vels[3]) * r / 4;
  double omega_z = (-wheel_ang_vels[0] + wheel_ang_vels[1] - wheel_ang_vels[2] + wheel_ang_vels[3]) * r / (2 * (L + W));

  chassis_vel.linear.x = vx;
  chassis_vel.linear.y = vy;
  chassis_vel.angular.z = omega_z;

  // ROS_INFO("Calculated vx: %f", vx);
  // ROS_INFO("Calculated vy: %f", vy);
  // ROS_INFO("Calculated omega_z: %f", omega_z);
}

} // namespace kinematics_helper