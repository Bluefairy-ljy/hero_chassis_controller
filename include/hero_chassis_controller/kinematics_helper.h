#ifndef KINEMATICS_HELPER_H
#define KINEMATICS_HELPER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

namespace kinematics_helper
{
  //结构体
  struct ChassisParams
  {
    double wheelbase;
    double trackwidth;
    double wheel_radius;
  };

  bool loadChassisParams(ChassisParams& chassis_params, const std::string& config_file_path);
  void inverseKinematics(const geometry_msgs::Twist& cmd_vel, const
                         ChassisParams& chassis_params, std::vector<double>& wheel_speeds);
  void forwardKinematics(const std::vector<double>& wheel_speeds, const ChassisParams& chassis_params, geometry_msgs::Twist& chassis_vel);
} // namespace kinematics_helper

#endif  // KINEMATICS_HELPER_H
