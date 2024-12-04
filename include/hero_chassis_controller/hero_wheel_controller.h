#ifndef HERO_WHEEL_CONTROLLER_H
#define HERO_WHEEL_CONTROLLER_H

#include "controller_interface/controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <vector>
#include <rosbag/bag.h>
#include <hero_chassis_controller/kinematics_helper.h>
namespace hero_chassis_controller {
class HeroWheelController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  HeroWheelController();
  ~HeroWheelController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
  void starting(const ros::Time &time) override;
  void stopping(const ros::Time &time) override;
  void setWheelCommands(const std::vector<double> &commands);
  const std::vector<control_toolbox::Pid>&getPidControls() const {return pid_controllers;};

private:
  std::vector<control_toolbox::Pid> pid_controllers;
  std::vector<double> target_speeds;
  std::vector<double> current_speeds;
  std::vector<hardware_interface::JointHandle> joints;
  ros::Subscriber speed_subscriber;
  kinematics_helper::ChassisParams chassis_params;
  ros::Subscriber cmd_vel_subscriber;
  void speedCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

// 使用PLUGINLIB_EXPORT_CLASS宏注册插件，指定继承自controller_interface::ControllerBase
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroWheelController, controller_interface::ControllerBase)
}
#endif