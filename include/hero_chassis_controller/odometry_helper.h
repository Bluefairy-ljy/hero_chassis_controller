#ifndef ODOMETRY_HELPER_H
#define ODOMETRY_HELPER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "hero_chassis_controller/kinematics_helper.h"

class odometry_helper {
public:
  explicit odometry_helper(kinematics_helper::ChassisParams& params);
  ~odometry_helper();
  void update_odometry(const std::vector<double>& wheel_speeds, const ros::Time& current_time);

private:
  ros::NodeHandle nh;
  ros::Publisher odom_publisher;
  ros::Time last_time;
  kinematics_helper::ChassisParams& chassis_params;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;

  void update_odom_transform(const geometry_msgs::Twist& chassis_vel, double dt);
  void publish_odometry(const geometry_msgs::Twist& chassis_vel);
};

#endif