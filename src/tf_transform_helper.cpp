#include "hero_chassis_controller/tf_transform_helper.h"

namespace tf_transform_helper {
//构造函数
TFTransformHelper::TFTransformHelper() {
  transformer = tf::Transformer();
  broadcaster = tf::TransformBroadcaster();
}

TFTransformHelper::~TFTransformHelper() = default;

//transformCmdVelToBaseLink函数
bool TFTransformHelper::transformCmdVelToBaseLink(const geometry_msgs::Twist& cmd_vel_odom, geometry_msgs::Twist& cmd_vel_base_link) {
  try {
    tf::StampedTransform transform;
    // odom到base_link坐标变换
    ROS_INFO("try lookupTransform");
    listener.lookupTransform("base_link", "odom", ros::Time::now(), transform);
    ROS_INFO("lookupTransform succeeded");
    // 配Stamped<tf::Vector3>
    tf::Stamped<tf::Vector3> stamped_in_linear;
    stamped_in_linear.setData(tf::Vector3(cmd_vel_odom.linear.x, cmd_vel_odom.linear.y, 0.0));
    stamped_in_linear.frame_id_ = "odom";
    stamped_in_linear.stamp_ = ros::Time::now();
    tf::Stamped<tf::Vector3> stamped_in_angular;
    stamped_in_angular.setData(tf::Vector3(0.0, 0.0, cmd_vel_odom.angular.z));
    stamped_in_angular.frame_id_ = "odom";
    stamped_in_angular.stamp_ = ros::Time::now();
    // 线速度坐标变换
    tf::Stamped<tf::Vector3> stamped_out_linear;
    listener.transformVector("base_link", stamped_in_linear, stamped_out_linear);
    // 角速度坐标变换
    tf::Stamped<tf::Vector3> stamped_out_angular;
    listener.transformVector("base_link", stamped_in_angular, stamped_out_angular);
    // 线速度变换后赋值
    tf::Stamped<tf::Vector3> linear_vector = stamped_out_linear;
    cmd_vel_base_link.linear.x = linear_vector.x();
    cmd_vel_base_link.linear.y = linear_vector.y();
    cmd_vel_base_link.linear.z = linear_vector.z();
    // 角速度变换后赋值
    tf::Stamped<tf::Vector3> angular_vector = stamped_out_angular;
    cmd_vel_base_link.angular.x = angular_vector.x();
    cmd_vel_base_link.angular.y = angular_vector.y();
    cmd_vel_base_link.angular.z = angular_vector.z();
    return true;
  } catch (tf::TransformException& ex) {
    ROS_ERROR("failed to transform odom to base_link with error: %s", ex.what());
    return false;
  } catch (...) {
    ROS_ERROR("fail to transform odom to base_link");
    return false;
  }
}

}
