#include "hero_chassis_controller//odometry_helper.h"

//构造函数
odometry_helper::odometry_helper(kinematics_helper::ChassisParams &params) : chassis_params(params), last_time(ros::Time::now()) {
    odom_publisher = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
    broadcaster = tf::TransformBroadcaster();
}

//析构函数
odometry_helper::~odometry_helper() = default;

//update_odometry函数
void odometry_helper::update_odometry(const std::vector<double>& wheel_speeds, const ros::Time& current_time) {
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) {
        return;
    }
    last_time = current_time;

    geometry_msgs::Twist chassis_vel;
    kinematics_helper::forwardKinematics(wheel_speeds, chassis_params, chassis_vel);

    update_odom_transform(chassis_vel, dt);
    publish_odometry(chassis_vel);
}

//update_odom_transform函数
void odometry_helper::update_odom_transform(const geometry_msgs::Twist& chassis_vel, double dt) {
    double vx = chassis_vel.linear.x;
    double vy = chassis_vel.linear.y;
    double w = chassis_vel.angular.z;

    double delta_x = (vx * cos(odom_trans.transform.rotation.z) - vy * sin(odom_trans.transform.rotation.z)) * dt;
    double delta_y = (vx * sin(odom_trans.transform.rotation.z) + vy * cos(odom_trans.transform.rotation.z)) * dt;
    double delta_th = w * dt;

    odom_trans.transform.translation.x += delta_x;
    odom_trans.transform.translation.y += delta_y;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_trans.transform.rotation.z + delta_th);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = odom_trans.transform.translation.x;
    transform_stamped.transform.translation.y = odom_trans.transform.translation.y;
    transform_stamped.transform.translation.z = odom_trans.transform.translation.z;
    transform_stamped.transform.rotation = odom_trans.transform.rotation;
    broadcaster.sendTransform(transform_stamped);
}

//publish_odometry函数
void odometry_helper::publish_odometry(const geometry_msgs::Twist& chassis_vel) {
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = odom_trans.transform.translation.x;
    odom.pose.pose.position.y = odom_trans.transform.translation.y;
    odom.pose.pose.position.z = 0.0;

    odom.twist.twist = chassis_vel;

    odom_publisher.publish(odom);
}