#ifndef TF_TRANSFORM_HELPER_H
#define TF_TRANSFORM_HELPER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace tf_transform_helper
{
  class TFTransformHelper
  {
  public:
    TFTransformHelper();
    ~TFTransformHelper();
    bool transformCmdVelToBaseLink(const geometry_msgs::Twist& cmd_vel_odom, geometry_msgs::Twist& cmd_vel_base_link);

  private:
    tf::Transformer transformer;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
  };
}

#endif

