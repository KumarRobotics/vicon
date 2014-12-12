#ifndef __VICON_VELOCITY_H__
#define __VICON_VELOCITY_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>

class ViconVelocity
{
 public:
  ViconVelocity();

  geometry_msgs::Twist getTwist(const tf::Pose &pose, const ros::Time &timestamp);

 private:
  bool initialized;
  Eigen::Affine3d pose_history[2];
  ros::Time timestamp_history[2];
};

#endif
