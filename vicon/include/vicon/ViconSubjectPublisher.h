#ifndef __VICON_VICON_SUBJECT_PUBLISHER_H__
#define __VICON_VICON_SUBJECT_PUBLISHER_H__

#include <ros/ros.h>
#include "vicon/Subject.h"
#include <Eigen/Geometry>

class ViconSubjectPublisher
{
 public:
  ViconSubjectPublisher(ros::Publisher &pub);

  void publish(vicon::Subject &subject);
  void calcAndSetTwist(vicon::Subject &subject);

 private:
  ros::Publisher publisher_;
  bool initialized_;
  Eigen::Vector3d position_history_[2];
  Eigen::Matrix3d orientation_history_[2];
  ros::Time timestamp_history_[2];
};

#endif
