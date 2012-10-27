#include "vicon/ViconSubjectPublisher.h"

ViconSubjectPublisher::ViconSubjectPublisher(ros::Publisher &pub)
  : initialized_(false)
{
  publisher_ = pub;
}

void ViconSubjectPublisher::publish(vicon::Subject &subject)
{
  calcAndSetTwist(subject);
  publisher_.publish(subject);
}

template<class T>
static T getDerivative(const T x[], const double dt[])
{
  T num = (x[0]*dt[1]*(2*dt[0]+dt[1]) - x[1]*(dt[0]+dt[1])*(dt[0]+dt[1]) + x[2]*dt[0]*dt[0]);
  double den = (dt[0]*dt[1]*(dt[0]+dt[1]));
  return num/den;
}

void ViconSubjectPublisher::calcAndSetTwist(vicon::Subject &subject)
{
  const Eigen::Vector3d current_position(subject.position.x,
                                         subject.position.y,
                                         subject.position.z);
  const Eigen::Matrix3d current_orientation(Eigen::Quaterniond(
          subject.orientation.w,
          subject.orientation.x,
          subject.orientation.y,
          subject.orientation.z));

  if(!initialized_)
  {
    position_history_[0] = current_position;
    position_history_[1] = current_position;
    orientation_history_[0] = current_orientation;
    orientation_history_[1] = current_orientation;
    timestamp_history_[0] = subject.header.stamp - ros::Duration(0.01);
    timestamp_history_[1] = subject.header.stamp - ros::Duration(0.02);
    initialized_ = true;
  }

  const Eigen::Vector3d x[3] = {current_position, position_history_[0], position_history_[1]};
  const Eigen::Matrix3d R[3] = {current_orientation, orientation_history_[0],
    orientation_history_[1]};
  const double dt[2] = {(subject.header.stamp - timestamp_history_[0]).toSec(),
    (timestamp_history_[0] - timestamp_history_[1]).toSec()};

  Eigen::Vector3d v = getDerivative(x, dt);
  Eigen::Matrix3d R_dot = getDerivative(R, dt);
  Eigen::Matrix3d Omega = R_dot*R[0].transpose();

  subject.linear_velocity.x = v(0);
  subject.linear_velocity.y = v(1);
  subject.linear_velocity.z = v(2);
  subject.angular_velocity.x = Omega(2,1);
  subject.angular_velocity.y = Omega(0,2);
  subject.angular_velocity.z = Omega(1,0);

  position_history_[1] = position_history_[0];
  position_history_[0] = current_position;
  orientation_history_[1] = orientation_history_[0];
  orientation_history_[0] = current_orientation;
  timestamp_history_[1] = timestamp_history_[0];
  timestamp_history_[0] = subject.header.stamp;
}
