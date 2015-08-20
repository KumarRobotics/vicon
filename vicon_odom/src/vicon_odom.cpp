#include "vicon_odom/vicon_odom.hpp"

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

namespace vicon_odom
{
ViconOdom::ViconOdom(ros::NodeHandle &nh)
{
  double max_accel;
  nh.param("max_accel", max_accel, 5.0);
  nh.param("publish_tf", publish_tf_, false);
  nh.param<std::string>("child_frame_id", child_frame_id_, "base_link");
  if(publish_tf_ && child_frame_id_.empty())
    throw std::runtime_error("vicon_odom: child_frame_id required for publishing tf");

  // There should only be one vicon_fps, so we read from nh
  double dt, vicon_fps;
  nh.param("vicon_fps", vicon_fps, 100.0);
  ROS_ASSERT(vicon_fps > 0.0);
  dt = 1 / vicon_fps;

  // Initialize KalmanFilter
  KalmanFilter::State_t proc_noise_diag;
  proc_noise_diag(0) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(1) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(2) = 0.5 * max_accel * dt * dt;
  proc_noise_diag(3) = max_accel * dt;
  proc_noise_diag(4) = max_accel * dt;
  proc_noise_diag(5) = max_accel * dt;
  proc_noise_diag = proc_noise_diag.array().square();
  KalmanFilter::Measurement_t meas_noise_diag;
  meas_noise_diag(0) = 1e-4;
  meas_noise_diag(1) = 1e-4;
  meas_noise_diag(2) = 1e-4;
  meas_noise_diag = meas_noise_diag.array().square();
  kf_.initialize(KalmanFilter::State_t::Zero(),
                 0.01 * KalmanFilter::ProcessCov_t::Identity(),
                 proc_noise_diag.asDiagonal(), meas_noise_diag.asDiagonal());

  // Initialize publisher and subscriber
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  vicon_sub_ = nh.subscribe("vicon_subject", 10, &ViconOdom::ViconCallback,
                            this, ros::TransportHints().tcpNoDelay());
}

void ViconOdom::ViconCallback(const vicon::Subject::ConstPtr &msg)
{
  static ros::Time t_last_proc = msg->header.stamp;
  double dt = (msg->header.stamp - t_last_proc).toSec();
  t_last_proc = msg->header.stamp;

  // Kalman filter for getting translational velocity from position measurements
  kf_.processUpdate(dt);
  const KalmanFilter::Measurement_t meas(msg->position.x, msg->position.y,
                                         msg->position.z);
  if(!msg->occluded)
  {
    static ros::Time t_last_meas = msg->header.stamp;
    double meas_dt = (msg->header.stamp - t_last_meas).toSec();
    t_last_meas = msg->header.stamp;
    kf_.measurementUpdate(meas, meas_dt);
  }

  const KalmanFilter::State_t state = kf_.getState();
  const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();

  nav_msgs::Odometry odom_msg;
  odom_msg.header = msg->header;
  odom_msg.child_frame_id = child_frame_id_;
  odom_msg.pose.pose.position.x = state(0);
  odom_msg.pose.pose.position.y = state(1);
  odom_msg.pose.pose.position.z = state(2);
  odom_msg.twist.twist.linear.x = state(3);
  odom_msg.twist.twist.linear.y = state(4);
  odom_msg.twist.twist.linear.z = state(5);
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      odom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
      odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
    }
  }

  odom_msg.pose.pose.orientation = msg->orientation;

  // Single step differentitation for angular velocity
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                       msg->orientation.y, msg->orientation.z));
  if(dt > 1e-6)
  {
    const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
    const Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.twist.angular.z = w_hat(1, 0);
  }
  R_prev = R;

  odom_pub_.publish(odom_msg);
  if(publish_tf_)
  {
    PublishTransform(odom_msg.pose.pose, odom_msg.header,
                     odom_msg.child_frame_id);
  }
}

void ViconOdom::PublishTransform(const geometry_msgs::Pose &pose,
                                 const std_msgs::Header &header,
                                 const std::string &child_frame_id)
{
  // Publish tf
  geometry_msgs::Vector3 translation;
  translation.x = pose.position.x;
  translation.y = pose.position.y;
  translation.z = pose.position.z;

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header = header;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_.sendTransform(transform_stamped);
}

} // namespace vicon_odom

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_odom");
  ros::NodeHandle nh("~");

  try
  {
    vicon_odom::ViconOdom vicon_odom(nh);
    ros::spin();
  }
  catch(const std::exception &e)
  {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    return 1;
  }
  return 0;
}
