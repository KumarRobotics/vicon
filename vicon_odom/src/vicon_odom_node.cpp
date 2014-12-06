#include "vicon_odom/vicon_odom_node.hpp"

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>

namespace vicon_odom {

ViconOdomNode::ViconOdomNode(const ros::NodeHandle &nh,
                             const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  double max_accel;
  pnh_.param("max_accel", max_accel, 5.0);
  pnh_.param("publish_tf", publish_tf_, false);
  std::string model;
  pnh_.param<std::string>("model", model, "");
  if (model.empty()) throw std::runtime_error("vicon_odom: empty model name");
  // There should only be one vicon_fps, so we read from nh instead of pnh
  double dt, vicon_fps;
  nh_.param("vicon_fps", vicon_fps, 100.0);
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
  vicon_sub_ = nh_.subscribe(model, 10, &ViconOdomNode::ViconCallback, this,
                             ros::TransportHints().tcpNoDelay());
  // Publish odometry under /vicon/model/odom
  odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("odom", 10);
}

void ViconOdomNode::ViconCallback(const vicon::Subject::ConstPtr &vicon_msg) {
  static ros::Time t_last_proc = vicon_msg->header.stamp;
  double dt = (vicon_msg->header.stamp - t_last_proc).toSec();
  t_last_proc = vicon_msg->header.stamp;

  // Kalman filter for getting translational velocity from position measurements
  kf_.processUpdate(dt);
  const KalmanFilter::Measurement_t meas(
      vicon_msg->position.x, vicon_msg->position.y, vicon_msg->position.z);
  if (!vicon_msg->occluded) {
    static ros::Time t_last_meas = vicon_msg->header.stamp;
    double meas_dt = (vicon_msg->header.stamp - t_last_meas).toSec();
    t_last_meas = vicon_msg->header.stamp;
    kf_.measurementUpdate(meas, meas_dt);
  }

  const KalmanFilter::State_t state = kf_.getState();
  const KalmanFilter::ProcessCov_t proc_noise = kf_.getProcessNoise();

  nav_msgs::Odometry odom_msg;
  odom_msg.header = vicon_msg->header;
  odom_msg.child_frame_id = vicon_msg->name;
  odom_msg.pose.pose.position.x = state(0);
  odom_msg.pose.pose.position.y = state(1);
  odom_msg.pose.pose.position.z = state(2);
  odom_msg.twist.twist.linear.x = state(3);
  odom_msg.twist.twist.linear.y = state(4);
  odom_msg.twist.twist.linear.z = state(5);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      odom_msg.pose.covariance[6 * i + j] = proc_noise(i, j);
      odom_msg.twist.covariance[6 * i + j] = proc_noise(3 + i, 3 + j);
    }
  }

  odom_msg.pose.pose.orientation = vicon_msg->orientation;

  // Single step differentitation for angular velocity
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(
      Eigen::Quaterniond(vicon_msg->orientation.w, vicon_msg->orientation.x,
                         vicon_msg->orientation.y, vicon_msg->orientation.z));
  if (dt > 1e-6) {
    const Eigen::Matrix3d R_dot = (R - R_prev) / dt;
    const Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.twist.angular.z = w_hat(1, 0);
  }
  R_prev = R;

  odom_pub_.publish(odom_msg);
  if (publish_tf_) {
    PublishTransform(odom_msg.pose.pose, odom_msg.header,
                     odom_msg.child_frame_id);
  }
}

void ViconOdomNode::PublishTransform(const geometry_msgs::Pose &pose,
                                     const std_msgs::Header &header,
                                     const std::string &child_frame_id) {
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

}  // namespace vicon_odom
