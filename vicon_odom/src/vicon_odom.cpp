#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <vicon/Subject.h>
#include "vicon_odom/filter.h"
#include <Eigen/Geometry>

static ros::Publisher odom_pub;
static KalmanFilter kf;
static nav_msgs::Odometry odom_msg;

static void vicon_callback(const vicon::Subject::ConstPtr &msg)
{
  static ros::Time t_last_proc = msg->header.stamp;

  double dt = (msg->header.stamp - t_last_proc).toSec();
  t_last_proc = msg->header.stamp;

  // Kalman filter for getting translational velocity from position measurements
  kf.processUpdate(dt);
  const KalmanFilter::Measurement_t meas(msg->position.x, msg->position.y, msg->position.z);
  if(!msg->occluded)
  {
    static ros::Time t_last_meas = msg->header.stamp;
    double meas_dt = (msg->header.stamp - t_last_meas).toSec();
    t_last_meas = msg->header.stamp;
    kf.measurementUpdate(meas, meas_dt);
  }

  const KalmanFilter::State_t state = kf.getState();
  const KalmanFilter::ProcessCov_t proc_noise = kf.getProcessNoise();

  odom_msg.header.seq = msg->header.seq;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = msg->header.frame_id;
  odom_msg.child_frame_id = msg->name;
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
      odom_msg.pose.covariance[6*i+j] = proc_noise(i,j);
      odom_msg.twist.covariance[6*i+j] = proc_noise(3+i, 3+j);
    }
  }

  odom_msg.pose.pose.orientation.x = msg->orientation.x;
  odom_msg.pose.pose.orientation.y = msg->orientation.y;
  odom_msg.pose.pose.orientation.z = msg->orientation.z;
  odom_msg.pose.pose.orientation.w = msg->orientation.w;

  // Single step differentitation for angular velocity
  static Eigen::Matrix3d R_prev(Eigen::Matrix3d::Identity());
  Eigen::Matrix3d R(Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z));
  if(dt > 1e-6)
  {
    Eigen::Matrix3d R_dot = (R - R_prev)/dt;
    Eigen::Matrix3d w_hat = R_dot * R.transpose();

    odom_msg.twist.twist.angular.x = w_hat(2, 1);
    odom_msg.twist.twist.angular.y = w_hat(0, 2);
    odom_msg.twist.twist.angular.z = w_hat(1, 0);
  }
  R_prev = R;

  odom_pub.publish(odom_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_odom");

  ros::NodeHandle n("~");

  double max_accel;
  n.param("max_accel", max_accel, 5.0);

  double dt, vicon_fps;
  n.param("vicon_fps", vicon_fps, 100.0);
  ROS_ASSERT(vicon_fps > 0.0);
  dt = 1/vicon_fps;

  KalmanFilter::State_t proc_noise_diag;
  proc_noise_diag(0) = 0.5*max_accel*dt*dt;
  proc_noise_diag(1) = 0.5*max_accel*dt*dt;
  proc_noise_diag(2) = 0.5*max_accel*dt*dt;
  proc_noise_diag(3) = max_accel*dt;
  proc_noise_diag(4) = max_accel*dt;
  proc_noise_diag(5) = max_accel*dt;
  proc_noise_diag = proc_noise_diag.array().square();
  KalmanFilter::Measurement_t meas_noise_diag;
  meas_noise_diag(0) = 1e-4;
  meas_noise_diag(1) = 1e-4;
  meas_noise_diag(2) = 1e-4;
  meas_noise_diag = meas_noise_diag.array().square();
  kf.initialize(KalmanFilter::State_t::Zero(),
                0.01*KalmanFilter::ProcessCov_t::Identity(),
                proc_noise_diag.asDiagonal(),
                meas_noise_diag.asDiagonal());

  ros::Subscriber vicon_sub = n.subscribe("vicon", 10, &vicon_callback,
                                          ros::TransportHints().tcpNoDelay());

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

  ros::spin();

  return 0;
}
