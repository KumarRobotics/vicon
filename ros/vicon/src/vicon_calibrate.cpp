#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include "ViconCalib.h"
#include "vicon/Subject.h"
#include "vicon/SetPose.h"

static ros::Publisher zero_pose_pub;
static std::map<std::string, Eigen::Vector3d> marker_pos_map;
static std::vector<Eigen::Vector3d> calib_ref_points, calib_actual_points;
static Eigen::Affine3d calib_transform;
static bool enable_calibration = false;
static bool calib_stand_ready = false;
static ros::ServiceClient set_zero_pose_srv;

static void calib_stand_callback(const vicon::Subject::ConstPtr &msg)
{
  calib_ref_points.clear();
  calib_actual_points.clear();

  std::map<std::string, Eigen::Vector3d>::iterator it;
  for (size_t i = 0; i < msg->markers.size(); i++)
  {
    if(!msg->markers[i].occluded)
    {
      it = marker_pos_map.find(msg->markers[i].name);
      if(it != marker_pos_map.end())
      {
        calib_ref_points.push_back(it->second);
        calib_actual_points.push_back(Eigen::Vector3d(msg->markers[i].position.x,
                                                      msg->markers[i].position.y,
                                                      msg->markers[i].position.z));
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Marker %s is not present in calib_marker_pos_file"
                          ", skipping it",
                          msg->markers[i].name.c_str());
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1, "Marker %s is occluded, skipping it",
                        msg->markers[i].name.c_str());
    }
  }
  if(!ViconCalib::getTransform(calib_ref_points, calib_actual_points, calib_transform))
  {
    ROS_WARN("ViconCalib::getTransform failed");
    calib_stand_ready = false;
  }
  else
  {
    calib_stand_ready = true;
  }
}

static void subject_callback(const vicon::Subject::ConstPtr &msg)
{
  static bool calibrating = false;
  static double t_x, t_y, t_z;
  static double q_x, q_y, q_z, q_w;
  static unsigned int count;

  if(enable_calibration && !calibrating)
  {
    // Start calib
    t_x = t_y = t_z = 0;
    q_x = q_y = q_z = q_w = 0;
    count = 0;
    calibrating = true;
  }
  else if(!enable_calibration && calibrating)
  {
    // Stop calib
    calibrating = false;

    vicon::SetPose pose;
    Eigen::Quaterniond q(q_w/count, q_x/count, q_y/count, q_z/count);
    q.normalize();

    pose.request.subject_name = msg->name;
    pose.request.pose.position.x = t_x/count;
    pose.request.pose.position.y = t_y/count;
    pose.request.pose.position.z = t_z/count;
    pose.request.pose.orientation.x = q.x();
    pose.request.pose.orientation.y = q.y();
    pose.request.pose.orientation.z = q.z();
    pose.request.pose.orientation.w = q.w();
    set_zero_pose_srv.call(pose);
  }

  if(calibrating)
  {
    Eigen::Affine3d zero_pose, current_pose;
    Eigen::Vector3d t(msg->position.x,
                      msg->position.y,
                      msg->position.z);
    Eigen::Quaterniond q(msg->orientation.w,
                         msg->orientation.x,
                         msg->orientation.y,
                         msg->orientation.z);
    current_pose.setIdentity();
    current_pose.translate(t);
    current_pose.rotate(q);

    zero_pose = calib_transform.inverse() * current_pose;

    t = zero_pose.translation();
    q = Eigen::Quaterniond(zero_pose.rotation());

    // Accumulate (to calculate mean)
    t_x += t(0);
    t_y += t(1);
    t_z += t(2);
    q_x += q.x();
    q_y += q.y();
    q_z += q.z();
    q_w += q.w();
    count++;

    geometry_msgs::PoseStamped::Ptr zero_pose_msg(new geometry_msgs::PoseStamped);
    zero_pose_msg->header.stamp = msg->header.stamp;
    zero_pose_msg->header.frame_id = "/vicon";

    zero_pose_msg->pose.position.x = t_x/count;
    zero_pose_msg->pose.position.y = t_y/count;
    zero_pose_msg->pose.position.z = t_z/count;

    q = Eigen::Quaterniond(q_w/count, q_x/count, q_y/count, q_z/count);
    q.normalize();

    zero_pose_msg->pose.orientation.x = q.x();
    zero_pose_msg->pose.orientation.y = q.y();
    zero_pose_msg->pose.orientation.z = q.z();
    zero_pose_msg->pose.orientation.w = q.w();

    zero_pose_pub.publish(zero_pose_msg);
  }
}

static bool toggle_calib_callback(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
{
  if(!calib_stand_ready)
  {
    ROS_ERROR("Do not have calib stand pose, cannot calibrate");
    enable_calibration = false;
    return false;
  }

  enable_calibration = !enable_calibration;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_calibrate");

  ros::NodeHandle nh("~");

  std::string calib_marker_pos_file;
  nh.param("calib_marker_pos_file", calib_marker_pos_file,
           std::string("QuadrotorCalib.yaml"));

  if(!ViconCalib::loadCalibMarkerPos(calib_marker_pos_file, marker_pos_map))
  {
    ROS_FATAL("Error loading calib marker positions from file: %s",
              calib_marker_pos_file.c_str());
    return -1;
  }

  calib_ref_points.resize(marker_pos_map.size());
  calib_actual_points.resize(marker_pos_map.size());

  ros::Subscriber calib_sub = nh.subscribe("vicon_calib", 10, &calib_stand_callback,
                                           ros::TransportHints().tcp().tcpNoDelay());
  ros::Subscriber subject_sub = nh.subscribe("vicon_subject", 10, &subject_callback,
                                             ros::TransportHints().tcp().tcpNoDelay());

  zero_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("zero_pose", 10);

  set_zero_pose_srv = nh.serviceClient<vicon::SetPose>("set_zero_pose");

  ros::ServiceServer toggle_calib_srv =
      nh.advertiseService("toggle_calibration", &toggle_calib_callback);

  ros::spin();

  return 0;
}
