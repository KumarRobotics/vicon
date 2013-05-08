#include <ros/ros.h>
#include <pthread.h>
#include <map>
#include <set>
#include <boost/thread.hpp>
#include "ViconDriver.h"
#include "ViconCalib.h"
#include "vicon/Subject.h"
#include "vicon/Markers.h"
#include "vicon/SetPose.h"

static ros::NodeHandle *nh = NULL;
static std::string calib_files_dir;
static bool running = false;
static pthread_mutex_t calib_set_mutex = PTHREAD_MUTEX_INITIALIZER;
static std::map<std::string, bool> calib_set;
static std::map<std::string, Eigen::Affine3d, std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string,
    Eigen::Affine3d> > > calib_pose;

static void *loadCalibThread(void *arg)
{
  std::string *subject_name = reinterpret_cast<std::string*>(arg);
  std::string calib_filename = calib_files_dir + "/" + *subject_name + ".yaml";
  Eigen::Affine3d zero_pose;

  if(!ViconCalib::loadZeroPoseFromFile(calib_filename, zero_pose))
  {
    ROS_WARN_STREAM("Error loading calib for " << *subject_name << " from file " <<
                    calib_filename << ", setting calib pose to Identity");
  }

  calib_pose[*subject_name] = zero_pose.inverse();
  pthread_mutex_lock(&calib_set_mutex);
  calib_set[*subject_name] = true;
  pthread_mutex_unlock(&calib_set_mutex);

  delete subject_name;
  return NULL;
}

static bool loadCalib(const std::string subject_name)
{
  if(pthread_mutex_trylock(&calib_set_mutex) != 0)
    return false;

  std::map<std::string, bool>::iterator it = calib_set.find(subject_name);
  if(it == calib_set.end())
  {
    calib_set[subject_name] = false;
    pthread_mutex_unlock(&calib_set_mutex);

    pthread_t thread_id;
    std::string *subject_name_ = new std::string(subject_name);
    pthread_create(&thread_id, NULL, loadCalibThread, subject_name_);
    return false;
  }

  pthread_mutex_unlock(&calib_set_mutex);

  return it->second;
}

static void saveCalibThread(const vicon::SetPose::Request &req)
{
  Eigen::Affine3d zero_pose;
  Eigen::Vector3d t(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  Eigen::Quaterniond q(req.pose.orientation.w,
                       req.pose.orientation.x,
                       req.pose.orientation.y,
                       req.pose.orientation.z);
  zero_pose.setIdentity();
  zero_pose.translate(t);
  zero_pose.rotate(q);

  zero_pose = zero_pose * calib_pose[req.subject_name].inverse();

  std::string calib_filename = calib_files_dir + "/" + req.subject_name + ".yaml";
  if(ViconCalib::saveZeroPoseToFile(zero_pose, calib_filename))
  {
    pthread_mutex_lock(&calib_set_mutex);
    calib_set.erase(req.subject_name);
    pthread_mutex_unlock(&calib_set_mutex);
  }
  else
  {
    ROS_ERROR_STREAM("Error saving zero pose for " << req.subject_name <<
                     ", keeping old calib pose");
  }
}

static bool saveCalib(vicon::SetPose::Request &req,
                      vicon::SetPose::Response &res)
{
  boost::thread save_calib_thread(saveCalibThread, req);
  save_calib_thread.detach();
  return true;
}

static void subject_publish_callback(const ViconDriver::Subject &subject)
{
  if(!running)
    return;

  static std::map<std::string, ros::Publisher> vicon_publishers;
  std::map<std::string, ros::Publisher>::iterator it;

  it = vicon_publishers.find(subject.name);
  if(it == vicon_publishers.end())
  {
    ros::Publisher pub = nh->advertise<vicon::Subject>(subject.name, 10);
    it = vicon_publishers.insert(std::make_pair(subject.name, pub)).first;
  }

  if(loadCalib(subject.name))
  {
    Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();
    current_pose.translate(Eigen::Vector3d(subject.translation));
    current_pose.rotate(Eigen::Quaterniond(subject.rotation));
    current_pose = current_pose * calib_pose[subject.name];
    const Eigen::Vector3d position(current_pose.translation());
    const Eigen::Quaterniond rotation(current_pose.rotation());

    vicon::Subject subject_ros;
    subject_ros.header.seq = subject.frame_number;
    subject_ros.header.stamp = ros::Time(subject.time_usec/1e6);
    subject_ros.header.frame_id = "/vicon";
    subject_ros.name = subject.name;
    subject_ros.occluded = subject.occluded;
    subject_ros.position.x = position.x();
    subject_ros.position.y = position.y();
    subject_ros.position.z = position.z();
    subject_ros.orientation.x = rotation.x();
    subject_ros.orientation.y = rotation.y();
    subject_ros.orientation.z = rotation.z();
    subject_ros.orientation.w = rotation.w();
    subject_ros.markers.resize(subject.markers.size());
    for(size_t i = 0; i < subject_ros.markers.size(); i++)
    {
      subject_ros.markers[i].name = subject.markers[i].name;
      subject_ros.markers[i].subject_name = subject.markers[i].subject_name;
      subject_ros.markers[i].position.x = subject.markers[i].translation[0];
      subject_ros.markers[i].position.y = subject.markers[i].translation[1];
      subject_ros.markers[i].position.z = subject.markers[i].translation[2];
      subject_ros.markers[i].occluded = subject.markers[i].occluded;
    }
    it->second.publish(subject_ros);
  }
}

static void unlabeled_markers_publish_callback(const ViconDriver::Markers &markers)
{
  if(!running)
    return;
  static bool publisher_created = false;
  static ros::Publisher unlabeled_markers_pub;

  if(!publisher_created)
  {
    unlabeled_markers_pub = nh->advertise<vicon::Markers>("unlabeled_markers", 10);
    publisher_created = true;
  }

  vicon::Markers markers_ros;
  markers_ros.header.seq = markers.frame_number;
  markers_ros.header.stamp = ros::Time(markers.time_usec/1e6);
  markers_ros.header.frame_id = "/vicon";
  markers_ros.markers.resize(markers.markers.size());
  for(size_t i = 0; i < markers_ros.markers.size(); i++)
  {
    markers_ros.markers[i].name = markers.markers[i].name;
    markers_ros.markers[i].subject_name = markers.markers[i].subject_name;
    markers_ros.markers[i].position.x = markers.markers[i].translation[0];
    markers_ros.markers[i].position.y = markers.markers[i].translation[1];
    markers_ros.markers[i].position.z = markers.markers[i].translation[2];
    markers_ros.markers[i].occluded = markers.markers[i].occluded;
  }
  unlabeled_markers_pub.publish(markers_ros);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon");

  nh = new ros::NodeHandle("~");

  std::string vicon_server;
  nh->param("vicon_server", vicon_server, std::string("alkaline"));

  nh->param("calib_files_dir", calib_files_dir, std::string("calib"));

  bool enable_unlabeled_markers;
  nh->param("enable_unlabeled_markers", enable_unlabeled_markers, false);

  ros::ServiceServer set_zero_pose_srv =
      nh->advertiseService("set_zero_pose", &saveCalib);

  ViconDriver vd;
  if(!vd.init(vicon_server))
  {
    ROS_ERROR("Error connecting to vicon server");
    nh->shutdown();
    return -1;
  }

  vd.setSubjectPubCallback(subject_publish_callback);
  if(enable_unlabeled_markers)
  {
    vd.setUnlabeledMarkersPubCallback(unlabeled_markers_publish_callback);
    vd.enableUnlabeledMarkerData(true);
  }
  running = vd.start();

  ros::spin();

  running = false;

  vd.shutdown();

  delete nh;

  return 0;
}
