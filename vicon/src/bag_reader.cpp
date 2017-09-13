#include <fstream>
#include <stdio.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <vicon/Markers.h>

std::string nav_log_name_ = "text.yaml";

void read_bag(std::string file_name, std::string topic) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  vicon::Markers ms;
  bool set_init_time =false;
  double init_time;
  std::ofstream fout(nav_log_name_);
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    ROS_INFO_ONCE("Get msg!");
    vicon::Markers::ConstPtr ms_ptr = m.instantiate<vicon::Markers>();
    if (ms_ptr != NULL) {
      if(!set_init_time) {
        set_init_time = true;
        init_time = ms_ptr->header.stamp.toSec();
      }
      double t = ms_ptr->header.stamp.toSec() - init_time;
      std::string pts = std::to_string(t) + ", ";
      for(int i = 0; i < ms_ptr->markers.size(); i++) {
        std::string pt;
        pt += std::to_string(ms_ptr->markers[i].position.x) + ", ";
        pt += std::to_string(ms_ptr->markers[i].position.y) + ", ";
        if(i != ms_ptr->markers.size() - 1)
          pt += std::to_string(ms_ptr->markers[i].position.z) + ", ";
        else
          pt += std::to_string(ms_ptr->markers[i].position.z);
        pts += pt;
      }
      pts += "\n";

      fout << pts.c_str();
    }
  }
  bag.close();
  ROS_WARN("Wrote!");
}


int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");


  std::string file_name, topic_name;
  nh.param("file", file_name, std::string("test.bag"));
  nh.param("topic", topic_name, std::string("test"));
  nav_log_name_ = file_name.substr(0, file_name.length() - 4) + ".yaml";


  ROS_INFO("File: %s", file_name.c_str());
  ROS_INFO("Topic: %s", topic_name.c_str());
  read_bag(file_name, topic_name);
  ros::spin();


  return 0;
}


