#include "vicon_odom/vicon_odom_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vicon_odom_node");
  ros::NodeHandle nh, pnh("~");

  try {
    vicon_odom::ViconOdomNode vicon_odom_node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
