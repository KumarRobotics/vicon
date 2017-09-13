#include <ros/ros.h>
#include <vicon/Markers.h>
#include <sensor_msgs/PointCloud.h>


ros::Publisher cloud_pub;

void markersCallback(const vicon::Markers::ConstPtr& msg) {
  sensor_msgs::PointCloud cloud;
  cloud.header = msg->header;

  for(int i = 0; i < msg->markers.size(); i++) {
    geometry_msgs::Point32 pt;
    pt.x = msg->markers[i].position.x;
    pt.y = msg->markers[i].position.y;
    pt.z = msg->markers[i].position.z;
    cloud.points.push_back(pt);
  }

  ROS_INFO_THROTTLE(1, "Num of markers: %zu", msg->markers.size());

  cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "markers");

  ros::NodeHandle nh("~");

  ros::Subscriber markers_sub = nh.subscribe("markers", 1, markersCallback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);


  ros::spin();
  return 0;
}
