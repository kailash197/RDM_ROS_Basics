#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void counterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  ROS_INFO("%f", msg->pose.pose.position.x);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "topic_subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odom", 1000, counterCallback);

  ros::spin();

  return 0;
}