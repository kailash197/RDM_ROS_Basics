#include "my_custom_srv_msg_pkg/MyCustomServiceMessage.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

ros::Publisher cmd_vel_pub;

bool my_callback(my_custom_srv_msg_pkg::MyCustomServiceMessage::Request &req,
                 my_custom_srv_msg_pkg::MyCustomServiceMessage::Response &res) {
  ROS_INFO("Request received to move bb8 in circle for %d seconds.",
           req.duration);

  geometry_msgs::Twist command;
  command.linear.x = 0.5;
  command.angular.z = 0.5;

  int frequency = 10;

  ros::Rate rate(frequency);
  int ticks = req.duration * frequency;

  for (int i = 0; i < ticks; i++) {
    cmd_vel_pub.publish(command);
    rate.sleep();
  }
  ROS_INFO("Request completed.");
  command.linear.x = 0;
  command.angular.z = 0;
  cmd_vel_pub.publish(command);
  res.message = "SUCCESSFUL OPERATION!!!";

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_server");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::ServiceServer my_service =
      nh.advertiseService("/move_bb8_in_circle_custom", my_callback);

  ros::spin();

  return 0;
}