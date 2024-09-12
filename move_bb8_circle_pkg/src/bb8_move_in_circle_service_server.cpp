#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
// Import the service message header file generated from the Empty.srv message

ros::Publisher cmd_vel_pub;

// We define the callback function of the service
bool my_callback(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res) {
  // res.some_variable = req.some_variable + req.other_variable;
  ROS_INFO(
      "Request received to move bb8 in circle."); // We print an string whenever
                                                  // the Service gets called

  geometry_msgs::Twist command;
  auto &vel = command.linear.x;
  auto &dir = command.angular.z;

  vel = 0.5;
  dir = 0.5;
  cmd_vel_pub.publish(command);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_bb8_circle_server_node");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::ServiceServer my_service = nh.advertiseService(
      "/move_bb8_in_circle",
      my_callback); // create the Service called // my_service
                    // with the defined // callback
  ros::spin();      // maintain the service open.

  return 0;
}