#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <ros/package.h>
/*
user:~$ rosservice info /move_bb8_in_circle
Node: /move_bb8_circle_server_node
URI: rosrpc://1_xterm:49493
Type: std_srvs/Empty
Args:
user:~$ rossrv show std_srvs/Empty
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_bb8_circle_client_node");
  ros::NodeHandle nh;

  // Create the connection to the service /move_bb8_in_circle
  ros::service::waitForService("/move_bb8_in_circle");
  ros::ServiceClient bb8_service_client =
      nh.serviceClient<std_srvs::Empty>("/move_bb8_in_circle");
  std_srvs::Empty srv; // Create an object of type empty

  if (bb8_service_client.call(srv)) // Send through the connection the name of
                                    // the trajectory to execute
  {
    ROS_INFO("Service succesfully called. Moving bb8 in a circle.");
  } else {
    ROS_ERROR("Failed to call service /move_bb8_in_circle");
    return 1;
  }

  return 0;
}