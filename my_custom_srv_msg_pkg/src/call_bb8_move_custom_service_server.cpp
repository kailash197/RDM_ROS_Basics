#include "my_custom_srv_msg_pkg/MyCustomServiceMessage.h"
#include "ros/ros.h"
#include <ros/package.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "custom_service_client");
  ros::NodeHandle nh;

  // Create the connection to the service /move_bb8_in_circle
  ros::service::waitForService("/move_bb8_in_circle_custom");
  ros::ServiceClient bb8_service_client =
      nh.serviceClient<my_custom_srv_msg_pkg::MyCustomServiceMessage>(
          "/move_bb8_in_circle_custom");
  my_custom_srv_msg_pkg::MyCustomServiceMessage srv;
  srv.request.duration = 5;

  if (bb8_service_client.call(srv)) // Send through the connection the name of
                                    // the trajectory to execute
  {
    ROS_INFO("Service requested: move bb8 for %d seconds.",
             srv.request.duration);
  } else {
    ROS_ERROR("Failed to call service /move_bb8_in_circle_custom");
    return 1;
  }

  return 0;
}