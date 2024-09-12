#include "move_bb8_square_pkg/BB8CustomServiceMessage.h"
#include "ros/ros.h"
#include <ros/package.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_bb8_square_client_node");
  ros::NodeHandle nh;

  ros::service::waitForService("/move_bb8_in_square_custom");
  ros::ServiceClient bb8_service_client =
      nh.serviceClient<move_bb8_square_pkg::BB8CustomServiceMessage>(
          "/move_bb8_in_square_custom");
  move_bb8_square_pkg::BB8CustomServiceMessage srv;
  srv.request.side = 1;
  srv.request.repetitions = 1;
  for (int i = 0; i < 3; i++) {

    if (bb8_service_client.call(srv)) {
      ROS_INFO("Service call to /move_bb8_in_square_custom successful.");
    } else {
      ROS_ERROR("Failed to call service /move_bb8_in_square_custom");
      return 1;
    }
    srv.request.side += 0.5;
  }

  return 0;
}