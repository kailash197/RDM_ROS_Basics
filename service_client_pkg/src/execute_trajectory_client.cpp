#include "iri_wam_reproduce_trajectory/ExecTraj.h"
#include "ros/ros.h"
#include <ros/package.h>
/*
user:~$ rosservice list | grep execute/execute_trajectory
user:~$ rosservice info /execute_trajectory
Node: /iri_wam_reproduce_trajectory
URI: rosrpc://1_xterm:46625
Type: iri_wam_reproduce_trajectory/ExecTraj
Args: file
user:~$ rossrv show iri_wam_reproduce_trajectory/ExecTraj
string file
---

*/
// #include "trajectory_by_name_srv/TrajByName.h"
// Import the service message used by the service /trajectory_by_name

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "execute_trajectory_service_client"); // Initialise a ROS node with
                                                  // the name service_client
  ros::NodeHandle nh;

  // Create the connection to the service /trajectory_by_name
  ros::service::waitForService(
      "/execute_trajectory"); // wait for service to be running
  ros::ServiceClient execute_traj_client =
      nh.serviceClient<iri_wam_reproduce_trajectory::ExecTraj>(
          "/execute_trajectory");
  iri_wam_reproduce_trajectory::ExecTraj
      srv; // Create an object of type ExecTraj
  // This ros::package::getPath works in the same way as $(find name_of_package)
  // in the launch files.
  srv.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") +
                     "/config/get_food.txt";

  if (execute_traj_client.call(srv)) // Send through the connection the name of
                                     // the trajectory to execute
  {
    ROS_INFO("Service succesfully completed.");
  } else {
    ROS_ERROR("Failed to call service /execute_trajectory");
    return 1;
  }

  return 0;
}