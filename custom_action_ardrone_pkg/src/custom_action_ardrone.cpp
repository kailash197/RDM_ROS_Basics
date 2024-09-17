#include "custom_action_ardrone_pkg/CustomAction.h"
#include "custom_action_ardrone_pkg/CustomGoal.h"
#include "ros/forwards.h"
#include "ros/publisher.h"
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>

class CustomActionArdrone {

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<custom_action_ardrone_pkg::CustomAction> as_;
  ros::Publisher droneTakeOffPublisher_;
  ros::Publisher droneLandPublisher_;
  std::string action_name_;
  custom_action_ardrone_pkg::CustomFeedback feedback_;
  std_msgs::Empty takeOff_;
  std_msgs::Empty land_;
  bool success_;
  ros::Rate *rate_;

public:
  CustomActionArdrone(std::string action_name)
      : as_(nh_, action_name,
            boost::bind(&CustomActionArdrone::executeCB, this, _1), false),
        action_name_(action_name) {
    as_.start();
    droneTakeOffPublisher_ =
        nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
    droneLandPublisher_ = nh_.advertise<std_msgs::Empty>("/drone/land", 1000);
    rate_ = new ros::Rate(1);
    success_ = true;
  }
  ~CustomActionArdrone(void) {}
  void executeCB(const custom_action_ardrone_pkg::CustomGoalConstPtr goal) {
    std::string command = goal->goal;
    ROS_INFO("Action Received: %s ", command.c_str());
    rate_ = new ros::Rate(10);

    for (int i = 0; i < 1000; i++) {
      if (command == "TAKEOFF") {
        droneTakeOffPublisher_.publish(takeOff_);
        ros::Duration(3).sleep();
        ROS_INFO("Drone takeoff completed.");

      } else if (command == "LAND") {
        droneLandPublisher_.publish(land_);
        ros::Duration(3).sleep();
        ROS_INFO("Drone landed successfully.");

      } else {
        ROS_INFO("Unknown Command. \n Options: TAKEOFF or LAND");
      }

      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success_ = false;
        break;
      }
      feedback_.feedback = command;
      as_.publishFeedback(feedback_);
      rate_->sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "custom_action_ardrone");

  CustomActionArdrone customActionArdrone("custom_action_ardrone");
  ros::spin();

  return 0;
}