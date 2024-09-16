#include "actionlib/TestGoal.h"
#include "ros/duration.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <actionlib/TestAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

class FlyDroneSquareActionServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib::TestAction> as_;
  ros::Publisher cmd_vel_pub;
  ros::Publisher droneLandPub;
  ros::Publisher droneTakeOffPub;

  std::string action_name_;
  // create messages that are used to publish feedback and result
  actionlib::TestFeedback feedback_;
  actionlib::TestResult result_;
  geometry_msgs::Twist command_vel;
  std_msgs::Empty takeOff;
  std_msgs::Empty land;
  bool success_;
  ros::Rate *rate_;

public:
  FlyDroneSquareActionServer(std::string name)
      : as_(nh_, name,
            boost::bind(&FlyDroneSquareActionServer::executeCB, this, _1),
            false),
        action_name_(name) {
    as_.start();
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    droneTakeOffPub = nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
    droneLandPub = nh_.advertise<std_msgs::Empty>("/drone/land", 1000);
    rate_ = new ros::Rate(1);
    success_ = true;
    command_vel.linear.x = 0.0;
    command_vel.linear.y = 0.0;
    command_vel.angular.z = 0.0;
  }
  void rotate90(void);
  void move(int duration);
  void stop(void);

  ~FlyDroneSquareActionServer(void) {}

  void executeCB(const actionlib::TestGoalConstPtr &goal) {
    ros::Time start_time = ros::Time::now();
    int size_of_square = goal->goal;
    ROS_INFO("Goal Received: Fly Drone in square of %d m.", size_of_square);

    droneTakeOffPub.publish(takeOff);
    ros::Duration(3).sleep();
    ROS_INFO("Drone Takeoff completed.");

    for (int side = 1; side <= 4; side++) {
      move(size_of_square);
      rotate90();

      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success_ = false;
        break;
      }

      feedback_.feedback = side;
      as_.publishFeedback(feedback_);
      rate_->sleep();
    }
    stop();
    ros::Duration total_time = ros::Time::now() - start_time;

    if (success_) {
      result_.result = total_time.toSec();
      ROS_INFO("%s: Succeeded in %d seconds.", action_name_.c_str(),
               result_.result);
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
    droneLandPub.publish(land);
    ros::Duration(3).sleep();
    ROS_INFO("Drone landed successfully.");
  }
};

void FlyDroneSquareActionServer::rotate90() {
  ROS_DEBUG("Turning drone.");

  command_vel.linear.x = 0;
  command_vel.linear.y = 0;
  command_vel.angular.z = 1.5708;
  cmd_vel_pub.publish(command_vel);
  ros::Duration(1.1).sleep();
  stop();
}

void FlyDroneSquareActionServer::move(int duration) {
  ROS_DEBUG("Moving drone.");
  command_vel.linear.x = 1;
  command_vel.angular.z = 0;
  for (int i = 0; i < duration; i++) {
    cmd_vel_pub.publish(command_vel);
    rate_->sleep();
  }
  stop();
}

void FlyDroneSquareActionServer::stop() {
  ROS_DEBUG("Stopping drone.");
  command_vel.linear.x = 0;
  command_vel.linear.y = 0;
  command_vel.angular.z = 0;
  cmd_vel_pub.publish(command_vel);
  ros::Duration(0.2).sleep();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fly_drone_action");

  FlyDroneSquareActionServer flyDroneSquareActionServer("fly_drone_action");
  ros::spin();

  return 0;
}

/*
user:~/catkin_ws/src$ cat $(find /opt/ros/noetic/ -name Test.action)
int32 goal
---
int32 result
---
int32 feedback
  */