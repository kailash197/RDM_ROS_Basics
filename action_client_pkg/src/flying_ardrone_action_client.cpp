#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>

ros::Publisher cmd_vel_pub;
ros::Publisher droneLandPub;
double current_yaw = 0.0;
int nImage = 0; // Initialization of a global variable

void move_bb8(float side) {
  geometry_msgs::Twist command;
  command.linear.x = 0.5;
  cmd_vel_pub.publish(command);
  ros::Duration(1).sleep();
  command.linear.x = 0;
  cmd_vel_pub.publish(command);
}

void rotate_90() {
  geometry_msgs::Twist command;
  command.angular.z = 0.5;
  cmd_vel_pub.publish(command);
  ros::Duration(0.5).sleep();
  command.angular.z = 0.0;
  cmd_vel_pub.publish(command);
}

// Definition of the done calback. It is called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
            const ardrone_as::ArdroneResultConstPtr &result) {
  ROS_INFO("The Action has been completed");

  std_msgs::Empty trigger;
  droneLandPub.publish(trigger);
  ros::Duration(2.0).sleep();
  ROS_INFO("Drone: Landed.");
  ros::shutdown();
}

// Definition of the active callback. It is called once when the goal becomes
// active
void activeCb() { ROS_INFO("Goal just went active"); }

// Definition of the feedback callback. This will be called when feedback is
// received from the action server. It just // prints a message indicating a new
// message has been received
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr &feedback) {
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_action_client");
  ros::NodeHandle nh;

  // Publishers and subscribers
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher droneTakeOffPub =
      nh.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
  droneLandPub = nh.advertise<std_msgs::Empty>("/drone/land", 1000);

  std_msgs::Empty trigger;

  // Create the connection to the action server
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client(
      "ardrone_action_server", true);
  client.waitForServer(); // Waits until the action server is up and running

  droneTakeOffPub.publish(trigger);
  ROS_INFO("Drone: Take off started.");
  ros::Duration(2.0).sleep();

  ardrone_as::ArdroneGoal goal; // Creates a goal to send to the action server
  goal.nseconds = 20;           // Indicates, take pictures along 10 seconds
  ROS_INFO("Action: Goal sent. Action started.");
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  // sends the goal to the action server, specifying which //
  // functions to call when the goal completes, when the //
  // goal becames active, and when feedback is received

  ros::Rate loop_rate(2);
  actionlib::SimpleClientGoalState state_result = client.getState();
  ROS_INFO("[State Result]: %s", state_result.toString().c_str());

  while (state_result == actionlib::SimpleClientGoalState::ACTIVE ||
         state_result == actionlib::SimpleClientGoalState::PENDING) {

    if (nImage % 2 == 0) {
      move_bb8(1);
      ROS_INFO("[Move Result]: %s", state_result.toString().c_str());
    } else {
      rotate_90();
      ROS_INFO("[Rotate Result]: %s", state_result.toString().c_str());
    }
    state_result = client.getState();
  }

  ros::spin();
  return 0;
}