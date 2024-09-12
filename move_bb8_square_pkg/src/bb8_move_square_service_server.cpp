#include "move_bb8_square_pkg/BB8CustomServiceMessage.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

ros::Publisher cmd_vel_pub;
ros::Subscriber odom_sub;
double current_yaw = 0.0;

double normalize_angle(double angle) {
  while (angle >= 2 * M_PI)
    angle -= 2 * M_PI;
  while (angle < 0)
    angle += 2 * M_PI;
  return angle;
}

void move_bb8(float side) {
  geometry_msgs::Twist command;
  float linear_speed = 0.6; // m/s
  int frequency = 10;
  ros::Rate rate(frequency);
  int ticks = (side * frequency) / linear_speed;

  for (int i = 0; i < ticks; i++) {
    command.linear.x = linear_speed;
    cmd_vel_pub.publish(command);
    rate.sleep();
  }
  command.linear.x = 0;
  cmd_vel_pub.publish(command);
}
void rotate_90() {
  ros::Rate rate(10); // 10 Hz
  geometry_msgs::Twist command;
  float angular_speed = 0.17;
  double change_yaw = M_PI_2;
  double start_yaw = current_yaw;
  double stop_yaw = normalize_angle(start_yaw + change_yaw);

  while (fabs(current_yaw - stop_yaw) > 0.01) {
    command.angular.z = -angular_speed;
    cmd_vel_pub.publish(command);

    // Ensure callbacks are processed
    ros::spinOnce();

    rate.sleep();
    // ROS_INFO("Current yaw: %f, Stop yaw: %f", current_yaw, stop_yaw);
  }

  command.angular.z = 0.0;
  cmd_vel_pub.publish(command);
}

bool service_callback(
    move_bb8_square_pkg::BB8CustomServiceMessage::Request &req,
    move_bb8_square_pkg::BB8CustomServiceMessage::Response &res) {
  ROS_INFO("Move request received: Side = %f for %d repititions.", req.side,
           req.repetitions);
  float side = req.side > 1 ? req.side : 1;

  for (int count = 0; count < req.repetitions; count++) {
    move_bb8(side);
    rotate_90();
    move_bb8(side);
    rotate_90();
    move_bb8(side);
    rotate_90();
    move_bb8(side);
    rotate_90();
  }

  ROS_INFO("Request completed.");
  res.success = true;

  return true;
}
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert the quaternion to roll, pitch, and yaw (RPY)
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_yaw = normalize_angle(yaw);
  //   ROS_INFO("ODOM current yaw %f ", current_yaw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_server");
  ros::NodeHandle nh;
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  odom_sub = nh.subscribe("/odom", 1000, odom_callback);
  ros::ServiceServer my_service =
      nh.advertiseService("/move_bb8_in_square_custom", service_callback);

  ros::spin();

  return 0;
}