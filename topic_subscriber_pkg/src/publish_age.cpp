#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <topic_subscriber_pkg/Age.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "publish_age_node");
  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<topic_subscriber_pkg::Age>("robot_age", 200);
  ros::Rate loop_rate(2);
  topic_subscriber_pkg::Age msg;
  msg.years = 5;
  msg.months = 6;
  msg.days = 7;
  while (ros::ok()) {
    msg.days += 1;
    if (msg.days > 0 && msg.days % 30 == 0) {
      msg.days = 0;
      msg.months += 1;
    }
    if (msg.months > 0 && msg.months % 12 == 0) {
      msg.months = 0;
      msg.years += 1;
    }

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}