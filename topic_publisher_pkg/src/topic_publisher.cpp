#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "topic_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate loop_rate(2);

  geometry_msgs::Twist command1, command2;
  command1.linear.x = 0.5;
  command2.linear.x = -0.5;

  while (ros::ok()) {
    pub.publish(command1);
    pub.publish(command2);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}