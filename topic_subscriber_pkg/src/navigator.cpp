#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

ros::Publisher cmd_vel_pub;

const float calc_distance(const std::vector<float> &r, int index,
                          int sample = 10) {
  float distance = 0.0;
  int start = index - sample / 2;
  int end = index + sample / 2;

  if (start < 0)
    start = 0;
  if (end > r.size())
    end = r.size();

  for (int i = start; i < end; ++i) {
    distance += r[i];
  }

  return distance / sample;
}

void callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  int range_size = msg->ranges.size();
  float front_distance = calc_distance(msg->ranges, range_size / 2);
  float left_distance = calc_distance(msg->ranges, range_size - 20, 20);
  float right_distance = calc_distance(msg->ranges, 0, 20);

  ROS_INFO("Distances Left: %f, Front: %f, Right: %f", left_distance,
           front_distance, right_distance);

  geometry_msgs::Twist command;
  auto &vel = command.linear.x;
  auto &dir = command.angular.z;

  vel = 0.0;
  dir = 0.0;

  if (front_distance < 1.0) {
    if (left_distance > 1.0) {
      dir += 0.5;
    } else if (right_distance > 1.0) {
      dir -= 0.5;
    } else {
      dir += 0.5;
    }
  } else {
    vel = 0.2;

    if (right_distance < 1.0) {
      dir += 0.3;
    } else if (left_distance < 1.0) {
      dir -= 0.3;
    }
  }

  cmd_vel_pub.publish(command);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigator_node");
  ros::NodeHandle nh;

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sub = nh.subscribe("/kobuki/laser/scan", 1000, callback);
  ros::spin();

  return 0;
}
