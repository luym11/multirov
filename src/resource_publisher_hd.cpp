#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "resouce_publisher__keyboard_node");

  ros::NodeHandle n;

  ros::Publisher resource_location_keyboard_pub = n.advertise<geometry_msgs::Point>("resource_location_from_keyboard", 1000);

  ros::spin(); 

  return 0;
}