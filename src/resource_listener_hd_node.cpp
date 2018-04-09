#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#include "multirov/resource_listener_hd.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "resource_listener_node");

  ros::NodeHandle node;
  
  resource_listener_hd resource_listener_hd_1(node); 
  resource_listener_hd_1.resource_location_publ = node.advertise<geometry_msgs::Point>("resource_location", 10); 
  resource_listener_hd_1.resource_location_subs = node.subscribe("resource_location_from_keyboard", 1000, &resource_listener_hd::subsCallback, &resource_listener_hd_1); 

  
  ros::spin();

  return 0;
}
