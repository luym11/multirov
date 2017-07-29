#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

geometry_msgs::Point resource_center; 

void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	resource_center.x = p->x;
	resource_center.y = p->y;
	resource_center.z = p->z; 
	ROS_INFO("center x is %f", resource_center.x); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "resource_heatmap");
  ros::NodeHandle nh;
  ros::Subscriber resource_location_subs;
  resource_location_subs = nh.subscribe( "resource_location", 10, resource_location_Callback); 
  
  ros::spin();
  return 0;
}
