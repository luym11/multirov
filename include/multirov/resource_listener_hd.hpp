#ifndef RES_LISTENER_HD_NODE
#define RES_LISTENER_HD_NODE

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <math.h>

class resource_listener_hd{
public: 
	ros::NodeHandle nodeHandle_;
	ros::NodeHandle *nh_1; 

	geometry_msgs::Point resource_location_point; 

  	resource_listener_hd(ros::NodeHandle& nodeHandle); 
  	bool eequal(geometry_msgs::Point a, geometry_msgs::Point b); 
  	void subsCallback(const geometry_msgs::Point::ConstPtr& p);

  	ros::Publisher resource_location_publ;
  	ros::Subscriber resource_location_subs; 
};

#endif