#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>


// define resource heatmap class here
class resource_heatmap{
public:
	resource_heatmap(ros::NodeHandle& nodeHandle);

private:
	ros::NodeHandle nodeHandle_;
	
	// Coordinates of the center of the resource vehicle, received from subscribing topic "resource_location"
	geometry_msgs::Point resource_center; 

	// An Eigen matrix, used to store heatmap
	Eigen::MatrixXf heatmap;

	// Subscribing topic "resource_location"
	// #Callback function is resource_location_Callback()	
	ros::Subscriber resource_location_subs;
	
	// Call back of subscriver resource_location_subs
	void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p);
};