#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <math.h>


// define resource heatmap class here
class resource_heatmap{
public:
	resource_heatmap(ros::NodeHandle& nodeHandle);

private:
	ros::NodeHandle nodeHandle_;

	// Coordinates of the center of the resource vehicle, received from subscribing topic "resource_location"
	geometry_msgs::Point resource_center; 

	// An Eigen matrix, used to store heatmap
	Eigen::MatrixXi heatmap;

	// Subscribing topic "resource_location"
	// #Callback function is resource_location_Callback()	
	ros::Subscriber resource_location_subs;
	
	// Call back of subscriver resource_location_subs
	void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p);

	// Update heatmap while subscriber resource_location_subs receives anything (new):: now 2D version
	void heatmap_update(int x, int y);
};