#ifndef LOCATION_BRIDGE
#define LOCATION_BRIDGE

#include <apriltags2_ros/common_functions.h>

class location_bridge{
public: 
	ros::NodeHandle nodeHandle_;
	ros::NodeHandle *nh_1; 

	geometry_msgs::Point origin_location_point; 
	geometry_msgs::Point x_location_point;
	geometry_msgs::Point rov1_location_point;
	geometry_msgs::Point rov2_location_point;
	geometry_msgs::Point rov3_location_point;

	geometry_msgs::Point origin_location_point_r; 
	geometry_msgs::Point x_location_point_r;
	geometry_msgs::Point rov1_location_point_r;
	geometry_msgs::Point rov2_location_point_r;
	geometry_msgs::Point rov3_location_point_r;

  	location_bridge(ros::NodeHandle& nodeHandle);

  	ros::Subscriber markers_location_subs; 
  	void subsCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& p);

  	ros::Publisher origin_location_publ;
	ros::Publisher x_location_publ;
	ros::Publisher rov1_location_publ;
	ros::Publisher rov2_location_publ;
	ros::Publisher rov3_location_publ;
};

#endif