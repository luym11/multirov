#ifndef ALGO_NODE
#define ALGO_NODE

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include "multirov/explore_algo.hpp"
#include "multirov/coveragemap.hpp"
#include <string.h>

class explore_algo_node{
public: 

	ros::NodeHandle nodeHandle_;
	ros::NodeHandle *nh_1; 
	// constructor
	explore_algo_node(ros::NodeHandle& nodeHandle); 

	explore_algo ex; 
	coveragemap c; 
	int done_flag;
	std_msgs::Int8 d; 

	int rovNum; 

	Eigen::MatrixXi heatmap_update(int x, int y); 

	void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p); 

	void agent_location_Callback(const geometry_msgs::Point::ConstPtr& p); 

	// subs and publs
	ros::Publisher go_direction_publ;
	ros::Subscriber resource_location_subs;
	
	ros::Subscriber rexrov1_location_subs; 
	ros::Subscriber rexrov2_location_subs;
	ros::Subscriber rexrov3_location_subs; 

	ros::Subscriber current_angle_subs; 
	void current_angle_Callback(const geometry_msgs::TwistStamped::ConstPtr& a); 
};

#endif