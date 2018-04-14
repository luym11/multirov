#include <multirov/location_bridge.hpp>

int main(int argc, char** argv){
	ros::init(argc, argv, "resource_listener_node");

	ros::NodeHandle node;

	location_bridge location_bridge_1(node); 
	location_bridge_1.rov1_location_publ = node.advertise<geometry_msgs::Point>("/rexrov1/agent_location", 10); 
	location_bridge_1.rov2_location_publ = node.advertise<geometry_msgs::Point>("/rexrov2/agent_location", 10); 
	location_bridge_1.rov3_location_publ = node.advertise<geometry_msgs::Point>("/rexrov3/agent_location", 10); 
	location_bridge_1.origin_location_publ = node.advertise<geometry_msgs::Point>("/origin_location", 10); 
	location_bridge_1.x_location_publ = node.advertise<geometry_msgs::Point>("/x_location", 10); 
	
	location_bridge_1.markers_location_subs = node.subscribe("tag_detections", 1000, &location_bridge::subsCallback, &location_bridge_1); 

	  
	ros::spin();
	return 0; 
}