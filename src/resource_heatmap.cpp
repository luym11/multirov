#include <ros/ros.h>
#include "multirov/resource_heatmap.hpp"

resource_heatmap::resource_heatmap(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle){
	heatmap = Eigen::MatrixXf::Zero(20, 20);
	resource_location_subs = nodeHandle.subscribe( "/resource_location", 5, &resource_heatmap::resource_location_Callback, this); 
}

//void: resource_heatmap::heatmap_update(){
//	heatmap()
//}

void resource_heatmap::resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	resource_center.x = p->x;
	resource_center.y = p->y;
	resource_center.z = p->z; 
	//heatmap_update(); 
	ROS_INFO("center x is %f", resource_center.x); 
}

 

 