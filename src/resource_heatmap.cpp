#include <ros/ros.h>
#include "multirov/resource_heatmap.hpp"

resource_heatmap::resource_heatmap(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle){
	heatmap = Eigen::MatrixXi::Zero(200, 200);
	resource_location_subs = nodeHandle.subscribe( "/resource_location", 5, &resource_heatmap::resource_location_Callback, this); 
}

void resource_heatmap::heatmap_update(int x, int y){
	heatmap = Eigen::MatrixXi::Zero(200, 200);
	heatmap.block(x-3, y-3, 7, 7) = Eigen::MatrixXi::Constant(7, 7, 4); 
	heatmap.block(x-2, y-2, 5, 5) = Eigen::MatrixXi::Constant(5, 5, 6); 
	heatmap.block(x-1, y-1, 3, 3) = Eigen::MatrixXi::Constant(3, 3, 8); 
	heatmap(x, y) = 10;

}

void resource_heatmap::resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	resource_center.x = (int)round(p->x);
	resource_center.y = (int)round(p->y);
	resource_center.z = (int)round(p->z); 
	heatmap_update((int)round(p->x), (int)round(p->y)); 
	ROS_INFO("center x is %f, heatmap values are %d %d %d %d", resource_center.x, heatmap((int)round(p->x), (int)round(p->y)), heatmap((int)round(p->x)+1, (int)round(p->y)+1), heatmap((int)round(p->x)+2, (int)round(p->y)+2), heatmap((int)round(p->x)+3, (int)round(p->y)+3)); 
	Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	std::cout << heatmap.block((int)round(p->x)-4, (int)round(p->y)-4,9,9).format(HeavyFmt) << std::endl;
}

 

 