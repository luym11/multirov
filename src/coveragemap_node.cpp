#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "multirov/coveragemap.hpp"

#define HEIGHT 200
#define LENGTH 200

// init a covermap with zeros
coveragemap c; 

void agent_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	std::vector<int> vec(2,1); 
	vec[0] = (int)round(p->x); vec[1] = (int)round(p->y); 
	c.agents[0][0] = vec[0]; c.agents[0][1] = vec[1]; 

	std::cout << c.agents[0][0] << " " << c.agents[0][1] << " " << c.agents.size() << " " << c.col << std::endl; 

	c.set_coveragemap(); 
	for(int i = 5; i >= -5; i--){
		for(int j = -5; j <= 5; j++){
			if(vec[0]+i >= 0 &  vec[1]+j >= 0 & vec[0]+i < LENGTH & vec[1]+j < HEIGHT){
				printf("%f ", c.covermap2(vec[0]+i, vec[1]+j)); 
			}
		} 
		printf("\n");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coveragemap_node");
	ros::NodeHandle nh;

	std::string ns; 
		if(argc != 2){
	ROS_WARN("Warning! Didn't pass correct namespace format");
	}else{
		ns = argv[1];
	}
	ns.append("/agent_location"); 

	
	// just init agent list, take a space
	std::vector<int> vec1(2,1); 
	c.agents.push_back(vec1); 

	// init a subs for agent locations
	ros::Subscriber agent_location_subs = nh.subscribe( ns, 10, agent_location_Callback); 

	ros::spin();
	return 0;
}
