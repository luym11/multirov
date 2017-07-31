#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include "multirov/coveragemap.hpp"

#define HEIGHT 200
#define LENGTH 200

// init a covermap with zeros
coveragemap c;

void agent_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	std::vector<int> vec(2,1); 
	vec[0] = (int)round(p->x); vec[1] = (int)round(p->y); 
	int rovnumMinus1 = (int)round(p->z)-1;
	c.agents[rovnumMinus1][0] = vec[0]; c.agents[rovnumMinus1][1] = vec[1]; 

	std::cout << c.agents[rovnumMinus1][0] << " " << c.agents[rovnumMinus1][1] << " " << c.agents.size() << std::endl; 

	c.set_coveragemap(); 
	for(int j = 5; j >= -5; j--){
		for(int i = -5; i <= 5; i++){
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

	std::vector< std::string > namespaces;  // store namespaces string: /rexrov1 /rexrov2....
	std::vector< int > rov_nums;  // store rov numbers: 1 2.....
	/*
	if(argc < 2){
	ROS_WARN("Warning! Didn't pass correct namespace format");
	}else{
		for(int i = 0; i < argc-1; i++){
			namespaces.push_back(argv[i+1]); 
			namespaces[i].append("/agent_location"); 
			int num; 
			sscanf(argv[i+1], "/rexrov%d", &num); 
			rov_nums.push_back(num); 
		}
	}
	*/
	nh.getParam("/coveragemap_node/rovs", namespaces); 
	for(int i = 0; i < namespaces.size(); i++){
		int num; 
		char* cstr = &namespaces[i][0u];
		sscanf(cstr, "/rexrov%d", &num); // must be char[], not string
		namespaces[i].append("/agent_location"); 
		rov_nums.push_back((int)num); 
		// std::cout << namespaces[i] << std::endl; 
	}
	
	//for(int i = 0; i < rov_nums.size(); i++){
	//	printf("%d ", rov_nums[i]); 
	//}
	//printf("\n");

	// just init agent list, take a space
	std::vector<int> vec1(2,1); 
	for(int i = 0; i < namespaces.size(); i++){
		c.agents.push_back(vec1);
	} 

	// vector of subs does not work
	ros::Subscriber agent_location_subs_1 = nh.subscribe( namespaces[0], 10, agent_location_Callback);
	ros::Subscriber agent_location_subs_2 = nh.subscribe( namespaces[1], 10, agent_location_Callback);

	ros::spin();
	return 0;
}
