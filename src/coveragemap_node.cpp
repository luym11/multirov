#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "multirov/coveragemap.hpp"

#define HEIGHT 20
#define LENGTH 20

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coveragemap_node");
  ros::NodeHandle nh;
  coveragemap c; 
  std::vector<int> vec(2,1); 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,1]
	vec[1] = 2;  
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,2]
	vec[0] = 6; vec[1] = 6; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [6,6]
	vec[0] = 2; vec[1] = 2; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [2,2]
	vec[0] = 1; vec[1] = 3; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,3]
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	printf("\n");
	c.agents.push_back(vec); // [1,3] again
	c.set_coveragemap(); 
	for(int i = c.col - 1; i >= 0; i--){
		for(int j = 0; j <= c.row - 1; j++){
			printf("%f ", c.covermap2(i, j)); 
		} 
		printf("\n");
	}
  ros::spin();
  return 0;
}
