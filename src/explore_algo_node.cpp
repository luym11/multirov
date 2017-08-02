#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "multirov/explore_algo.hpp"
#include "multirov/coveragemap.hpp"
#include <string.h>

explore_algo ex; 
coveragemap c(200, 200); 
int done_flag = 0;
ind d; 

Eigen::MatrixXi heatmap_update(int x, int y){
	Eigen::MatrixXi _heatmap = Eigen::MatrixXi::Zero(200, 200);
	_heatmap.block(x-3, y-3, 7, 7) = Eigen::MatrixXi::Constant(7, 7, 4); 
	_heatmap.block(x-2, y-2, 5, 5) = Eigen::MatrixXi::Constant(5, 5, 6); 
	_heatmap.block(x-1, y-1, 3, 3) = Eigen::MatrixXi::Constant(3, 3, 8); 
	_heatmap(x, y) = 10;
	return _heatmap;
}

void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	geometry_msgs::Point resource_center; 
	resource_center.x = (int)round(p->x);
	resource_center.y = (int)round(p->y);
	resource_center.z = (int)round(p->z); 
	ex.heatmap  =heatmap_update((int)round(p->x), (int)round(p->y)); 
	// ROS_INFO("center x is %f, heatmap values are %d %d %d %d", resource_center.x, heatmap((int)round(p->x), (int)round(p->y)), heatmap((int)round(p->x)+1, (int)round(p->y)+1), heatmap((int)round(p->x)+2, (int)round(p->y)+2), heatmap((int)round(p->x)+3, (int)round(p->y)+3)); 
	// Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	// std::cout << _heatmap.block((int)round(p->x)-4, (int)round(p->y)-4,9,9).format(HeavyFmt) << std::endl;
	
	ex.remap_heatmap(); 
	ex.remap_coordinates(); 
	ex.calculate_covermap(); 
	ex.move_to_see_the_scores();
	
	for(int i = 0 ; i < ex.scores_at_different_directions.size(); i++){
		std::cout<< ex.scores_at_different_directions[i] << " "; 
	}
	std::cout << std::endl;

	ex.find_margin_utilities();
	printf("%d\n", ex.find_new_direction());
	
}

void agent_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	std::vector<int> vec(2,1); 
	vec[0] = (int)round(p->x); vec[1] = (int)round(p->y); 
	int rovnumMinus1 = (int)round(p->z)-1;
	c.agents[rovnumMinus1][0] = vec[0]; c.agents[rovnumMinus1][1] = vec[1]; 

	std::cout << c.agents[rovnumMinus1][0] << " " << c.agents[rovnumMinus1][1] << " " << c.agents.size() << std::endl; 

	c.set_coveragemap(); 
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			if(vec[0]+i >= 0 &  vec[1]+j >= 0 & vec[0]+i < c.col & vec[1]+j < c.row){
				printf("%f ", c.covermap2(vec[0]+i, vec[1]+j)); 
			}
		} 
		printf("\n");
	}

	ex.my_location = vec; 
	if(done_flag == 1){
		ex.remap_heatmap(); 
		ex.remap_coordinates(); 
		ex.calculate_covermap(); 
		ex.move_to_see_the_scores();
		
		for(int i = 0 ; i < ex.scores_at_different_directions.size(); i++){
			std::cout<< ex.scores_at_different_directions[i] << " "; 
		}
		std::cout << std::endl;

		ex.find_margin_utilities();
		printf("%d\n", ex.find_new_direction());
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "explore_algo_node");
	ros::NodeHandle nh;
	ex.heatmap = Eigen::MatrixXi::Zero(200, 200);
	ros::Subscriber resource_location_subs = nh.subscribe("/resource_location", 5, resource_location_Callback);

 	if(argc != 2){
	ROS_WARN("Warning! Didn't pass correct namespace format");
	}else{
			strcat(argv[1], "/agent_location"); 
		
	}
	std::cout << argv[1] << std::endl; 

	//init
	std::vector<int> vec1(2,1); 
	c.agents.push_back(vec1);

 	ros::Subscriber my_location_subs = nh.subscribe(argv[1], 10, agent_location_Callback);


	ex.my_location.push_back(22); ex.my_location.push_back(53); 
	std::vector<int> a2; a2.push_back(1); a2.push_back(1); 
	std::vector<int> a3(2,2); 
	ex.agent_locations.push_back(ex.my_location); ex.agent_locations.push_back(a2); ex.agent_locations.push_back(a3);

	ex.remap_heatmap(); 
	ex.remap_coordinates(); 
	ex.calculate_covermap(); 
	ex.move_to_see_the_scores();
	
	for(int i = 0 ; i < ex.scores_at_different_directions.size(); i++){
		std::cout<< ex.scores_at_different_directions[i] << " "; 
	}
	std::cout << std::endl;

	d = ex.find_margin_utilities();
	printf("%d\n", d;
	done_flag = 1; 
	ros::spin();
	return 0; 
}