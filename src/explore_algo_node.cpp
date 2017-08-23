#include "multirov/explore_algo_node.hpp"

//extern void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p); 

//explore_algo_node::explore_algo_node(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle), c(200, 200){
explore_algo_node::explore_algo_node(ros::NodeHandle& nodeHandle):nh_1(&nodeHandle), c(200, 200){
	resource_location_subs = nh_1->subscribe("/resource_location", 5, &explore_algo_node::resource_location_Callback, this);
	done_flag = 0; 
}

Eigen::MatrixXi explore_algo_node::heatmap_update(int x, int y){
	Eigen::MatrixXi _heatmap = Eigen::MatrixXi::Zero(200, 200);
	_heatmap.block(x-3, y-3, 7, 7) = Eigen::MatrixXi::Constant(7, 7, 4); 
	_heatmap.block(x-2, y-2, 5, 5) = Eigen::MatrixXi::Constant(5, 5, 6); 
	_heatmap.block(x-1, y-1, 3, 3) = Eigen::MatrixXi::Constant(3, 3, 8); 
	_heatmap(x, y) = 10;
	return _heatmap;
}


void explore_algo_node::resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
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
	d.data = ex.find_new_direction();
	std::cout << d << std::endl;
	go_direction_publ.publish(d);
	
}


void explore_algo_node::agent_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	std::vector<int> vec(2,1); 
	vec[0] = (int)round(p->x); vec[1] = (int)round(p->y); 
	int rovnumMinus1 = (int)round(p->z)-1;
	c.agents[rovnumMinus1][0] = vec[0]; c.agents[rovnumMinus1][1] = vec[1]; 
	ex.agent_locations[rovnumMinus1][0] = vec[0]; ex.agent_locations[rovnumMinus1][1] = vec[1]; 
	//ex.my_location[0] = vec[0]; ex.my_location[1] = vec[1];
 
	std::cout << "Change of location happens at " << rovnumMinus1 + 1 << " agent" << std::endl; 
	std::cout << "The " << "1 " <<" agent's location is " << ex.agent_locations[0][0] << " " << ex.agent_locations[0][1] << ", and now we have " << ex.agent_locations.size() << " agents. "<< std::endl; 
	std::cout << "The 2 agent's location is " << ex.agent_locations[1][0] << " " << ex.agent_locations[1][1] << ", and now we have " << ex.agent_locations.size() << " agents. "<< std::endl; 
	//std::cout << "The 3 agent's location is " << ex.agent_locations[2][0] << " " << ex.agent_locations[2][1] << ", and now we have " << ex.agent_locations.size() << " agents. "<< std::endl; 
	std::cout << "now we have " << ex.agent_locations.size() << " agents. "<< std::endl; 
	std::cout << "c has agent size of " << c.agents.size() << std::endl; 
	std::cout << "nearby_agent_local has agent size of " << ex.nearby_agent_locations_local.size() << std::endl; 
	// set overall covermap by agent location list
	c.set_coveragemap(); 

	std::cout << "print nearby part of the covermap" << std::endl;

	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			if(ex.agent_locations[rovNum-1][0]+i >= 0 & ex.agent_locations[rovNum-1][1]+j >= 0 & ex.agent_locations[rovNum-1][0]+i < c.col & ex.agent_locations[rovNum-1][1]+j < c.row){
				printf("%f ", c.covermap2(ex.agent_locations[rovNum-1][0]+i, ex.agent_locations[rovNum-1][1]+j)); 
			}
		} 
		printf("\n");
	}
	// ex.my_location = ex.agent_locations[rovNum-1]; // double make sure, see line 50
	ex.my_location[0] = ex.agent_locations[rovNum-1][0]; ex.my_location[1] = ex.agent_locations[rovNum-1][1];// double make sure, see line 50
	if( (done_flag == 1) & ((rovnumMinus1+1)==rovNum) ){
		ex.remap_heatmap(); 
		ex.remap_coordinates(); 
		ex.calculate_covermap(); 
		ex.move_to_see_the_scores();
		
		for(int i = 0 ; i < ex.scores_at_different_directions.size(); i++){
			std::cout<< ex.scores_at_different_directions[i] << " "; 
		}
		std::cout << std::endl;

		ex.find_margin_utilities();
		d.data = ex.find_new_direction();
		std::cout << d << std::endl;
		go_direction_publ.publish(d);
	}
}