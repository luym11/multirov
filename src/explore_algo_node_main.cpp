#include "multirov/explore_algo_node.hpp"

explore_algo_node *ex_node;
/*
void resource_location_Callback(const geometry_msgs::Point::ConstPtr& p){
	geometry_msgs::Point resource_center; 
	resource_center.x = (int)round(p->x);
	resource_center.y = (int)round(p->y);
	resource_center.z = (int)round(p->z); 
	ex_node->ex.heatmap = heatmap_update((int)round(p->x), (int)round(p->y)); 
	// ROS_INFO("center x is %f, heatmap values are %d %d %d %d", resource_center.x, heatmap((int)round(p->x), (int)round(p->y)), heatmap((int)round(p->x)+1, (int)round(p->y)+1), heatmap((int)round(p->x)+2, (int)round(p->y)+2), heatmap((int)round(p->x)+3, (int)round(p->y)+3)); 
	// Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
	// std::cout << _heatmap.block((int)round(p->x)-4, (int)round(p->y)-4,9,9).format(HeavyFmt) << std::endl;
	
	ex_node->ex.remap_heatmap(); 
	ex_node->ex.remap_coordinates(); 
	ex_node->ex.calculate_covermap(); 
	ex_node->ex.move_to_see_the_scores();
	
	for(int i = 0 ; i < ex_node->ex.scores_at_different_directions.size(); i++){
		std::cout<< ex_node->ex.scores_at_different_directions[i] << " "; 
	}
	std::cout << std::endl;

	ex_node->ex.find_margin_utilities();
	ex_node->d.data = ex_node->ex.find_new_direction();
	std::cout << ex_node->d << std::endl;
	ex_node->go_direction_publ.publish(ex_node->d);
	
}
*/
int main(int argc, char** argv){
	ros::init(argc, argv, "explore_algo_node");
	ros::NodeHandle nh;
	
	 
	ex_node = new explore_algo_node(nh); 

	//ex_node->resource_location_subs = nh.subscribe("/resource_location", 5, &explore_algo_node::resource_location_Callback, ex_node);

	ex_node->ex.heatmap = Eigen::MatrixXi::Zero(200, 200);

	if(argc != 2){
	ROS_WARN("Warning! Didn't pass correct namespace format");
	}else{
		strcat(argv[1], "/agent_location"); 
		sscanf(argv[1], "/rexrov%d", &(ex_node->rovNum) ); 
	}
	std::cout << argv[1] << std::endl; 

	ex_node->rexrov1_location_subs = nh.subscribe("/rexrov1/agent_location", 10, &explore_algo_node::agent_location_Callback, ex_node);
	ex_node->rexrov2_location_subs = nh.subscribe("/rexrov2/agent_location", 10, &explore_algo_node::agent_location_Callback, ex_node);

	ex_node->go_direction_publ = nh.advertise<std_msgs::Int8>("/rexrov1/direction_to_go", 10);


 	// init 
	std::vector<int> a1; a1.push_back(0); a1.push_back(0); 
	std::vector<int> a2; a2.push_back(10); a2.push_back(10); 
	// std::vector<int> a3(19,29); 
	ex_node->ex.agent_locations.push_back(a1); ex_node->ex.agent_locations.push_back(a2); // ex_node->ex.agent_locations.push_back(a3);
	// For we only have 2 agents now. This now has to be written by hard code
	ex_node->ex.my_location.push_back(ex_node->ex.agent_locations[ex_node->rovNum-1][0]); ex_node->ex.my_location.push_back(ex_node->ex.agent_locations[ex_node->rovNum-1][1]); 
	
	//init c
	ex_node->c.agents.push_back(a1); ex_node->c.agents.push_back(a2); // ex_node->c.agents.push_back(a3);

	ex_node->ex.remap_heatmap(); 
	ex_node->ex.remap_coordinates(); 
	ex_node->ex.calculate_covermap(); 
	ex_node->ex.move_to_see_the_scores();
	
	for(int i = 0 ; i < ex_node->ex.scores_at_different_directions.size(); i++){
		std::cout<< ex_node->ex.scores_at_different_directions[i] << " "; 
	}
	std::cout << std::endl;

	ex_node->ex.find_margin_utilities();
	ex_node->d.data = ex_node->ex.find_new_direction();
	std::cout << ex_node->d.data << std::endl;
	ex_node->go_direction_publ.publish(ex_node->d);

	ex_node->done_flag = 1; 
	ros::spin();

	return 0; 
}