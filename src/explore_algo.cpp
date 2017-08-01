#include "multirov/explore_algo.hpp"


explore_algo::explore_algo(){
	my_location_local.push_back(2); my_location_local.push_back(2); 
}

void explore_algo::remap_heatmap(){
	nearby_heatmap = Eigen::MatrixXi::Zero(5, 5);
	for(int i = -2; i < 2; i++){
		for(int j = -2; j < 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				nearby_heatmap(i+2, j+2) = heatmap(my_location[0] + i, my_location[1] + j); 
			}
		}
	}
	// nearby_heatmap = heatmap.block(my_location[0] - 2, my_location[1] - 2, 5, 5); // farther than 2 to the edges
	for(int j = 4; j >= 0; j--){
		for(int i = 0; i <= 4; i++){
			printf("%d ", heatmap(i, j)); 
		} 
		printf("\n");
	}
}

void explore_algo::remap_coordinates(){
	for(int i = 0; i < agent_locations.size(); i++){
		agent_locations[i][0] = agent_locations[i][0] - my_location[0] + 2; 
		agent_locations[i][1] = agent_locations[i][1] - my_location[1] + 2; 
		if(agent_locations[i][0] >= 0 & agent_locations[i][0] < 5 & agent_locations[i][1] >= 0 & agent_locations[i][1] < 5 & not(agent_locations[i][0] == 2 & agent_locations[i][1] == 2 )){
			nearby_agent_locations_local.push_back(agent_locations[i]); 
		}
	}

}

void explore_algo::calculate_covermap(){
	coveragemap cm(5, 5); 
	cm.agents = nearby_agent_locations_local; 
	cm.agents.insert(cm.agents.begin(), my_location_local); 
	std::cout << cm.agents.size() << std::endl;
	cm.set_coveragemap(); 

	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", cm.covermap2(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}

	printf("\n"); 

	nearby_covermap = Eigen::MatrixXf::Zero(5, 5); 
	for(int i = -2; i < 2; i++){
		for(int j = -2; j < 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				nearby_covermap(i+2, j+2) = cm.covermap2(my_location_local[0] + i, my_location_local[1] + j); 
			}
		}
	}
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", nearby_covermap(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
}  