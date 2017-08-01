#include "multirov/explore_algo.hpp"


explore_algo::explore_algo(){
	my_location_local.push_back(2); my_location_local.push_back(2); 
}

void explore_algo::remap_heatmap(){
	nearby_heatmap = Eigen::MatrixXi::Zero(5, 5);
	for(int i = -2; i <= 2; i++){
		for(int j = -2; j <= 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				nearby_heatmap(i+2, j+2) = heatmap(my_location[0] + i, my_location[1] + j); 
			}
		}
	}
	// nearby_heatmap = heatmap.block(my_location[0] - 2, my_location[1] - 2, 5, 5); // farther than 2 to the edges
	printf("NEARBY HEATMAP\n");
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%d ", nearby_heatmap(i+2, j+2)); 
		} 
		printf("\n");
	}
}

void explore_algo::remap_coordinates(){
	// init
	std::vector<int> v(2,-1);
	for(int i = 0; i < agent_locations.size(); i++){
		agent_locations_local.push_back(v); 
	}

	for(int i = 0; i < agent_locations.size(); i++){
		agent_locations_local[i][0] = agent_locations[i][0] - my_location[0] + 2; 
		agent_locations_local[i][1] = agent_locations[i][1] - my_location[1] + 2; 
		if(agent_locations_local[i][0] >= 0 & agent_locations_local[i][0] < 5 & agent_locations_local[i][1] >= 0 & agent_locations_local[i][1] < 5 & not(agent_locations_local[i][0] == 2 & agent_locations_local[i][1] == 2 )){
			nearby_agent_locations_local.push_back(agent_locations_local[i]); 
		}
	}

}

void explore_algo::calculate_covermap(){
	coveragemap cm(5, 5); 
	cm.agents = nearby_agent_locations_local; 
	cm.agents.insert(cm.agents.begin(), my_location_local); 
	// std::cout << cm.agents.size() << std::endl;
	cm.set_coveragemap(); 
/*
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", cm.covermap2(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
	printf("\n"); 
*/
	nearby_covermap = Eigen::MatrixXf::Zero(5, 5); 
	for(int i = -2; i <= 2; i++){
		for(int j = -2; j <= 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				nearby_covermap(i+2, j+2) = cm.covermap2(my_location_local[0] + i, my_location_local[1] + j); 
			}
		}
	}
	printf("NEARBY COVERMAP\n");
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", nearby_covermap(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
}  

void explore_algo::move_to_see_the_scores(){
	for(int i = -1; i <= 1; i++){
		for(int j = -1; j <= 1; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				scores_at_different_directions.push_back( compute_score(my_location_local[0] + i, my_location_local[1] + j, nearby_heatmap) ); 
			}else{
				scores_at_different_directions.push_back(-1000); 
			}
			
		}
	}
} 

float explore_algo::compute_score(int x, int y, Eigen::MatrixXi h){
	coveragemap cm(5, 5); 
	cm.agents = nearby_agent_locations_local; 
	std::vector<int> me; me.push_back(x); me.push_back(y); 
	cm.agents.insert(cm.agents.begin(), me); 
	// std::cout << cm.agents.size() << std::endl;
	cm.set_coveragemap();

	Eigen::MatrixXf _nearby_covermap = Eigen::MatrixXf::Zero(5, 5); 
	for(int i = -2; i <= 2; i++){
		for(int j = -2; j <= 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				_nearby_covermap(i+2, j+2) = cm.covermap2(my_location_local[0] + i, my_location_local[1] + j); 
			}
		}
	}
	printf("DIFFERENT POSITION COVERMAPS for %d %d\n", x, y);
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", _nearby_covermap(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
	Eigen::MatrixXf h_f = h.cast<float>();
	return (_nearby_covermap.cwiseProduct(h_f)).sum(); 
}

void explore_algo::find_margin_utilities(){
	for(int i = 0; i < scores_at_different_directions.size(); i++){
		margin_utilities_at_different_directions.push_back(scores_at_different_directions[i] - scores_at_different_directions[4]); 
	}
}

int explore_algo::find_new_direction(){
	float Z; 
	for(int i = 0; i < margin_utilities_at_different_directions.size(); i++){
		Z = Z + exp(margin_utilities_at_different_directions[i]); 
	}
	std::vector<float> p; 
	for(int i = 0; i < margin_utilities_at_different_directions.size(); i++){
		p.push_back(exp(margin_utilities_at_different_directions[i])/Z ); 
	}
	std::srand((unsigned)time(NULL)); 
	float random_num = std::rand()/double(RAND_MAX); 
	int d; 
	while(random_num >= 0){
		random_num -= p[d]; 
		d++;
	}
	return d; 
}