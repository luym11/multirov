#include "multirov/explore_algo.hpp"


int main(int argc, char** argv){
	explore_algo ex; 

	ex.heatmap = Eigen::MatrixXi::Zero(200, 200); 
	ex.heatmap(0, 0) = 10; ex.heatmap(2,1) = 8; 
	ex.my_location.push_back(0); ex.my_location.push_back(1); 
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
	return 0; 
}