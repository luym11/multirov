#include "multirov/explore_algo.hpp"

int main(int argc, char** argv){
	explore_algo ex; 

	ex.heatmap = Eigen::MatrixXi::Zero(200, 200); 
	ex.heatmap(0, 0) = 10; 
	ex.my_location.push_back(0); ex.my_location.push_back(1); 
	std::vector<int> a2; a2.push_back(1); a2.push_back(1); 
	std::vector<int> a3(2,2); 
	ex.agent_locations.push_back(ex.my_location); ex.agent_locations.push_back(a2); ex.agent_locations.push_back(a3);

	ex.remap_heatmap(); 
	ex.remap_coordinates(); 
	ex.calculate_covermap(); 
	return 0; 
}