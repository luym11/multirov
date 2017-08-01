#ifndef EXPLORE_ALGO
#define EXPLORE_ALGO
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <stdio.h>
#include "multirov/coveragemap.hpp"

class explore_algo{

public:
	explore_algo(); 

	// int* find_a_incremental_to_go(); 

	// receive whole heatmap and keep nearby 
	void remap_heatmap(); 

	// recalculate coordinates of the agents, and only keep those nearby in nearby_agent_locations
	void remap_coordinates(); 

	// calculate nearby_covermap
	void calculate_covermap(); 


	std::vector<int> my_location; 
	std::vector<int> my_location_local; 
	std::vector< std::vector<int> > agent_locations; 
	std::vector< std::vector<int> > agent_locations_local; 
	Eigen::MatrixXi heatmap; 
	
private:
	Eigen::MatrixXf nearby_covermap; 
	Eigen::MatrixXi nearby_heatmap; 
	std::vector< std::vector<int> > nearby_agent_locations_local; 
};

#endif