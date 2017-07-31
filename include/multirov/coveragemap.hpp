#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <stdio.h>

#define HEIGHT 20
#define LENGTH 20

// define agent coveragemap class here
class coveragemap{
public:
	// coveragemap(ros::NodeHandle& nodeHandle);
	coveragemap();


	// properties of the covermap
	int col;
	int row;

	// store another coveragemap for processing
	Eigen::MatrixXf covermap2;

	// agents locations, agent_size * 2 vector
	std::vector< std::vector <int> > agents; 

	// set coveragemap based on agent locations
	void set_coveragemap();

	// compare
	bool eequal(float a, float b);

private:
	/////////////////////////////////////////
	// Members
	/////////////////////////////////////////

	//ros::NodeHandle nodeHandle_;

	// store coveragemap here
	Eigen::MatrixXf covermap;	

	

	// agent number map
	Eigen::MatrixXi agent_number_map; 

	//////////////////////////////////////////
	// Methods
	//////////////////////////////////////////



	// step 0, init covermap2 as zeros

	// step 1, place all the agents as COVERAGE 2, and examine if more than 1 agents are at the same sector. If so, average the COVERAGE of that sector. 
	void place_agents(std::vector< std::vector <int> > a); 

	// step 2, traverse each sector of the covermap, calculate each sector's COVERAGE
	void calculate_coveragemap(); 

	// step 2's sub function: count how many agents are within 1 hop for one sector
	int count_nearby_agents(int x, int y); 
};

	