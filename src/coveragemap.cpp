#include "multirov/coveragemap.hpp"
#define HEIGHT 20
#define LENGTH 20

// coveragemap::coveragemap(ros::NodeHandle& nodeHandle):nodeHandle_(nodeHandle){
coveragemap::coveragemap( ){
	covermap = Eigen::MatrixXf::Zero(LENGTH, HEIGHT);
	col = covermap.cols();
	row = covermap.rows();
	covermap2 = Eigen::MatrixXf::Zero(LENGTH, HEIGHT);
	agent_number_map = Eigen::MatrixXi::Zero(LENGTH, HEIGHT);
}

void coveragemap::set_coveragemap(){
	// step0, init the covermap2
	covermap2 = Eigen::MatrixXf::Zero(LENGTH, HEIGHT);

	// step1, place the agent
	place_agents(agents); 

	// step2, compute sector values based on No. of agents in surrounded sectors(assume agent sense 2 hops)
	calculate_coveragemap(); 
}

void coveragemap::place_agents(std::vector< std::vector <int> > a){
	agent_number_map = Eigen::MatrixXi::Zero(LENGTH, HEIGHT);
	for(int i = 0; i < a.size(); i++){
		if( eequal(covermap2( a[i][0], a[i][1] ) , 0) ){
			agent_number_map( a[i][0], a[i][1] )++;
			covermap2( a[i][0], a[i][1] ) = 2; 
		}else{
			covermap2( a[i][0], a[i][1] ) = ( covermap2( a[i][0], a[i][1] ) * agent_number_map( a[i][0], a[i][1] ) / ( agent_number_map( a[i][0], a[i][1] ) + 1) );
			agent_number_map( a[i][0], a[i][1] )++; 	 
		}
	}
	// overlapsed observing zone: dealing with center COVERAGE value
	int nearby_agents; 
	for(int i = 0 ; i < a.size(); i++){
		nearby_agents = count_nearby_agents(a[i][0], a[i][1]); 
		if( nearby_agents == 0 ){
			// covermap2(a[i][0], a[i][1]) = 2;
		}else{
			covermap2(a[i][0], a[i][1]) = float(2) / float(nearby_agents); 
		}
	}

	for(int i = col - 1; i >= 0; i--){
		for(int j = 0; j <= row - 1; j++){
			printf("%f ", covermap2(i, j)); 
		} 
		printf("\n");
	}
	printf("\n");printf("\n");printf("\n");printf("\n");printf("\n");
}

void coveragemap::calculate_coveragemap(){
	for(int i = 0; i < col; i++){
		for(int j = 0; j < row; j++){
			if(eequal( covermap2(i, j), 0) ){ // Avoid re-assign values to agents' stands
				int nearby_agents = count_nearby_agents(i, j); 
				printf("nearby agents for sector %d, %d: %d\n", i, j, nearby_agents); 
				if( nearby_agents == 0 ){
					covermap2(i, j) = 0;
				}else{
					covermap2(i, j) = float(1) / float(nearby_agents); 
				}
			}
		}
	}
} 

int coveragemap::count_nearby_agents(int x, int y){
	int nearby_agent_num = 0; 
	if(x!=0 & y!=0 & x!= col-1 & y!=row-1){
		for(int i = -1; i < 2; i++){
			for(int j = -1; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x==0 & y!=0 & x!= col-1 & y!=row-1){
		for(int i = 0; i < 2; i++){
			for(int j = -1; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x!=0 & y==0 & x!= col-1 & y!=row-1){
		for(int i = -1; i < 2; i++){
			for(int j = 0; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x!=0 & y!=0 & x== col-1 & y!=row-1){
		for(int i = -1; i < 1; i++){
			for(int j = -1; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x!=0 & y!=0 & x!= col-1 & y==row-1){
		for(int i = -1; i < 2; i++){
			for(int j = -1; j < 1; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x==0 & y==0 & x!= col-1 & y!=row-1){
		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x!=0 & y==0 & x== col-1 & y!=row-1){
		for(int i = -1; i < 1; i++){
			for(int j = 0; j < 2; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x!=0 & y!=0 & x== col-1 & y==row-1){
		for(int i = -1; i < 1; i++){
			for(int j = -1; j < 1; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}
	if(x==0 & y!=0 & x!= col-1 & y==row-1){
		for(int i = 0; i < 2; i++){
			for(int j = -1; j < 1; j++){
				if( !(i==0 & j==0) ){ 
					nearby_agent_num += agent_number_map(x+i, y+j);
				}
			}
		}
	}

	return nearby_agent_num; 
}

bool coveragemap::eequal(float a, float b){
  if((int)round(a*100) == (int)round(b*100) ){
    return true; 
  }else{
    return false;
  }
}