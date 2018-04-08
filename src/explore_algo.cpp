#include "multirov/explore_algo.hpp"


explore_algo::explore_algo(){
	my_location_local.push_back(2); my_location_local.push_back(2); // Always (2,2): don't change this unless you know what you're doing
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
	/*
	printf("NEARBY HEATMAP\n");
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%d ", nearby_heatmap(i+2, j+2)); 
		} 
		printf("\n");
	}
	*/
}

void explore_algo::remap_coordinates(){
	// init
	nearby_agent_locations_local.clear(); 
	agent_locations_local.clear(); 
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
	// std::cout << "calculate_covermap: covermap has agent number: " << cm.agents.size() << std::endl;
	// std::cout << "calculate_covermap: agent_locations has size: " << agent_locations.size() << std::endl;
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
	/*
	printf("NEARBY COVERMAP\n");
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", nearby_covermap(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
	*/
}  

void explore_algo::move_to_see_the_scores(){
	scores_at_different_directions.clear();
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
	//std::cout << "compute_score: covermap has agent number: " << cm.agents.size() << std::endl;
	//std::cout << "compute_score: agent_locations has size: " << agent_locations.size() << std::endl;
	cm.set_coveragemap();

	Eigen::MatrixXf _nearby_covermap = Eigen::MatrixXf::Zero(5, 5); 
	for(int i = -2; i <= 2; i++){
		for(int j = -2; j <= 2; j++){
			if(my_location[0] + i >= 0 & my_location[0] + i < 200 &  my_location[1] + j >= 0 &  my_location[1] + j < 200 ){
				_nearby_covermap(i+2, j+2) = cm.covermap2(my_location_local[0] + i, my_location_local[1] + j); 
			}
		}
	}
	/*
	printf("DIFFERENT POSITION COVERMAPS for %d %d\n", x, y);
	for(int j = 2; j >= -2; j--){
		for(int i = -2; i <= 2; i++){
			printf("%f ", _nearby_covermap(my_location_local[0]+i, my_location_local[1]+j)); 
		} 
		printf("\n");
	}
	*/
	Eigen::MatrixXf h_f = h.cast<float>();
	return (_nearby_covermap.cwiseProduct(h_f)).sum(); 
}

void explore_algo::find_margin_utilities(){
	margin_utilities_at_different_directions.clear();
	for(int i = 0; i < scores_at_different_directions.size(); i++){
		margin_utilities_at_different_directions.push_back(scores_at_different_directions[i] - scores_at_different_directions[4]); 
	}
	//for(int i = 0; i < scores_at_different_directions.size(); i++){
	// use 0.2 to be parameter for current
		if(currentAngle.x > 0){
			margin_utilities_at_different_directions[8] *= (1+currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[8], 0) ) margin_utilities_at_different_directions[8] += 2*currentAngle.x; 
			margin_utilities_at_different_directions[7] *= (1+currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[7], 0) ) margin_utilities_at_different_directions[7] += 2*currentAngle.x; 
			margin_utilities_at_different_directions[6] *= (1+currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[6], 0) ) margin_utilities_at_different_directions[6] += 2*currentAngle.x;
			margin_utilities_at_different_directions[2] *= (1-currentAngle.x*0.2);
			margin_utilities_at_different_directions[1] *= (1-currentAngle.x*0.2); 
			margin_utilities_at_different_directions[0] *= (1-currentAngle.x*0.2);
		}else if(currentAngle.x < 0){
			margin_utilities_at_different_directions[8] *= (1+currentAngle.x*0.2);
			margin_utilities_at_different_directions[7] *= (1+currentAngle.x*0.2); 
			margin_utilities_at_different_directions[6] *= (1+currentAngle.x*0.2);
			margin_utilities_at_different_directions[2] *= (1-currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[2], 0) ) margin_utilities_at_different_directions[2] -= 2*currentAngle.x; 
			margin_utilities_at_different_directions[1] *= (1-currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[1], 0) )margin_utilities_at_different_directions[1] -= 2*currentAngle.x;
			margin_utilities_at_different_directions[0] *= (1-currentAngle.x*0.2); 
			if(eequal(margin_utilities_at_different_directions[0], 0) ) margin_utilities_at_different_directions[0] -= 2*currentAngle.x;
		}else{

		}
		if(currentAngle.y > 0){
			margin_utilities_at_different_directions[2] *= (1+currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[2], 0) ) margin_utilities_at_different_directions[2] += 2*currentAngle.y;
			margin_utilities_at_different_directions[5] *= (1+currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[5], 0) ) margin_utilities_at_different_directions[5] += 2*currentAngle.y;
			margin_utilities_at_different_directions[8] *= (1+currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[8], 0) ) margin_utilities_at_different_directions[8] += 2*currentAngle.y;
			margin_utilities_at_different_directions[0] *= (1-currentAngle.y*0.2);
			margin_utilities_at_different_directions[3] *= (1-currentAngle.y*0.2); 
			margin_utilities_at_different_directions[6] *= (1-currentAngle.y*0.2);
		}else if(currentAngle.y < 0){
			margin_utilities_at_different_directions[2] *= (1+currentAngle.y*0.2); 
			margin_utilities_at_different_directions[5] *= (1+currentAngle.y*0.2); 
			margin_utilities_at_different_directions[8] *= (1+currentAngle.y*0.2);
			margin_utilities_at_different_directions[0] *= (1-currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[0], 0) ) margin_utilities_at_different_directions[0] -= 2*currentAngle.y;
			margin_utilities_at_different_directions[3] *= (1-currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[3], 0) ) margin_utilities_at_different_directions[3] -= 2*currentAngle.y;
			margin_utilities_at_different_directions[6] *= (1-currentAngle.y*0.2); 
			if(eequal(margin_utilities_at_different_directions[6], 0) ) margin_utilities_at_different_directions[6] -= 2*currentAngle.y;
		}else{

		}
	//}
}

bool explore_algo::eequal(float a, float b){
  if((int)round(a*100) == (int)round(b*100) ){
    return true; 
  }else{
    return false;
  }
}

int explore_algo::find_new_direction(){

	std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(0, 8);
 	
 	int random_direction; 
    random_direction = dis(gen);

	float Z; 
	// Binary log-linear learning has ONLY two options: stay or one randomly CHOSEN direction	
	Z = exp(margin_utilities_at_different_directions[4]) + exp(margin_utilities_at_different_directions[random_direction]);

	std::vector<float> p; 
	p.push_back(exp(margin_utilities_at_different_directions[4])/Z ); 
	p.push_back(exp(margin_utilities_at_different_directions[random_direction])/Z ); 
	
	srand(time(NULL));  
	float random_num = std::rand()/double(RAND_MAX); 
	int d; 
	if(random_num < p[0]){ 
		d = 4; // stay
	}else{
		d = random_direction; 
	}

	// prevent from bumping to the wall. 
	if(my_location[0] < 1 && (d==0 || d==1 || d==2)) d=7;
	if(my_location[0] > 8 && (d==8 || d==7 || d==6)) d=1;
	if(my_location[1] < 1 && (d==0 || d==3 || d==6)) d=5;
	if(my_location[1] > 8 && (d==2 || d==5 || d==8)) d=3;

	return d; 
}