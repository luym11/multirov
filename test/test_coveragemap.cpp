#include "multirov/coveragemap.hpp"

int main(int argc, char** argv){
	coveragemap c(20, 20); 
	std::vector<int> vec(2,1); 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,1]
	vec[1] = 2;  
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,2]
	vec[0] = 6; vec[1] = 6; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [6,6]
	vec[0] = 2; vec[1] = 2; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [2,2]
	vec[0] = 1; vec[1] = 3; 
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	c.agents.push_back(vec); // [1,3]
	for (int i = 0 ; i < vec.size(); i++){
		printf("%d ", vec[i]); 
	}
	printf("\n");
	printf("\n");
	c.agents.push_back(vec); // [1,3] again
	c.set_coveragemap(); 
	for(int j = c.row - 1; j >= 0; j--){
		for(int i = 0; i <= c.col - 1; i++){
			printf("%f ", c.covermap2(i, j)); 
		} 
		printf("\n");
	}
	return 0; 
}