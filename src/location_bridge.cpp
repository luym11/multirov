#include <multirov/location_bridge.hpp>

location_bridge::location_bridge(ros::NodeHandle& nodeHandle){
}

void location_bridge::subsCallback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& p){

	// how many tags are detected in this frame? 
	std::cout << p->detections.size() << std::endl;

	int num_detected; 
	int id; 
	geometry_msgs::Point currentPoint; 
	bool flag_0=false;
	bool flag_1 = false;
	bool flag_2 = false; 
	bool flag_3 = false;
	bool flag_4 = false; 

	num_detected = p->detections.size(); 
	if(num_detected == 0){ // nothing is detected in this frame, which is the case when start to fly the DJI
		return; 
	}else{
		for(int i_Tag = 0; i_Tag < num_detected; i_Tag++){
			id = p->detections[i_Tag].id[0];
			// std::cout <<  p->detections[i_Tag].id[0] << std::endl; 
			currentPoint = p->detections[i_Tag].pose.pose.pose.position; 
			switch(id){
				case 0: {
					origin_location_point.x = currentPoint.x; 
					origin_location_point.y = currentPoint.y; 
					flag_0 = true;
					break;
				}
				case 1: {
					rov1_location_point.x = currentPoint.x; 
					rov1_location_point.y = currentPoint.y; 
					flag_1 = true; 
					break;
				}
				case 2: {
					rov2_location_point.x = currentPoint.x; 
					rov2_location_point.y = currentPoint.y; 
					flag_2 = true; 
					break;
				}
				case 3: {
					rov3_location_point.x = currentPoint.x; 
					rov3_location_point.y = currentPoint.y; 
					flag_3 = true; 
					break;
				}
				case 4: {
					x_location_point.x = currentPoint.x; 
					x_location_point.y = currentPoint.y; 
					flag_4 = true; 
					break;
				}
			}
		}
	}
	if(flag_0) origin_location_publ.publish(origin_location_point); 
	if(flag_1) rov1_location_publ.publish(rov1_location_point); 
	if(flag_2) rov2_location_publ.publish(rov2_location_point); 
	if(flag_3) rov3_location_publ.publish(rov3_location_point); 
	if(flag_4) x_location_publ.publish(x_location_point); 
	// std::cout << p->detections[1].pose.pose.pose.position.x << std::endl; 

	/*id
	resource_location_point.x = (int)round(p->x); 
	resource_location_point.y = (int)round(p->y); 
	resource_location_point.z = (int)round(p->z); 
	resource_location_publ.publish(resource_location_point); 
	ROS_INFO("Command received, x = %f y = %f z = %f", resource_location_point.x, resource_location_point.y, resource_location_point.z); 
	*/

}

